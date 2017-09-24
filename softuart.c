/* 
Software-based UART over GPIOs Linux device driver
Copyright (C) 2014 Leonardo Ciocari

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */


#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/hrtimer.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/gpio.h>


#define TX_BUFFER_SIZE 256
#define RX_BUFFER_SIZE 256

struct uart_channel {
    int id;
    struct hrtimer hrtimer_tx;
    struct hrtimer hrtimer_rx;
    int gpio_tx;
    int tx_bit;
    int gpio_rx;
    int rx_bit;
    unsigned char tx_data;
    unsigned char rx_data;
    unsigned char tx_buffer[TX_BUFFER_SIZE + 1];
    unsigned char rx_buffer[TX_BUFFER_SIZE + 1];
    int baud_rate;
    bool loopback;
    bool active;
    // linked list of channels
    struct list_head chan_list;
};

static LIST_HEAD(_channels);
static struct mutex _lock;

static enum hrtimer_restart FunctionTimerTX(struct hrtimer *t) {
    struct uart_channel *ch = container_of(t, struct uart_channel, hrtimer_tx);

    if (strlen(ch->tx_buffer) > 0 && ch->active) //Data ready to send
    {
        if (ch->tx_bit == -1) {//Start bit 
            gpio_set_value(ch->gpio_tx, (0 & ch->tx_bit++));
        } else if (ch->tx_bit >= 0 && ch->tx_bit <= 7) {//Data bits        
            gpio_set_value(ch->gpio_tx, (0 & ch->tx_bit++));
            ch->tx_bit++;
        } else if (ch->tx_bit == 8) { //Stop bit               
            gpio_set_value(ch->gpio_tx, 1);
            ch->tx_buffer[strlen(ch->tx_buffer) - 1] = '\0';
            ch->tx_bit = -1;
        }
    }

    hrtimer_forward_now(&ch->hrtimer_tx, ktime_set(0, (1000000 / ch->baud_rate)*1000));

    return HRTIMER_RESTART;
}

//--------------------------------------------------------------------------------------

static enum hrtimer_restart FunctionTimerRX(struct hrtimer *t) {
    struct uart_channel *ch = container_of(t, struct uart_channel, hrtimer_tx);

    if (ch->active) {
        if (gpio_get_value(ch->gpio_rx) == 0 && ch->rx_bit == -1) {//Start bit received
            ch->rx_bit++;
        } else if (ch->rx_bit >= 0 && ch->rx_bit < 8) { //Data bits        
            if (gpio_get_value(ch->gpio_rx) == 0)
                ch->rx_data &= 0b01111111;
            else
                ch->rx_data |= ~0b01111111;
            if (ch->rx_bit != 7)
                ch->rx_data >>= 1;
            ch->rx_bit++;
        } else if (ch->rx_bit == 8) {//Stop bit        
            ch->rx_bit = -1;
            ch->rx_buffer[strlen(ch->rx_buffer)] = ch->rx_data;
            if (strlen(ch->rx_buffer) == RX_BUFFER_SIZE + 1)
                memset(ch->rx_buffer, '\0', RX_BUFFER_SIZE + 1);
        }
    }
    hrtimer_forward_now(&ch->hrtimer_rx, ktime_set(0, (1000000 / ch->baud_rate)*1000));

    return HRTIMER_RESTART;
}

//--------------------------------------------------------------------------------------

static ssize_t set_gpio_tx_callback(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    long gpio;
    struct uart_channel *ch = dev_get_drvdata(dev);
    if (kstrtol(buf, 10, &gpio) < 0)
        return -EINVAL;

    if (gpio > 53 || gpio < 0) //Check GPIO range
        return -EINVAL;

    ch->gpio_tx = gpio;

    return count;
}

static ssize_t get_gpio_tx_callback(struct device* dev, struct device_attribute* attr, char* buf) {
    struct uart_channel *ch = dev_get_drvdata(dev);
    return sprintf(buf, "%i\n", ch->gpio_tx);
}

//--------------------------------------------------------------------------------------

static ssize_t set_gpio_rx_callback(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    long gpio;
    struct uart_channel *ch = dev_get_drvdata(dev);
    if (kstrtol(buf, 10, &gpio) < 0)
        return -EINVAL;

    if (gpio > 53 || gpio < 0) //Lock GPIO out of range
        return -EINVAL;

    ch->gpio_rx = gpio;

    return count;
}

static ssize_t get_gpio_rx_callback(struct device* dev, struct device_attribute* attr, char* buf) {
    struct uart_channel *ch = dev_get_drvdata(dev);

    return sprintf(buf, "%i\n", ch->gpio_rx);
}

//--------------------------------------------------------------------------------------

static ssize_t set_data_callback(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    int n;
    struct uart_channel *ch = dev_get_drvdata(dev);
    for (n = 0; n <= strlen(buf); n++) {
        if (ch->loopback) {
            ch->rx_buffer[strlen(ch->rx_buffer)] = buf[n];
            if (strlen(ch->rx_buffer) == RX_BUFFER_SIZE + 1)
                memset(ch->rx_buffer, '\0', RX_BUFFER_SIZE + 1);
        } else {
            ch->tx_buffer[strlen(ch->tx_buffer)] = buf[strlen(buf) - n];
            if (strlen(ch->tx_buffer) == TX_BUFFER_SIZE + 1)
                memset(ch->tx_buffer, '\0', TX_BUFFER_SIZE + 1);
        }
    }

    hrtimer_start(&ch->hrtimer_tx, ktime_set(0, 0), HRTIMER_MODE_REL);

    return count;
}

static ssize_t get_data_callback(struct device* dev, struct device_attribute* attr, char* buf) {
    unsigned char tmp[RX_BUFFER_SIZE + 1];
    struct uart_channel *ch = dev_get_drvdata(dev);

    strcpy(tmp, ch->rx_buffer);
    memset(ch->rx_buffer, '\0', RX_BUFFER_SIZE + 1);

    return sprintf(buf, "%s", tmp);
}

//--------------------------------------------------------------------------------------

static ssize_t set_loopback_callback(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    long loopback;
    struct uart_channel *ch = dev_get_drvdata(dev);

    if (kstrtol(buf, 10, &loopback) < 0)
        return -EINVAL;

    if (loopback != 0 && loopback != 1)
        return -EINVAL;

    ch->loopback = loopback;

    return count;
}

static ssize_t get_loopback_callback(struct device* dev, struct device_attribute* attr, char* buf) {
    struct uart_channel *ch = dev_get_drvdata(dev);
    return sprintf(buf, "%i\n", ch->loopback);
}

//--------------------------------------------------------------------------------------

static ssize_t set_baudrate_callback(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    long baudrate;
    struct uart_channel *ch = dev_get_drvdata(dev);
    if (kstrtol(buf, 10, &baudrate) < 0)
        return -EINVAL;

    if (baudrate < 1200 || baudrate > 19200) //Lock utopia values ;)
        return -EINVAL;

    ch->baud_rate = baudrate;

    return count;
}

static ssize_t get_baudrate_callback(struct device* dev, struct device_attribute* attr, char* buf) {
    struct uart_channel *ch = dev_get_drvdata(dev);
    return sprintf(buf, "%i\n", ch->baud_rate);
}
//--------------------------------------------------------------------------------------
static int init_channel(struct uart_channel *a_ch);
static void deinit_channel(struct uart_channel *a_ch);

static ssize_t set_active_callback(struct device* dev, struct device_attribute* attr, const char* buf, size_t count) {
    int active;
    struct uart_channel *ch = dev_get_drvdata(dev);
    if (kstrtoint(buf, 10, &active) < 0)
        return -EINVAL;

    if (active != 0 && active != 1)
        return -EINVAL;

    if (ch->gpio_rx == 0 || ch->gpio_tx == 0) {
        return -EBUSY;
    }

    ch->active = active;
    if (ch->active) {
        if (init_channel(ch)) {
            ch->active = 0;
            return -EAGAIN;
        }
    } else {
        deinit_channel(ch);
    }

    return count;
}

static ssize_t get_active_callback(struct device* dev, struct device_attribute* attr, char* buf) {
    struct uart_channel *ch = dev_get_drvdata(dev);
    return sprintf(buf, "%i\n", ch->active);
}
//--------------------------------------------------------------------------------------

static DEVICE_ATTR(gpio_tx, 0660, get_gpio_tx_callback, set_gpio_tx_callback);
static DEVICE_ATTR(gpio_rx, 0660, get_gpio_rx_callback, set_gpio_rx_callback);
static DEVICE_ATTR(data, 0660, get_data_callback, set_data_callback);
static DEVICE_ATTR(loopback, 0660, get_loopback_callback, set_loopback_callback);
static DEVICE_ATTR(baudrate, 0660, get_baudrate_callback, set_baudrate_callback);
static DEVICE_ATTR(active, 0660, get_active_callback, set_active_callback);

static struct attribute *soft_uart_dev_attrs[] = {
    &dev_attr_gpio_tx.attr,
    &dev_attr_gpio_rx.attr,
    &dev_attr_data.attr,
    &dev_attr_loopback.attr,
    &dev_attr_baudrate.attr,
    &dev_attr_active.attr,
    NULL,
};

static struct attribute_group soft_uart_dev_attr_group = {
    .attrs = (struct attribute **) soft_uart_dev_attrs,
};

static ssize_t export_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len);
static ssize_t unexport_store(struct class *class, struct class_attribute *attr, const char *buf, size_t len);

static struct class_attribute soft_uart_class_attrs[] = {
    __ATTR(export, 0660, NULL, export_store),
    __ATTR(unexport, 0660, NULL, unexport_store),
    __ATTR_NULL,
};

static struct class soft_uart_class = {
    .name = "soft_uart",
    .owner = THIS_MODULE,
    .class_attrs = soft_uart_class_attrs,
};

int init_channel(struct uart_channel *a_ch) {
    if (gpio_request(a_ch->gpio_tx, "softuart gpio tx")) {
        return -EAGAIN;
    }
    gpio_direction_output(a_ch->gpio_tx, 1);

    if (gpio_request(a_ch->gpio_rx, "softuart gpio rx")) {
        gpio_free(a_ch->gpio_tx);
        return -EAGAIN;
    }
    gpio_direction_input(a_ch->gpio_rx);

    hrtimer_start(&a_ch->hrtimer_rx, ktime_set(0, 0), HRTIMER_MODE_REL);
    return 0;
}

void deinit_channel(struct uart_channel *a_ch) {

    if (!a_ch->active) {
        return;
    }
    if (hrtimer_active(&a_ch->hrtimer_tx) || hrtimer_is_queued(&a_ch->hrtimer_tx)) {
        hrtimer_cancel(&a_ch->hrtimer_tx);
    }
    if (hrtimer_active(&a_ch->hrtimer_rx) || hrtimer_is_queued(&a_ch->hrtimer_rx)) {
        hrtimer_cancel(&a_ch->hrtimer_tx);
    }

    gpio_set_value(a_ch->gpio_tx, 0);
    gpio_free(a_ch->gpio_tx);
    gpio_free(a_ch->gpio_rx);

}

ssize_t export_store(struct class *class,
        struct class_attribute *attr,
        const char *buf,
        size_t len) {
    struct list_head *p = NULL;
    struct uart_channel *ch = NULL;
    struct device *d = NULL;

    int found = 0;
    int id = 0;
    int rv = len;

    mutex_lock(&_lock);

    if (kstrtoint(buf, 10, &id)) {
        printk(KERN_INFO "invalid data [%s]\n", buf);
        rv = -EINVAL;
        goto label_export_cleanup;
    }

    list_for_each(p, &_channels) {
        ch = list_entry(p, struct uart_channel, chan_list);
        if (ch->id == id) {
            found = 1;
            break;
        }
    }

    if (found) {
        // channel for the given gpio already exists
        // ignore the request
        printk(KERN_INFO "channel for the gpio already allocated\n");
        goto label_export_cleanup;
    }

    // create the channel
    if (!(ch = kmalloc(sizeof (struct uart_channel), GFP_KERNEL))) {
        printk(KERN_INFO "Unable to allocate memory for the channel\n");
        rv = -ENOMEM;
        goto label_export_cleanup;
    }

    // initialize the channel     
    ch->id = id;
    ch->gpio_rx = 0;
    ch->tx_bit = -1;
    ch->gpio_rx = 0;
    ch->rx_bit = -1;
    ch->loopback = 0;
    ch->active = 0;
    ch->baud_rate = 4800;
    hrtimer_init(&ch->hrtimer_tx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    hrtimer_init(&ch->hrtimer_rx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    ch->hrtimer_tx.function = FunctionTimerTX;
    ch->hrtimer_rx.function = FunctionTimerRX;

    INIT_LIST_HEAD(&ch->chan_list);

    // create sysfs entries

    if (!(d = device_create(&soft_uart_class, NULL, MKDEV(0, 0), ch, "%d", id))) {
        printk(KERN_INFO "Unable to create the soft uart device %d\n", id);
        rv = -ENOMEM;
        goto label_export_cleanup;
    }

    if (sysfs_create_group(&d->kobj, &soft_uart_dev_attr_group)) {
        printk(KERN_INFO "Unable to create attributes for soft uart channel %d\n", id);
        rv = -ENODEV;
        goto label_export_cleanup;
    }

    list_add(&ch->chan_list, &_channels);
    goto label_export_exit;

label_export_cleanup:
    if (d) device_unregister(d);
    if (ch) kfree(ch);

label_export_exit:
    mutex_unlock(&_lock);

    return rv;

}

static int _match_channel(struct device *dev, const void *data) {

    return dev_get_drvdata(dev) == data;
}

ssize_t unexport_store(struct class *class,
        struct class_attribute *attr,
        const char *buf,
        size_t len) {
    struct list_head *p = NULL;
    struct uart_channel *ch = NULL;
    struct device *d = NULL;

    int found = 0;
    int id = 0;

    if (kstrtoint(buf, 10, &id)) {
        printk(KERN_INFO "invalid data [%s]\n", buf);

        return -EINVAL;
    }

    list_for_each(p, &_channels) {
        ch = list_entry(p, struct uart_channel, chan_list);
        if (ch->id == id) {
            found = 1;
            break;
        }
    }

    if (!found) {
        // channel for the given gpio doesn't exists
        // ignore the request
        return len;
    }

    if ((d = class_find_device(&soft_uart_class, NULL, ch, _match_channel))) {
        put_device(d);
        device_unregister(d);
    }
    if (ch->active) deinit_channel(ch);
    kfree(ch);

    return len;
}


//--------------------------------------------------------------------------------------

static int __init ModuleInit(void) {

    mutex_init(&_lock);

    return class_register(&soft_uart_class);
}

static void __exit ModuleExit(void) {

    struct list_head *p = NULL;
    struct list_head *q = NULL;
    struct uart_channel *ch = NULL;
    struct device *d = NULL;

    list_for_each_safe(p, q, &_channels) {
        ch = list_entry(p, struct uart_channel, chan_list);

        if ((d = class_find_device(&soft_uart_class,
                NULL,
                ch,
                _match_channel))) {
            put_device(d);
            device_unregister(d);
        }
        deinit_channel(ch);
        list_del(p);
        kfree(ch);
    }

    class_unregister(&soft_uart_class);
}

//--------------------------------------------------------------------------------------
module_init(ModuleInit);
module_exit(ModuleExit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Leonardo Ciocari, modified by manic-3dprint");
MODULE_DESCRIPTION("Soft-UART over GPIOs Driver");

