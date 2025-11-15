/*
 * SSD1306 OLED Display Driver
 * I2C Interface
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define DRIVER_NAME "ssd1306"
#define DRIVER_CLASS "ssd1306Class"

/* SSD1306 Commands */
#define SSD1306_DISPLAY_OFF           0xAE
#define SSD1306_DISPLAY_ON            0xAF
#define SSD1306_SET_DISPLAY_CLOCK     0xD5
#define SSD1306_SET_MULTIPLEX         0xA8
#define SSD1306_SET_DISPLAY_OFFSET    0xD3
#define SSD1306_SET_START_LINE        0x40
#define SSD1306_CHARGE_PUMP           0x8D
#define SSD1306_MEMORY_MODE           0x20
#define SSD1306_SEG_REMAP             0xA1
#define SSD1306_COM_SCAN_DEC          0xC8
#define SSD1306_SET_COM_PINS          0xDA
#define SSD1306_SET_CONTRAST          0x81
#define SSD1306_SET_PRECHARGE         0xD9
#define SSD1306_SET_VCOM_DETECT       0xDB
#define SSD1306_DISPLAY_ALL_ON_RESUME 0xA4
#define SSD1306_NORMAL_DISPLAY        0xA6
#define SSD1306_COLUMN_ADDR           0x21
#define SSD1306_PAGE_ADDR             0x22

#define SSD1306_WIDTH  128
#define SSD1306_HEIGHT 64
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * SSD1306_HEIGHT / 8)


/* Device structure */
struct ssd1306_dev {
    struct i2c_client *client;
    struct cdev cdev;
    dev_t dev_num;
    struct class *class;
    struct device *device;
    u8 *buffer;
    struct mutex lock;
};

static struct ssd1306_dev *ssd1306_device;

/* Write command to SSD1306 */
static int ssd1306_write_cmd(struct i2c_client *client, u8 cmd)
{
    u8 buf[2] = {0x00, cmd}; // Control byte + command
    int ret;
    
    ret = i2c_master_send(client, buf, 2);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to send command 0x%02x\n", cmd);
        return ret;
    }
    return 0;
}

/* Write data to SSD1306 */
static int ssd1306_write_data(struct i2c_client *client, u8 *data, size_t len)
{
    u8 *buf;
    int ret;
    
    buf = kmalloc(len + 1, GFP_KERNEL);
    if (!buf)
        return -ENOMEM;
    
    buf[0] = 0x40; // Data control byte
    memcpy(buf + 1, data, len);
    
    ret = i2c_master_send(client, buf, len + 1);
    kfree(buf);
    
    if (ret < 0) {
        dev_err(&client->dev, "Failed to send data\n");
        return ret;
    }
    return 0;
}

/* Initialize SSD1306 display */
static int ssd1306_init_display(struct i2c_client *client)
{
    int ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_DISPLAY_OFF);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SET_DISPLAY_CLOCK);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x80);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SET_MULTIPLEX);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x3F);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SET_DISPLAY_OFFSET);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x00);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SET_START_LINE | 0x00);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_CHARGE_PUMP);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x14);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_MEMORY_MODE);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x00);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SEG_REMAP);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_COM_SCAN_DEC);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SET_COM_PINS);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x12);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SET_CONTRAST);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0xCF);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SET_PRECHARGE);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0xF1);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_SET_VCOM_DETECT);
    if (ret) return ret;
    ret = ssd1306_write_cmd(client, 0x40);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_DISPLAY_ALL_ON_RESUME);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_NORMAL_DISPLAY);
    if (ret) return ret;
    
    ret = ssd1306_write_cmd(client, SSD1306_DISPLAY_ON);
    if (ret) return ret;
    
    dev_info(&client->dev, "SSD1306 initialized successfully\n");
    return 0;
}

/* Clear display */
static int ssd1306_clear_display(struct ssd1306_dev *dev)
{
    memset(dev->buffer, 0, SSD1306_BUFFER_SIZE);
    
    ssd1306_write_cmd(dev->client, SSD1306_COLUMN_ADDR);
    ssd1306_write_cmd(dev->client, 0);
    ssd1306_write_cmd(dev->client, SSD1306_WIDTH - 1);
    
    ssd1306_write_cmd(dev->client, SSD1306_PAGE_ADDR);
    ssd1306_write_cmd(dev->client, 0);
    ssd1306_write_cmd(dev->client, 7);
    
    return ssd1306_write_data(dev->client, dev->buffer, SSD1306_BUFFER_SIZE);
}

/* File operations */
static int ssd1306_open(struct inode *inode, struct file *file)
{
    file->private_data = ssd1306_device;
    return 0;
}

static int ssd1306_release(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t ssd1306_write(struct file *file, const char __user *buf, 
                              size_t count, loff_t *ppos)
{
    struct ssd1306_dev *dev = file->private_data;
    size_t to_write;
    int ret;
    
    if (*ppos >= SSD1306_BUFFER_SIZE)
        return 0;
    
    to_write = min(count, (size_t)(SSD1306_BUFFER_SIZE - *ppos));
    
    mutex_lock(&dev->lock);
    
    if (copy_from_user(dev->buffer + *ppos, buf, to_write)) {
        mutex_unlock(&dev->lock);
        return -EFAULT;
    }
    
    /* Update display */
    ssd1306_write_cmd(dev->client, SSD1306_COLUMN_ADDR);
    ssd1306_write_cmd(dev->client, 0);
    ssd1306_write_cmd(dev->client, SSD1306_WIDTH - 1);
    
    ssd1306_write_cmd(dev->client, SSD1306_PAGE_ADDR);
    ssd1306_write_cmd(dev->client, 0);
    ssd1306_write_cmd(dev->client, 7);
    
    ret = ssd1306_write_data(dev->client, dev->buffer, SSD1306_BUFFER_SIZE);
    
    mutex_unlock(&dev->lock);
    
    if (ret < 0)
        return ret;
    
    *ppos += to_write;
    return to_write;
}

static ssize_t ssd1306_read(struct file *file, char __user *buf,
                             size_t count, loff_t *ppos)
{
    struct ssd1306_dev *dev = file->private_data;
    size_t to_read;
    
    if (*ppos >= SSD1306_BUFFER_SIZE)
        return 0;
    
    to_read = min(count, (size_t)(SSD1306_BUFFER_SIZE - *ppos));
    
    mutex_lock(&dev->lock);
    
    if (copy_to_user(buf, dev->buffer + *ppos, to_read)) {
        mutex_unlock(&dev->lock);
        return -EFAULT;
    }
    
    mutex_unlock(&dev->lock);
    
    *ppos += to_read;
    return to_read;
}

static const struct file_operations ssd1306_fops = {
    .owner = THIS_MODULE,
    .open = ssd1306_open,
    .release = ssd1306_release,
    .read = ssd1306_read,
    .write = ssd1306_write,
};

/* I2C probe function */
static int ssd1306_probe(struct i2c_client *client,
                         const struct i2c_device_id *id)
{
    int ret;
    
    dev_info(&client->dev, "SSD1306 probe started\n");
    
    ssd1306_device = kzalloc(sizeof(struct ssd1306_dev), GFP_KERNEL);
    if (!ssd1306_device)
        return -ENOMEM;
    
    ssd1306_device->buffer = kzalloc(SSD1306_BUFFER_SIZE, GFP_KERNEL);
    if (!ssd1306_device->buffer) {
        kfree(ssd1306_device);
        return -ENOMEM;
    }
    
    ssd1306_device->client = client;
    mutex_init(&ssd1306_device->lock);
    
    /* Initialize display */
    ret = ssd1306_init_display(client);
    if (ret) {
        dev_err(&client->dev, "Failed to initialize display\n");
        goto err_init;
    }
    
    /* Clear display */
    ssd1306_clear_display(ssd1306_device);
    
    /* Allocate character device */
    ret = alloc_chrdev_region(&ssd1306_device->dev_num, 0, 1, DRIVER_NAME);
    if (ret) {
        dev_err(&client->dev, "Failed to allocate chrdev region\n");
        goto err_init;
    }
    
    /* Initialize cdev */
    cdev_init(&ssd1306_device->cdev, &ssd1306_fops);
    ret = cdev_add(&ssd1306_device->cdev, ssd1306_device->dev_num, 1);
    if (ret) {
        dev_err(&client->dev, "Failed to add cdev\n");
        goto err_cdev;
    }
    
    /* Create device class */
    ssd1306_device->class = class_create(THIS_MODULE, DRIVER_CLASS);
    if (IS_ERR(ssd1306_device->class)) {
        dev_err(&client->dev, "Failed to create class\n");
        ret = PTR_ERR(ssd1306_device->class);
        goto err_class;
    }
    
    /* Create device */
    ssd1306_device->device = device_create(ssd1306_device->class, NULL,
                                           ssd1306_device->dev_num, NULL,
                                           DRIVER_NAME);
    if (IS_ERR(ssd1306_device->device)) {
        dev_err(&client->dev, "Failed to create device\n");
        ret = PTR_ERR(ssd1306_device->device);
        goto err_device;
    }
    
    i2c_set_clientdata(client, ssd1306_device);
    
    dev_info(&client->dev, "SSD1306 driver loaded successfully\n");
    return 0;

err_device:
    class_destroy(ssd1306_device->class);
err_class:
    cdev_del(&ssd1306_device->cdev);
err_cdev:
    unregister_chrdev_region(ssd1306_device->dev_num, 1);
err_init:
    kfree(ssd1306_device->buffer);
    kfree(ssd1306_device);
    return ret;
}

/* I2C remove function */
static void ssd1306_remove(struct i2c_client *client)
{
    struct ssd1306_dev *dev = i2c_get_clientdata(client);
    
    device_destroy(dev->class, dev->dev_num);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->dev_num, 1);
    
    kfree(dev->buffer);
    kfree(dev);
    
    dev_info(&client->dev, "SSD1306 driver removed\n");
    return;
}

/* I2C device ID table */
static const struct i2c_device_id ssd1306_id[] = {
    {
        "ssd1306", 0
    },
    {}
};
MODULE_DEVICE_TABLE(i2c, ssd1306_id);

/* Device tree compatible */
static const struct of_device_id ssd1306_of_match[] = {
    {
        .compatible = "minhnt29,ssd1306" 
    },
    {}
};
MODULE_DEVICE_TABLE(of, ssd1306_of_match);

static struct i2c_driver ssd1306_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = ssd1306_of_match,
    },
    .probe = ssd1306_probe,
    .remove = ssd1306_remove,
    .id_table = ssd1306_id,
};


module_i2c_driver(ssd1306_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("minhnt29");
MODULE_DESCRIPTION("SSD1306 OLED Display Driver");
MODULE_VERSION("1.0");

// End of ssd1306.c