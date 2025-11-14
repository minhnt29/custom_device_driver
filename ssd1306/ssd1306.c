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




module_i2c_driver(ssd1306_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("minhnt29");
MODULE_DESCRIPTION("SSD1306 OLED Display Driver");
MODULE_VERSION("1.0");

// End of ssd1306.c