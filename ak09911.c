#include <linux/module.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/timer.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/platform_device.h>
#include <asm/mach/map.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "ak09911.h"
#include <linux/sysfs.h>
#define DRV_VERSION "V1.0"
static struct i2c_client *ak09911_client;

static ssize_t ak09911_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"CH0_data = %d, CH1_data = %d, Prox_data = %d \n",CH0_data, CH1_data, Prox_data);
}
static ssize_t ak09911_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	return 0;
}
static DEVICE_ATTR(data, 0644 , ak09911_show, NULL);
static struct attribute *ak09911_attrs[] = {
    &dev_attr_data.attr,
    NULL
};
static struct attribute_group mydrv_attr_group = {
    .name = "ak09911",
    .attrs = ak09911_attrs,
};

static int ak09911_dev_init(void)
{
	int res;
	printk("%s called\n", __func__);
	res = i2c_smbus_write_byte_data(ak09911_client,, 0);
	mdelay(12);
	//read device id
	res = i2c_smbus_read_byte_data(ak09911_client, REPEATED_BYTE | APDS9930_ID);
	/*
	if(res == ) {
		printk("the device id:%x \n", res);
	}
	*/
	return 0;
}
static int ak09911_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	//struct proc_dir_entry *file;
	int ret;
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL))
                return -ENODEV;

	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");
	ak09911_client = i2c;
	ak09911_dev_init();
	printk("ak09911 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &mydrv_attr_group);
	return 0;
}
static int ak09911_remove(struct i2c_client *i2c)
{
	sysfs_remove_group(&i2c->dev.kobj, &mydrv_attr_group);
	return 0;
}

static struct of_device_id ak09911_of_match[] = {
        { .compatible = "akm,ak09911" },
        { },
};

struct i2c_driver ak09911_driver = {
    .driver = {
        .name           = "ak09911",
        .owner          = THIS_MODULE,
        .of_match_table = of_match_ptr(ak09911_of_match),
    },
    .probe      = ak09911_probe,
    .remove     = ak09911_remove,
};

static int ak09911_init(void)
{
	return i2c_add_driver(&ak09911_driver);
}

static void ak09911_exit(void)
{
	printk("exit ak09911 driver module");
	i2c_del_driver(&ak09911_driver);
}

module_init(ak09911_init);
module_exit(ak09911_exit);

//module_i2c_driver(ak09911_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-ak09911 driver for testing module ");
MODULE_VERSION("V1.0");
