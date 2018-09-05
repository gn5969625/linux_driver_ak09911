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
#define DRV_VERSION "V2.0"

#define AKM_09911_MAX_RANGE 8190
static struct i2c_client *ak09911_client;
typedef struct akm09911_data {
       struct input_dev *input_dev;
       struct work_struct work;
       unsigned char asa[3]; //get Sensitivity Adjustment Values from fuse rom
       struct mutex lock;
       unsigned char mode;
}akm09911_data;
akm09911_data akm_data;

static ssize_t ak09911_show(struct device *dev,struct device_attribute *attr, char *buf)
{
	return 0;
}
static ssize_t ak09911_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
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
void set_akm09911_reset() {
    
}
void set_akm09911_mode() {
     int res;
     akm_data.mode = AK09911_MODE_POWERDOWN;
     rest = i2c_smbus_write_byte_data(ak09911_client,AK09911_REG_CNTL2,akm_data.mode);
}
void get_akm09911_asa(void) {
     int res;
     //enter fuse rom mode
     res = i2c_smbus_write_byte_data(ak09911_client,AK09911_REG_CNTL2,AK09911_MODE_FUSE_ACCESS);
     mdelay(100);
     akm_data.asa[0] = i2c_smbus_read_byte_data(ak09911_client,AK09911_REG_ASAX);
     akm_data.asa[1] = i2c_smbus_read_byte_data(ak09911_client,AK09911_REG_ASAY);
     akm_data.asa[2] = i2c_smbus_read_byte_data(ak09911_client,AK09911_REG_ASAZ);

     //return power dowm mode
     res = res = i2c_smbus_write_byte_data(ak09911_client,AK09911_REG_CNTL2,AK09911_MODE_POWERDOWN);
}
static int ak09911_dev_init(void)
{
	int res;
	printk("%s called\n", __func__);
	res = i2c_smbus_read_byte_data(ak09911_client, AK09911_REG_WIA1);
        if(res < 0)
	   return 0;
        if(res == AK09911_WIA1_VALUE)
            printk("%s,Campany ID:%x\n",__func__,res);
        res = i2c_smbus_read_byte_data(ak09911_client, AK09911_REG_WIA2);
	if(res < 0)
           return 0;
        if(res == AK09911_WIA2_VALUE)
             printk("%s,Device ID:%x\n",__func__,res);

        get_akm09911_asa();
        set_mode();


        akm_data.input_dev = input_alloc_device();
        set_bit(EV_ABS, akm_data.input_dev->evbit);
        /* x-axis magnetic */
        input_set_abs_params(akm_data.input_dev, ABS_X, -AKM_09911_MAX_RANGE, AKM_09911_MAX_RANGE, 0, 0);
        /* y-axis magnetic */
        input_set_abs_params(akm_data.input_dev, ABS_Y, -AKM_09911_MAX_RANGE, AKM_09911_MAX_RANGE, 0, 0);
        /* z-axis magnetic */
        input_set_abs_params(akm_data.input_dev, ABS_Z, -AKM_09911_MAX_RANGE, AKM_09911_MAX_RANGE, 0, 0);
        akm_data.input_dev->name = "akm09911";
        //akm_data.input_dev->dev.parent = &client->dev;
        res = input_register_device(akm_data.input_dev);

}
static int ak09911_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
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

static struct file_operations akm09911_fops = {
        .owner = THIS_MODULE,
        .open = akm09911_open,
        .release = akm09911_release,
        .ioctl = akm09911_ioctl,
};

static struct miscdevice akm09911_device = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "akm09911_device",
        .fops = &akm09911_fops,
};

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
/*
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
*/
module_i2c_driver(ak09911_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-ak09911 driver for testing module ");
MODULE_VERSION("V1.0");
