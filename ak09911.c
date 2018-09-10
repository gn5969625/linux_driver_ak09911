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

/*
 * For AK09911, same calculation, but the device is less sensitive:
 *
 * H is in the range of +-8190.  The magnetometer has a range of
 * +-4912uT.  To go from the raw value to uT is:
 *
 * HuT = H * 4912/8190, or roughly, 6/10,
 */
// raw data convert to uT and multiple asa data to caculate the final data
static long ak09911_raw_to_gauss(u16 asa)
{
        return (((long)data + 128) * 6000) / 256;
}

static struct i2c_client *ak09911_client;
typedef struct akm09911_data {
       struct i2c_client *client;
       struct input_dev *input_dev;
       struct work_struct work;
       unsigned char asa[3]; //get Sensitivity Adjustment Values from fuse rom
       long raw_to_gauss[3]; //final scale to real data,ex: x_raw_data*raw_to_gauss[0]/10^6 (unit:gauss)
       struct mutex lock;
       unsigned char mode;
}akm09911_data;

static void set_akm09911_mode(akm09911_data *data,unsigned char mode) {
     int res;
     data->mode = mode;
     rest = i2c_smbus_write_byte_data(data->client,AK09911_REG_CNTL2,data->mode);
}
static int i2c_ak09911_read_len(struct i2c_client *client,unsigned char reg_addr,unsigned char len,unsigned char *buf)
{
        int ret;
        unsigned char txbuf = reg_addr;
        struct i2c_msg msg[] = {
                {client->addr,0,1,&txbuf},
                {client->addr,1,len,buf}
        };
        ret = i2c_transfer(client->adapter,msg,2);
        if(ret < 0) {
                printk("i2c_transfer read len error\n");
                return -1;
        }
        return 0;
}

//static akm09911_data akm_data;
void get_akm09911_raw_data(akm09911_data *data,unsigned char raw_data[]) {
     int ret;
     set_akm09911_mode(data,AK09911_MODE_SNG_MEASURE);
     do {
       ret = i2c_smbus_read_byte_data(data->client,AK09911_REG_ST2);
     }while(!(ret & AK09911_ST1_DRDY_MASK ));
     i2c_ak09911_read_len(data->client,AK09911_REG_HXLsiAK09911_REG_HXL,6,raw_data);
     
}
static ssize_t ak09911_mode_show(struct device *dev,struct device_attribute *attr, char *buf)
{
        akm09911_data *data = dev_get_drvdata(dev);
	return sprintf(buf ,"mode = %x\n",data->mode);
}
static ssize_t ak09911_mode_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
        akm09911_data *data = dev_get_drvdata(dev);
        data->mode =  (unsigned char)simple_strtoul(buf,NULL, 0);
	set_akm09911_mode(data,mode);
        return 0;
}
static DEVICE_ATTR(ak09911_data, 0644 , ak09911_mode_show, ak09911_mode_show);
static struct attribute *ak09911_attrs[] = {
    &dev_attr_ak09911_data.attr,
    NULL
};
static struct attribute_group mydrv_attr_group = {
    .name = "ak09911",
    .attrs = ak09911_attrs,
};
static void set_akm09911_reset(akm09911_data *data) {
     int ret;
     ret = i2c_smbus_write_byte_data(data->client,AK09911_REG_CNTL3,AK09911_RESET_DATA_MASK);
}

static void get_akm09911_asa(akm09911_data *data) {
     int res;
     //enter fuse rom mode
     res = i2c_smbus_write_byte_data(data->client,AK09911_REG_CNTL2,AK09911_MODE_FUSE_ACCESS);
     mdelay(100);
     data->asa[0] = i2c_smbus_read_byte_data(data->client,AK09911_REG_ASAX);
     data->asa[1] = i2c_smbus_read_byte_data(data->client,AK09911_REG_ASAY);
     data->asa[2] = i2c_smbus_read_byte_data(data->client,AK09911_REG_ASAZ);
    
     data->raw_to_gauss[0] = ak8963_09911_raw_to_gauss(data->asa[0]);
     data->raw_to_gauss[1] = ak8963_09911_raw_to_gauss(data->asa[1]);
     data->raw_to_gauss[2] = ak8963_09911_raw_to_gauss(data->asa[2]);
     //return power dowm mode
     res = i2c_smbus_write_byte_data(data->client,AK09911_REG_CNTL2,AK09911_MODE_POWERDOWN);
}
static void set_ak09911_input_dev(akm09911_data *data) {
        data->input_dev = input_alloc_device();
        set_bit(EV_ABS, data->input_dev->evbit);
        /* x-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_X, -AKM_09911_MAX_RANGE, AKM_09911_MAX_RANGE, 0, 0);
        /* y-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_Y, -AKM_09911_MAX_RANGE, AKM_09911_MAX_RANGE, 0, 0);
        /* z-axis magnetic */
        input_set_abs_params(data->input_dev, ABS_Z, -AKM_09911_MAX_RANGE, AKM_09911_MAX_RANGE, 0, 0);
        data->input_dev->name = "akm09911";
        data->input_dev->dev.parent = &data->client->dev;
        res = input_register_device(data->input_dev);
}
static int ak09911_dev_init(akm09911_data *data) {
	int res;
	printk("%s called\n", __func__);
	res = i2c_smbus_read_byte_data(data->client, AK09911_REG_WIA1);
        if(res < 0)
	   return 0;
        if(res == AK09911_WIA1_VALUE)
            printk("%s,Campany ID:%x\n",__func__,res);
        res = i2c_smbus_read_byte_data(data->client, AK09911_REG_WIA2);
	if(res < 0)
           return 0;
        if(res == AK09911_WIA2_VALUE)
             printk("%s,Device ID:%x\n",__func__,res);

        get_akm09911_asa(data);
        set_akm09911_mode(data,AK09911_MODE_POWERDOWN);
        set_ak09911_input_dev(data);

}

static int ak09911_probe(struct i2c_client *i2c, const struct i2c_device_id *id) {
	int ret;
        akm09911_data *data = NULL;
	dev_dbg(&i2c->dev, "%s\n", __func__);
	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL))
                return -ENODEV;

	dev_info(&i2c->dev, "chip found, driver version " DRV_VERSION "\n");
        data = kzalloc(sizeof(akm09911),GFP_KERNEL);
	ak09911_client = i2c;
        data->clinet = i2c;
        mutext_init(&data->lock);
	ak09911_dev_init(data);
	printk("ak09911 device component found!~\n");
	ret = sysfs_create_group(&i2c->dev.kobj, &mydrv_attr_group);
        i2c_set_clientdata(i2c,data);
	return 0;
}

static int ak09911_remove(struct i2c_client *i2c) {
        akm09911_data *data = i2c_get_clientdata(i2c);
        input_unregister_device(data->input_dev);
        input_free(data->input_dev);
        mutex_destroy(&data->lock);
        kfree(data);
	sysfs_remove_group(&i2c->dev.kobj, &mydrv_attr_group);
	return 0;
}
/*
static int akm09911_open(struct inode *inode, struct file *file) {
        return 0;
}

static int akm09911_release(struct inode *inode, struct file *file) {
        return 0;
}

static long akm09911_ioctl( struct file *file, unsigned int cmd,unsigned long arg) {
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
*/
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
module_i2c_driver(ak09911_driver);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kevin.Shen");
MODULE_DESCRIPTION("A i2c-ak09911 driver for testing module ");
MODULE_VERSION(DRV_VERSION);
