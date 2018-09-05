#ifndef __AK09911_H_
#define __AK09911_H_

//by CAD pin setting
#define AK09911_I2C_CAD0_ADD            0x0C
#define AK09911_I2C_CAD1_ADD            0x0D

#define AK09911_REG_WIA1		0x00
#define AK09911_REG_WIA2		0x01
#define AK09911_REG_INFO1               0x02
#define AK09911_REG_INFO2               0x03
#define AK09911_REG_ST1			0x10
#define AK09911_REG_HXL			0x11
#define AK09911_REG_HXH			0x12
#define AK09911_REG_HYL			0x13
#define AK09911_REG_HYH			0x14
#define AK09911_REG_HZL			0x15
#define AK09911_REG_HZH			0x16
#define AK09911_REG_ST2                 0x18
#define AK09911_REG_CNTL1		0x30 //dummy register
#define AK09911_REG_CNTL2		0x31 //set device operation mode
#define AK09911_REG_CNTL3		0x32 //software reset register
#define AK09911_REG_ASAX		0x60
#define AK09911_REG_ASAY		0x61
#define AK09911_REG_ASAZ		0x62

#define AK09911_WIA1_VALUE		0x48
#define AK09911_WIA2_VALUE		0x05

#define AK09911_MODE_SNG_MEASURE	0x01
#define AK09911_MODE_SELF_TEST		0x10
#define AK09911_MODE_FUSE_ACCESS	0x1F
#define AK09911_MODE_POWERDOWN		0x00

#define AK09911_ST1_HSM_MASK            0x80
#define AK09911_ST1_DOR_MASK            0x02
#define AK09911_ST1_DRDY_MASK           0x01

#define AK09911_ST2_HOLF_MASK           0x08

#define AK09911_RESET_DATA_MASK		0x01

#define AK09911_RAW_TO_GAUSS(asa)	((((asa) + 128) * 6000) / 256)
#define AK09911_MAX_CONVERSION_TIMEOUT_MS	500
#define AK09911_CONVERSION_DONE_POLL_TIME_MS	10

#endif
