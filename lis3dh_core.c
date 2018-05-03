/******************** (C) COPYRIGHT 2016 STMicroelectronics ********************
 *
 * File Name          : lis3dh_acc.c
 * Authors            : AMS - Motion Mems Division - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Denis Ciocca (denis.ciocca@st.com)
 *		      : Mario Tesi <mario.tesi@st.com>
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.14
 * Date               : 2016/Apr/26
 * Description        : LIS3DH accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 *******************************************************************************/
/*******************************************************************************
Version History.
 Revision 1.0.6 15/11/2010
  first revision
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7
 Revision 1.0.9: 2011/May/23
  update_odr func correction;
 Revision 1.0.10: 2011/Aug/16
  introduces default_platform_data, i2c_read and i2c_write function rewritten,
  manages smbus beside i2c
 Revision 1.0.11: 2012/Jan/09
  moved under input/misc
 Revision 1.0.12: 2012/Feb/29
  moved use_smbus inside status struct; modified:-update_fs_range;-set_range
  input format; allows gpio_intX to be passed as parameter at insmod time;
 Revision 1.0.13: 2013/Feb/25
  modified acc_remove function;
 Revision 1.0.14: 2016/Apr/26
  added new i2c and spi interface
******************************************************************************/

#include <linux/version.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/poll.h>   //poll  
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>

#include "lis3dh.h"
#include "lis3dhlib.h"

#define DRIVER_VER1     0
#define DRIVER_VER2     4
#define DRV_MAJOR       104
#define DRV_MINOR       1

#undef	DEBUG
//#define DEBUG
#ifdef DEBUG
#define	DPRINTK( x... )		printk("lis3dh_drv: " x)
#else
#define DPRINTK( x... )
#endif



#define LIS3DH_EN_OPEN_CLOSE

#define	G_MAX			16000

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/* Accelerometer Sensor Operating Mode */
#define LIS3DH_ACC_ENABLE	0x01
#define LIS3DH_ACC_DISABLE	0x00

#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_LIS3DH_ACC	0x33	/*	Expctd content for WAI	*/

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

#define CTRL_STATUS_REG		0x27
#define ZYXDA_MASK		0x08

#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/


#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/


#define ENABLE_HIGH_RESOLUTION	1
#define ALL_ZEROES		0x00

#define LIS3DH_ACC_FDS          0x80
#define LIS3DH_ACC_HP_IA1       0x01

#define LIS3DH_ACC_ZL           0x20
#define LIS3DH_ACC_YL           0x08
#define LIS3DH_ACC_XL           0x02

#define LIS3DH_ACC_PM_OFF		0x00
#define LIS3DH_ACC_ENABLE_ALL_AXES	0x07
#define LIS3DH_ACC_EN_INT1_DRDY		0x10
#define LIS3DH_ACC_I1_IA1               0x40

#define LIS3DH_ACC_ODR1		0x10  /* 1Hz output data rate */
#define LIS3DH_ACC_ODR10	0x20  /* 10Hz output data rate */
#define LIS3DH_ACC_ODR25	0x30  /* 25Hz output data rate */
#define LIS3DH_ACC_ODR50	0x40  /* 50Hz output data rate */
#define LIS3DH_ACC_ODR100	0x50  /* 100Hz output data rate */
#define LIS3DH_ACC_ODR200	0x60  /* 200Hz output data rate */
#define LIS3DH_ACC_ODR400	0x70  /* 400Hz output data rate */
#define LIS3DH_ACC_ODR1250	0x90  /* 1250Hz output data rate */

#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01
/* */
/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG4_BDU_ENABLE	0x80
#define	CTRL_REG4_BDU_MASK	0x80
#define CTRL_REG5_LIR_INT1      0x08
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02
/* */
#define NO_MASK			0xFF
#define INT1_DURATION_MASK	0x7F
#define	INT1_THRESHOLD_MASK	0x7F
#define TAP_CFG_MASK		0x3F
#define	TAP_THS_MASK		0x7F
#define	TAP_TLIM_MASK		0x7F
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK


/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01

#define	FUZZ			0
#define	FLAT			0

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16


#define VERSION1
#define VERSION2


/* end RESUME STATE INDICES */

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis3dh_acc_odr_table[] = {
		{    1, LIS3DH_ACC_ODR1250 },
		{    3, LIS3DH_ACC_ODR400  },
		{    5, LIS3DH_ACC_ODR200  },
		{   10, LIS3DH_ACC_ODR100  },
		{   20, LIS3DH_ACC_ODR50   },
		{   40, LIS3DH_ACC_ODR25   },
		{  100, LIS3DH_ACC_ODR10   },
		{ 1000, LIS3DH_ACC_ODR1    },
};

static int int1_gpio = LIS3DH_ACC_DEFAULT_INT1_GPIO;
static int int2_gpio = LIS3DH_ACC_DEFAULT_INT2_GPIO;
static struct lis3dh_acc_status *lis3dh_ptr = NULL;


module_param(int1_gpio, int, S_IRUGO);
module_param(int2_gpio, int, S_IRUGO);

static struct lis3dh_acc_platform_data default_lis3dh_acc_pdata = {
	.fs_range = LIS3DH_ACC_G_2G,
	.axis_map_x = 0,
	.axis_map_y = 1,
	.axis_map_z = 2,
	.negate_x = 0,
	.negate_y = 0,
	.negate_z = 0,
	.poll_interval = 100,
	.min_interval = LIS3DH_ACC_MIN_POLL_PERIOD_MS,
};

static int lis3dh_acc_hw_init(struct lis3dh_acc_status *stat)
{
	int err = -1;
	u8 buf[5];

	pr_info("%s: hw init start\n", LIS3DH_ACC_DEV_NAME);

	buf[0] = stat->resume_state[RES_CTRL_REG1];
	err = stat->tf->write(stat, CTRL_REG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TEMP_CFG_REG];
	err = stat->tf->write(stat, TEMP_CFG_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_FIFO_CTRL_REG];
	err = stat->tf->write(stat, FIFO_CTRL_REG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TT_THS];
	buf[1] = stat->resume_state[RES_TT_LIM];
	buf[2] = stat->resume_state[RES_TT_TLAT];
	buf[3] = stat->resume_state[RES_TT_TW];
	err = stat->tf->write(stat, TT_THS, 4, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_TT_CFG];
	err = stat->tf->write(stat, TT_CFG, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_THS1];
	buf[1] = stat->resume_state[RES_INT_DUR1];
	err = stat->tf->write(stat, INT_THS1, 2, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_INT_CFG1];
	err = stat->tf->write(stat, INT_CFG1, 1, buf);
	if (err < 0)
		goto err_resume_state;

	buf[0] = stat->resume_state[RES_CTRL_REG2];
	buf[1] = stat->resume_state[RES_CTRL_REG3];
	buf[2] = stat->resume_state[RES_CTRL_REG4];
	buf[3] = stat->resume_state[RES_CTRL_REG5];
	buf[4] = stat->resume_state[RES_CTRL_REG6];
	err = stat->tf->write(stat, CTRL_REG2, 5, buf);
	if (err < 0)
		goto err_resume_state;

	stat->hw_initialized = 1;
	pr_info("%s: hw init done\n", LIS3DH_ACC_DEV_NAME);

	return 0;

	stat->hw_working = 0;

err_resume_state:
	stat->hw_initialized = 0;
	dev_err(stat->dev, "hw init error: %d\n", err);

	return err;
}

static void lis3dh_acc_device_power_off(struct lis3dh_acc_status *stat)
{
	int err;
	u8 buf = LIS3DH_ACC_PM_OFF;

	err = stat->tf->write(stat, CTRL_REG1, 1, &buf);
	if (err < 0)
		dev_err(stat->dev, "soft power off failed: %d\n", err);

	if (stat->pdata->power_off) {
		stat->pdata->power_off();
		stat->hw_initialized = 0;
	}

	if (stat->hw_initialized)
		stat->hw_initialized = 0;
}

static int lis3dh_acc_device_power_on(struct lis3dh_acc_status *stat)
{
	int err = -1;

	if (stat->pdata->power_on) {
		err = stat->pdata->power_on();
		if (err < 0) {
			dev_err(stat->dev,
				"power_on failed: %d\n", err);

			return err;
		}
	}

	if (!stat->hw_initialized) {
		err = lis3dh_acc_hw_init(stat);
		if (stat->hw_working == 1 && err < 0) {
			lis3dh_acc_device_power_off(stat);

			return err;
		}
	}

	return 0;
}

static int lis3dh_acc_update_fs_range(struct lis3dh_acc_status *stat,
				      u8 new_fs_range)
{
	int err = -1;
	u8 sensitivity;
	u8 buf[2];
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS3DH_ACC_FS_MASK | HIGH_RESOLUTION;

	switch (new_fs_range) {
	case LIS3DH_ACC_G_2G:
		sensitivity = SENSITIVITY_2G;
		break;
	case LIS3DH_ACC_G_4G:
		sensitivity = SENSITIVITY_4G;
		break;
	case LIS3DH_ACC_G_8G:
		sensitivity = SENSITIVITY_8G;
		break;
	case LIS3DH_ACC_G_16G:
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(stat->dev, "invalid fs range requested: %u\n",
			new_fs_range);
		return -EINVAL;
	}

	/* Updates configuration register 4, which contains fs range setting */
	err = stat->tf->read(stat, CTRL_REG4, 1, buf);
	if (err < 0)
		goto error;

	init_val = buf[0];
	stat->resume_state[RES_CTRL_REG4] = init_val;
	new_val = new_fs_range | HIGH_RESOLUTION;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	err = stat->tf->write(stat, CTRL_REG4, 1, &updated_val);
	if (err < 0)
		goto error;

	stat->resume_state[RES_CTRL_REG4] = updated_val;
	stat->sensitivity = sensitivity;

	return err;

error:
	dev_err(stat->dev,
		"update fs range failed 0x%02x,0x%02x: %d\n",
		buf[0], buf[1], err);

	return err;
}

static int lis3dh_acc_update_odr(struct lis3dh_acc_status *stat,
				 int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis3dh_acc_odr_table) - 1; i >= 0; i--) {
		if ((lis3dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms) ||
		    (i == 0))
			break;
	}
	config = lis3dh_acc_odr_table[i].mask | LIS3DH_ACC_ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&stat->enabled)) {
		err = stat->tf->write(stat, CTRL_REG1, 1, &config);
		if (err < 0)
			goto error;
	}
	stat->resume_state[RES_CTRL_REG1] = config;

	return err;

error:
	dev_err(stat->dev, "update odr failed 0x%02x,0x%02x: %d\n",
		CTRL_REG1, config, err);

	return err;
}

static int lis3dh_acc_get_acceleration_data(struct lis3dh_acc_status *stat,
					    int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };

	err = stat->tf->read(stat, AXISDATA_REG, 6, acc_data);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * stat->sensitivity;
	hw_d[1] = hw_d[1] * stat->sensitivity;
	hw_d[2] = hw_d[2] * stat->sensitivity;

	xyz[0] = ((stat->pdata->negate_x) ? (-hw_d[stat->pdata->axis_map_x])
		   : (hw_d[stat->pdata->axis_map_x]));
	xyz[1] = ((stat->pdata->negate_y) ? (-hw_d[stat->pdata->axis_map_y])
		   : (hw_d[stat->pdata->axis_map_y]));
	xyz[2] = ((stat->pdata->negate_z) ? (-hw_d[stat->pdata->axis_map_z])
		   : (hw_d[stat->pdata->axis_map_z]));

	return err;
}


static int lis3dh_acc_enable(struct lis3dh_acc_status *stat)
{
	int err;

	if (!atomic_cmpxchg(&stat->enabled, 0, 1)) {
		err = lis3dh_acc_device_power_on(stat);
		if (err < 0) {
			atomic_set(&stat->enabled, 0);

			return err;
		}
		lis3dh_acc_update_odr(stat, stat->pdata->poll_interval);
		enable_irq(stat->irq);
	}

	return 0;
}

static int lis3dh_acc_disable(struct lis3dh_acc_status *stat)
{
	if (atomic_cmpxchg(&stat->enabled, 1, 0)) {
		lis3dh_acc_device_power_off(stat);
		disable_irq_nosync(stat->irq);
	}

	return 0;
}

static int lis3dh_acc_validate_pdata(struct lis3dh_acc_status *stat)
{
	/* checks for correctness of minimal polling period */
	stat->pdata->min_interval =
		max((unsigned int)LIS3DH_ACC_MIN_POLL_PERIOD_MS,
		    stat->pdata->min_interval);

	stat->pdata->poll_interval = max(stat->pdata->poll_interval,
					 stat->pdata->min_interval);

	if (stat->pdata->axis_map_x > 2 || stat->pdata->axis_map_y > 2 ||
	    stat->pdata->axis_map_z > 2) {
		dev_err(stat->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", stat->pdata->axis_map_x,
			stat->pdata->axis_map_y, stat->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (stat->pdata->negate_x > 1 || stat->pdata->negate_y > 1 ||
	    stat->pdata->negate_z > 1) {
		dev_err(stat->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", stat->pdata->negate_x,
			stat->pdata->negate_y, stat->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (stat->pdata->poll_interval < stat->pdata->min_interval) {
		dev_err(stat->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static irqreturn_t lis3dh_acc_save_timestamp(int irq, void *private)
{
	struct lis3dh_acc_status *stat = (struct lis3dh_acc_status *)private;

	disable_irq_nosync(irq);
	stat->timestamp = lis3dh_acc_get_time_ns();

	return IRQ_WAKE_THREAD;
}

static irqreturn_t lis3dh_acc_thread_fn(int irq, void *private)
{
    unsigned char data = 0;
	struct lis3dh_acc_status *stat = (struct lis3dh_acc_status *)private;

	mutex_lock(&stat->lock);
    //clear irq
    stat->tf->read(stat, INT_SRC1, 1, &data);
    wake_up_interruptible(&stat->drv_waitq);

	mutex_unlock(&stat->lock);

	enable_irq(irq);
	return IRQ_HANDLED;
}

static int drv_open(struct inode * inode, struct file * filp)  
{
    int ret = 0;
    int irq = 0;

    if(NULL == lis3dh_ptr)
    {
        ret = -ENOMEM;
        goto error;
    }

    if(!atomic_dec_and_test(&lis3dh_ptr->opened))
    {
        ret = -EBUSY;
        goto error; 
    }
        
    irq = lis3dh_ptr->irq;
    
	ret = request_threaded_irq(irq, lis3dh_acc_save_timestamp, lis3dh_acc_thread_fn,
                                  IRQF_TRIGGER_HIGH, lis3dh_ptr->name, lis3dh_ptr);
	if (ret < 0)
		goto error;

	disable_irq_nosync(irq);

error:
    
    return ret;  
}  
  
static ssize_t drv_read(struct file *file, char __user *user, size_t size,loff_t *ppos)  
{
    int ret = 0;
    int xyz[3] = {0};
    
    wait_event_interruptible(lis3dh_ptr->drv_waitq, (lis3dh_ptr->drv_dataflag == 1));
    lis3dh_ptr->drv_dataflag = 0;
    
    mutex_lock(&lis3dh_ptr->lock);
    
    ret = lis3dh_acc_get_acceleration_data(lis3dh_ptr, xyz);
	if (ret < 0)
		dev_err(lis3dh_ptr->dev, "get_acceleration_data failed\n");
	else{
       	put_user(xyz[0], user);
    	put_user(xyz[0], (user + 4));
    	put_user(xyz[0], (user + 8));
        ret = 12;
    }
    
    mutex_unlock(&lis3dh_ptr->lock);
    
    return ret;
}  
static ssize_t drv_write(struct file *file, const char __user *buf, size_t count, loff_t *f_pos)
{
    int ret = 0;
    
    return ret;  
}
 
static int drv_release(struct inode *inode, struct file *file)  
{
    free_irq(lis3dh_ptr->irq, (void *)lis3dh_ptr);
    
    atomic_inc(&lis3dh_ptr->opened);
    return 0;  
}

static long drv_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    switch(cmd){
        case SET_ODR:
        {
            unsigned long interval_ms = arg;
            mutex_lock(&lis3dh_ptr->lock);
        	lis3dh_ptr->pdata->poll_interval = interval_ms;
        	lis3dh_acc_update_odr(lis3dh_ptr, interval_ms);
        	mutex_unlock(&lis3dh_ptr->lock);
            break;
        }
        case SET_FS:
        {
        	u8 range = (u8)arg;

        	switch (range) {
        	case 2:
        		range = LIS3DH_ACC_G_2G;
        		break;
        	case 4:
        		range = LIS3DH_ACC_G_4G;
        		break;
        	case 8:
        		range = LIS3DH_ACC_G_8G;
        		break;
        	case 16:
        		range = LIS3DH_ACC_G_16G;
        		break;
        	default:
        		dev_err(lis3dh_ptr->dev, "invalid range request: %u, discarded\n", range);

        		return -EINVAL;
        	}
        	mutex_lock(&lis3dh_ptr->lock);
        	ret = lis3dh_acc_update_fs_range(lis3dh_ptr, range);
        	if (ret < 0) {
        		mutex_unlock(&lis3dh_ptr->lock);

        		return ret;
        	}
        	lis3dh_ptr->pdata->fs_range = range;
        	mutex_unlock(&lis3dh_ptr->lock);
            break;
        }
        case SET_ENABLE:
        {
            if(arg)
            {
                lis3dh_acc_enable(lis3dh_ptr);
            }else{
                lis3dh_acc_disable(lis3dh_ptr);
            }
            break;
        }        
    }
    
    return ret;
}

static unsigned int drv_poll(struct file *file, poll_table *wait)  
{  
    unsigned int mask = 0;  
   
    poll_wait(file, &lis3dh_ptr->drv_waitq, wait);  
  
    if(1 == lis3dh_ptr->drv_dataflag)  
    {  
        mask |= POLLIN | POLLRDNORM; 
    }  
    
    return mask;    
}


/* File operations struct for character device */  
static const struct file_operations drv_fops = {  
    .owner      = THIS_MODULE,
    .open       = drv_open,
    .read       = drv_read,
    .write      = drv_write,
    .release    = drv_release,
    .unlocked_ioctl = drv_ioctl,
    .poll       = drv_poll,
};  


/*
 * struct lis3dh_acc_status *stat is allocated/freed in tf probing
 * so let it manage this stuff
 */
int lis3dh_acc_probe(struct lis3dh_acc_status *stat, int irq)
{

	int err = -1;
	u8 wai = 0;

    printk(KERN_ERR "%s-V%d.%d\n", stat->name, DRIVER_VER1, DRIVER_VER2);
	dev_info(stat->dev, "probe start.\n");

	mutex_init(&stat->lock);
	mutex_init(&stat->tb.buf_lock);
    init_waitqueue_head(&stat->drv_waitq);
    stat->drv_dataflag = 0;
	/* Check device ID and bus connection */
	err = stat->tf->read(stat, WHO_AM_I, 1, &wai);
	if (err < 0) {
		dev_warn(stat->dev, "Error reading WHO_AM_I:"
			 " is device available/working?\n");

		return err;
	}

	if (wai != WHOAMI_LIS3DH_ACC) {
		dev_err(stat->dev,
			"device unknown. Expected: 0x%02x,"
			" Replies: 0x%02x\n", WHOAMI_LIS3DH_ACC, wai);

		return -ENODEV;
	}

	mutex_lock(&stat->lock);
	
	stat->hw_working = 1;
	stat->pdata = kmalloc(sizeof(struct lis3dh_acc_platform_data), GFP_KERNEL);
	if (!stat->pdata) {
		err = -ENOMEM;
		dev_err(stat->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err_mutexunlock;
	}

	if (stat->dev->platform_data == NULL) {
		memcpy(stat->pdata, &default_lis3dh_acc_pdata,
		       sizeof(*stat->pdata));
		dev_info(stat->dev, "using default plaform_data\n");
	} else {
		memcpy(stat->pdata, stat->dev->platform_data,
		       sizeof(*stat->pdata));
	}

	err = lis3dh_acc_validate_pdata(stat);
	if (err < 0) {
		dev_err(stat->dev, "failed to validate platform data\n");
		goto exit_kfree_pdata;
	}

	if (stat->pdata->init) {
		err = stat->pdata->init();
		if (err < 0) {
			dev_err(stat->dev, "init failed: %d\n", err);
			goto err_pdata_init;
		}
	}

	memset(stat->resume_state, 0, ARRAY_SIZE(stat->resume_state));
    /*enable xyz axis*/
	stat->resume_state[RES_CTRL_REG1] = (ALL_ZEROES |
					     LIS3DH_ACC_ENABLE_ALL_AXES);
    /*data from internal filter send to output, High-pass filter on interrupt 1*/
	stat->resume_state[RES_CTRL_REG2] = (LIS3DH_ACC_FDS | LIS3DH_ACC_HP_IA1);
    /*enable AOI interrupt 1*/
	stat->resume_state[RES_CTRL_REG3] = LIS3DH_ACC_I1_IA1;
    /*output registers not updated until MSB and LSB reading*/
	stat->resume_state[RES_CTRL_REG4] = (ALL_ZEROES | CTRL_REG4_BDU_ENABLE);
    /*interupt latched*/
	stat->resume_state[RES_CTRL_REG5] = (ALL_ZEROES | CTRL_REG5_LIR_INT1);
	
    /*the thresold value of interrupt 16*16 mg*/
    stat->resume_state[RES_INT_THS1] = (ALL_ZEROES | 0xF);
        
    /**/
    stat->resume_state[RES_INT_CFG1] = (ALL_ZEROES | LIS3DH_ACC_ZL | LIS3DH_ACC_YL | LIS3DH_ACC_XL);


	err = lis3dh_acc_device_power_on(stat);
	if (err < 0) {
		dev_err(stat->dev, "power on failed: %d\n", err);
		goto err_pdata_init;
	}

	atomic_set(&stat->enabled, 1);

	err = lis3dh_acc_update_fs_range(stat, stat->pdata->fs_range);
	if (err < 0) {
		dev_err(stat->dev, "update_fs_range failed\n");
		goto  err_power_off;
	}

	err = lis3dh_acc_update_odr(stat, stat->pdata->poll_interval);
	if (err < 0) {
		dev_err(stat->dev, "update_odr failed\n");
		goto  err_power_off;
	}

    //allocate the char device
    stat->drv_dev_num = MKDEV(DRV_MAJOR, 0);
    err = register_chrdev_region (stat->drv_dev_num, DRV_MINOR, stat->name);
    if (err){
        goto error;
    }

    //register the char device
    cdev_init(&stat->drv_cdev, &drv_fops);
    stat->drv_cdev.owner = THIS_MODULE;
    err = cdev_add(&stat->drv_cdev, stat->drv_dev_num, DRV_MINOR);
    if (err) {
        goto error;
    }
    
    //create device node
    stat->drv_class = class_create(THIS_MODULE, stat->name);
    if (stat->drv_class == NULL) {
        err = -ENOMEM;
        goto error;
    }

    device_create(stat->drv_class, NULL, stat->drv_dev_num, NULL, stat->name);
    
	lis3dh_acc_device_power_off(stat);

	/* As default, do not report information */
	atomic_set(&stat->enabled, 0);

    atomic_set(&stat->opened, 1);

	mutex_unlock(&stat->lock);

	dev_info(stat->dev, "%s: probed\n", LIS3DH_ACC_DEV_NAME);

	return 0;

error:
err_power_off:
	lis3dh_acc_device_power_off(stat);
err_pdata_init:
	if (stat->pdata->exit)
		stat->pdata->exit();
exit_kfree_pdata:
	kfree(stat->pdata);
err_mutexunlock:
	mutex_unlock(&stat->lock);
	pr_err("%s: Driver Init failed\n", LIS3DH_ACC_DEV_NAME);

	return err;
}
EXPORT_SYMBOL(lis3dh_acc_probe);

int lis3dh_acc_remove(struct lis3dh_acc_status *stat)
{
	dev_info(stat->dev, "driver removing\n");

	lis3dh_acc_disable(stat);

    device_destroy(stat->drv_class, stat->drv_dev_num);
    class_destroy(stat->drv_class);
    cdev_del(&stat->drv_cdev);
    unregister_chrdev_region(stat->drv_dev_num, DRV_MINOR);
	//lis3dh_acc_input_cleanup(stat);

	//remove_sysfs_interfaces(stat->dev);

	if (stat->pdata->exit)
		stat->pdata->exit();
	kfree(stat->pdata);
	kfree(stat);

    lis3dh_ptr = NULL;
	return 0;
}
EXPORT_SYMBOL(lis3dh_acc_remove);

#ifdef CONFIG_PM
int lis3dh_acc_common_resume(struct lis3dh_acc_status *stat)
{
	if (stat->on_before_suspend)
		return lis3dh_acc_enable(stat);

	return 0;
}
EXPORT_SYMBOL(lis3dh_acc_common_resume);

int lis3dh_acc_common_suspend(struct lis3dh_acc_status *stat)
{
	stat->on_before_suspend = atomic_read(&stat->enabled);

	return lis3dh_acc_disable(stat);
}
EXPORT_SYMBOL(lis3dh_acc_common_suspend);
#endif /* CONFIG_PM */

MODULE_DESCRIPTION("lis3dh accelerometer driver");
MODULE_AUTHOR("Matteo Dameno");
MODULE_AUTHOR("Denis Ciocca");
MODULE_AUTHOR("Mario tesi");
MODULE_LICENSE("GPL v2");
