#ifndef _LIS3DHLIB_H_
#define _LIS3DHLIB_H_
#ifdef __KERNEL__
#include <linux/ioctl.h>
#else
#include <sys/ioctl.h>
#endif
#define LIS3DH_IOC_MAGIC 0xE3

//set the output data rate(support 1,3,5,10,20,40,100,1000ms,default:100)
#define SET_ODR		_IO(LIS3DH_IOC_MAGIC,  0)	//
//set the full scale selection(support 2,4,8,16G,default 2G)
#define SET_FS      _IO(LIS3DH_IOC_MAGIC,  1)	//
//set the lis3dh enable or disable(0:disable 1:enable)
#define SET_ENABLE	_IO(LIS3DH_IOC_MAGIC,  2)	//

#endif/*_LIS3DHLIB_H_*/