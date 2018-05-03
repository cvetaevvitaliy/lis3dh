#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>

#include "lis3dhlib.h"
int main()
{
    int ret = 0;
    int fd = 0;
	fd_set fds;
    int i,count = 0;
    ret = open ("/dev/lis3dh",O_RDWR);
	if(ret < 0)
	{
		goto ERR_EXIT;
	}

	fd = ret;
	//SET output data rate 10Hz
	ret = ioctl(fd, SET_ODR, 10);
	if(ret < 0)
	{
		goto ERR_EXIT;
	}

	//SET fs range +-2G
	ret = ioctl(fd, SET_FS, 2);
	if(ret < 0)
	{
		goto ERR_EXIT;
	}

	//enable 
	ret = ioctl(fd, SET_ENABLE, 1);
	if(ret < 0)
	{
		goto ERR_EXIT;
	}
    while(1)
    {
    	struct timeval timeout = {0, 200000};
        FD_ZERO(&fds);
        FD_SET(fd, &fds);

		ret = select(fd +1, &fds, &fds, NULL, &timeout);
		if(ret)
        {
        	char databuf[12] = {0};
			int *pdata;
			float acc_x, acc_y, acc_z;
            ret = read(fd, &databuf, 12);
			if(ret < 12)
			{
				goto ERR_EXIT;
			}

			pdata = (int *)databuf;
			acc_x = pdata[0] * 9.8 / 1000;
			acc_y = pdata[1] * 9.8 / 1000;
			acc_z = pdata[2] * 9.8 / 1000;
            printf("x:%f y:%f z:%f\n", acc_x, acc_y, acc_z);
        }
    }

ERR_EXIT:
	
    return ret;
}
