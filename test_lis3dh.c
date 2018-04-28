#include <stdio.h>
#include <fcntl.h>
#include <linux/input.h>
static struct input_event test[5];
int main()
{
    int result = 0;
    int fd = 0;
    int i,count = 0;
    fd = open ("/dev/input/event2",O_RDWR);
    while(1)
    {
	count=read(fd, test, sizeof(struct input_event));
	for(i=0;i<(int)count/sizeof(struct input_event);i++)
	{
            if(EV_REL== test[i].type)
	    {
		printf("time:%ld.%d", test[i].time.tv_sec, test[i].time.tv_usec);
                printf("type:%d code:%d value:%d\n", test[i].type, test[i].code, test[i].value);
	    }

	    if(MSC_SERIAL== test[i].type)
	    {
		printf("time:%ld.%d", test[i].time.tv_sec, test[i].time.tv_usec);
		printf("x type:%d code:%d value:%d\n", test[i].type, test[i].code, test[i].value);
	    }
	    if(MSC_PULSELED== test[i].type)
	    {
		printf("time:%ld.%d", test[i].time.tv_sec, test[i].time.tv_usec);
		printf("y type:%d code:%d value:%d\n", test[i].type, test[i].code, test[i].value);
	    }
	    if(MSC_GESTURE== test[i].type)
	    {
		printf("time:%ld.%d", test[i].time.tv_sec, test[i].time.tv_usec);
		printf("z type:%d code:%d value:%d\n", test[i].type, test[i].code, test[i].value);
	    }
	    if(MSC_SCAN== test[i].type)
	    {
		printf("time:%ld.%d", test[i].time.tv_sec, test[i].time.tv_usec);
		printf("time msb type:%d code:%d value:%d\n", test[i].type, test[i].code, test[i].value);
	    }
	    if(MSC_MAX== test[i].type)
	    {
		printf("time:%ld.%d", test[i].time.tv_sec, test[i].time.tv_usec);
		printf("time lsb type:%d code:%d value:%d\n", test[i].type, test[i].code, test[i].value);
	    }
 	}
    }
    return result;
}
