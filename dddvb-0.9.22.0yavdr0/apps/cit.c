#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <pthread.h>

uint8_t fill[188]={0x47, 0x1f, 0xff, 0x10,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
		   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff };

uint8_t ts[188]={0x47, 0x0a, 0xaa, 0x00,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,
		   0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff };

void proc_buf(uint8_t *buf, uint32_t *d)
{
	uint32_t c;
	static uint32_t tc=0;
	

	if (buf[1]==0x1f && buf[2]==0xff) {
		//printf("fill\n");
		return;
	}
	if (buf[1]==0x9f && buf[2]==0xff) {
		//printf("fill\n");
		return;
	}
	if (buf[1]==0x1a && buf[2]==0xbb) {
		tc++;
		if (!(tc&0xfff))
			printf("T %d\n", tc);
		return;
	}
	//printf("%02x %02x %02x %02x\n", buf[0], buf[1], buf [2], buf[3]);
	if (buf[1]!=0x0a || buf[2]!=0xaa)
		return;
	c=(buf[4]<<24)|(buf[5]<<16)|(buf[6]<<8)|buf[7];
	if (c!=*d) {
		printf("CONT ERROR: got %08x  expected %08x\n", c, *d);
		*d=c;
	} else {
		if (memcmp(ts+8, buf+8, 180))
			printf("error\n");
		if (!(c&0xffff))
			printf("R %08x\n", c);
	}
	(*d)++;
}

void *get_ts(void *a)
{
	uint8_t buf[188*1024];
	int len, off;

	int fdi=open("/dev/dvb/adapter2/ci0", O_RDONLY);
	uint32_t d=0;

	while (1) {
		len=read(fdi, buf, 188*1024);
		if (len<0)
			continue;
		if (buf[0]!=0x47) {
			read(fdi, buf, 1);
			continue;
		}
		for (off=0; off<len; off+=188) {
			proc_buf(buf+off, &d);
		}
	}	
}

#define SNUM 233
//671
void send(void)
{
	uint8_t buf[188*SNUM], *cts;
	int i;
	uint32_t c=0;
	int fdo;

	fdo=open("/dev/dvb/adapter2/ci0", O_WRONLY);


	while (1) {
		for (i=0; i<SNUM; i++) {
			cts=buf+i*188;
			memcpy(cts, ts, 188);
			cts[4]=(c>>24);
			cts[5]=(c>>16);
			cts[6]=(c>>8);
			cts[7]=c;
			//write(fdo, fill, 188);
			//printf("S %d\n", c); 
			c++;
			//usleep(100000+0xffff&rand());
			//usleep(1000);
		}
		write(fdo, buf, 188*SNUM);
	}
}


int main()
{
	pthread_t th;

	memset(ts+8, 180, 0x5a);
	pthread_create(&th, NULL, get_ts, NULL);
	usleep(10000);
	send();
}
