#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <errno.h>
#include <limits.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <asm-generic/termbits.h>
#include <assert.h>

#define DATALEN 1024

char tx[DATALEN];
char rx[DATALEN];

int _tty_flush(int fd)
{
	int clnlen;
	fd_set rfds;
	struct timeval tv;
	int retval;
	int rlen;
	char tmpbuf[1024];

	clnlen = 0;
	
	do{
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		tv.tv_sec = 3;
		tv.tv_usec = 0;
		retval = select(fd+1, &rfds, 0, 0, &tv);

		if(retval < 0){
			printf("%s: select failed!\n", __func__);
			exit(0);
		}

		if(retval == 0) break;

		rlen = read(fd, tmpbuf, DATALEN);
		if(rlen < 0){
			printf("error durring clean\n");
			exit(0);
		}

		clnlen += rlen;
	}while(1);

	return clnlen;
}

int main(int argc, char **argv)
{
	int fd;
	int ret;
	struct termios2 newtio;
	fd_set wfds;
	fd_set rfds;
	struct timeval tv;
	int retval;

	if(argc != 3){
		printf("%s [tty_path] [baudrate]\n", argv[0]);
		return -1;
	}

	for(int i = 0; i < DATALEN; i++){
		if(i == 0){
			tx[i] = '!';
		}else{
			tx[i] = tx[i - 1] + 1;
			if(tx[i] > '}'){
				tx[i]= '!';
			}
		}
	}

	fd = open(argv[1], O_RDWR);
	printf("\n[%s] open fd = %d\n", argv[1], fd);
	ret = ioctl(fd, TCGETS2, &newtio);
	assert(ret == 0);
	printf("\n[%s] ospeed %d ispeed %d\n", argv[1], newtio.c_ospeed, newtio.c_ispeed);
	newtio.c_iflag &= ~(ISTRIP|IUCLC|IGNCR|ICRNL|INLCR|ICANON|IXON|PARMRK);
	newtio.c_iflag |= (IGNBRK|IGNPAR);
	newtio.c_lflag &= ~(ECHO|ICANON|ISIG);
	newtio.c_cflag &= ~CBAUD;
	newtio.c_cflag |= BOTHER|CRTSCTS;
	newtio.c_ospeed = atoi(argv[2]);
	newtio.c_ispeed = atoi(argv[2]);
	ret = ioctl(fd, TCSETS2, &newtio);
	assert(ret == 0);
	sleep(1);
	ret = ioctl(fd, TCGETS2, &newtio);
	assert(ret == 0);
	printf("[%s] ospeed %d ispeed %d\n", argv[1], newtio.c_ospeed, newtio.c_ispeed);
	
	_tty_flush(fd);
	
	int wlen = 0;
	int rlen = 0;
	int woff = 0;
	int roff = 0;
	do{
		wlen = 0;
		rlen = 0;
		FD_ZERO(&wfds);
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		FD_SET(fd, &wfds);
		
		tv.tv_sec = 30;
		tv.tv_usec = 0;
		retval = select(fd+1 , &rfds, &wfds, NULL, &tv);
		if(retval < 0){
			printf("\n[%s]: select failed!\n", argv[1]);
			break;
		}

		if(retval == 0) {
			printf("\n[%s]: select nothing\n", argv[1]);
			break;
		}

		if(FD_ISSET(fd, &wfds)){
			wlen = write(fd, &tx[woff], DATALEN - woff);
			if(wlen <= 0){
				printf("\n[%s]: write failed\n", argv[1]);
				break;
			}
			woff = (woff+wlen) % ('}' - '!' + 1);
		}

		if(FD_ISSET(fd, &rfds)){
			rlen = read(fd, rx, DATALEN-roff);
			if(rlen <= 0){
				printf("\n[%s]: read failed!\n", argv[1]);
				break;
			}
			if(memcmp(rx, &tx[roff], rlen)){
				printf("[%s]: Data error!\n", argv[1]);
				for(int i = 0; i < rlen; ++i)
					printf("%x ", rx[i]);
				printf("\n");
				break;
			}
			roff = (roff+rlen) % ('}' - '!' + 1);
		}
	}while(1);

	printf("\n[%s] close....\n", argv[1]);
	close(fd);

	return 0;
}

