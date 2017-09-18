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
#include <pthread.h>
#include <sys/select.h>

#define handle_error_en(en, msg) \
	do{ errno = en; perror(msg); exit(0); }while(0)
#define SIZE 8

unsigned char buf[SIZE];
int fd[4];
int tx_byte[4];
int rx_byte[4];
int err_byte[4];

static inline void init_tx(void)
{ 
	for(int i = 0; i < SIZE; ++i) buf[i] = 65+i;
	printf("tx buffer init done:\n");
	for(int i = 0; i < SIZE; ++i) 
		printf("%d ", buf[i]);
	printf("\n");
}

static void set_termios(int fd)
{
	int ret;
	struct termios2 newtio;

	ret = ioctl(fd, TCGETS2, &newtio);
	assert(ret == 0);
	printf("[%d]: ospeed %d ispeed %d\n", fd, newtio.c_ospeed, newtio.c_ispeed);
	newtio.c_iflag &= ~(ISTRIP|IUCLC|IGNCR|ICRNL|INLCR|ICANON|IXON|PARMRK);
	newtio.c_iflag |= (IGNBRK|IGNPAR);
	newtio.c_lflag &= ~(ECHO|ICANON|ISIG);
	newtio.c_cflag &= ~CBAUD;
	newtio.c_cflag |= BOTHER;
	newtio.c_ospeed = 9600;
	newtio.c_ispeed = 9600;
	ret = ioctl(fd, TCSETS2, &newtio);
	assert(ret == 0);
	sleep(1);
	ret = ioctl(fd, TCGETS2, &newtio);
	assert(ret == 0);
	printf("[%d]: ospeed %d ispeed %d\n", fd, newtio.c_ospeed, newtio.c_ispeed);
}

static void tty_flush(int fd, int i)
{
	int total = 0;
	fd_set rfds;
	struct timeval tv;
	int retval;
	int rlen;
	unsigned char tmp[32];
	
	do{
		FD_ZERO(&rfds);
		FD_SET(fd, &rfds);
		tv.tv_sec = 3;
		tv.tv_usec = 0;
		retval = select(fd+1, &rfds, 0, 0, &tv);
		if(retval < 0){
			printf("%s select failed!\n", __func__);
			exit(0);
		}
		
		if(retval == 0) break;
		
		rlen = read(fd, tmp, 32);
		if(rlen <= 0){
			printf("error durring clean\n");
			exit(0);
		}
		total += rlen;
	}while(1);

	printf("[S%d] flush %d byte\n", i+1, total);
}

void *do_work(void *data)
{
	int ret;
	int order = *(int *)data;
	int tfd = fd[order];
	unsigned char tmp[32];
	fd_set rfds;
	fd_set wfds;
	struct timeval tv;

	set_termios(tfd);

	tty_flush(tfd, order);

	int wlen = 0;
	int rlen = 0;
	do{
		FD_ZERO(&wfds);
		FD_ZERO(&rfds);
		FD_SET(tfd, &rfds);
		if(wlen == 0)
			FD_SET(tfd, &wfds);
		tv.tv_sec = 2;
		tv.tv_usec = 0;

		ret = select(tfd+1, &rfds, &wfds, NULL, &tv);
		if(ret < 0){
			printf("[%d] select failed!\n", order);
			break;
		}
	
		if(ret == 0) continue;
		
		if(FD_ISSET(tfd, &wfds)){
			wlen = write(tfd, buf, SIZE);
			if(wlen != SIZE){
				printf("[ttyS%d]: write failed!\n", order+1);
				break;
			}
			tx_byte[order] += wlen;
		}

		if(FD_ISSET(tfd, &rfds)){
			rlen = read(tfd, tmp, 32);
			if(rlen < 0){
				printf("[ttyS%d]: read failed!\n", order+1);
				break;
			}
			rx_byte[order] += rlen;
			wlen = 0;
		}
		
		if(rlen){
			for(int i = 0; i < SIZE; ++i){
				if(tmp[i] ^ buf[i]) err_byte[order]++;
			}
			rlen = 0;
		}
		printf("[S1]: wr(%d) rd(%d) err(%d)," 
				"[S2]: wr(%d) rd(%d) err(%d),"
				"[S3]: wr(%d) rd(%d) err(%d),"
				"[S4]: wr(%d) rd(%d) err(%d).. \r",
				tx_byte[0], rx_byte[0], err_byte[0],
				tx_byte[1], rx_byte[1], err_byte[1],
				tx_byte[2], rx_byte[2], err_byte[2],
				tx_byte[3], rx_byte[3], err_byte[3]);
		fflush(stdout);
	}while(1);
	
	close(tfd);
	pthread_exit(0);
}

int main(int argc, char **argv)
{
	int ret;
	char name[16];
	pthread_t tid[4];

	init_tx();

	for(int i = 0; i < 4; i++){
		sprintf(name, "/dev/ttyS%d", i+1);
		fd[i] = open(name, O_RDWR);
		assert(fd[i] > 0);
		printf("Open %s success, fd = %d\n", name, fd[i]);
		ret = pthread_create(&tid[i], NULL, do_work, (void *)&i);
		if(ret != 0) handle_error_en(ret, "pthred_create");
		sleep(2);
	}
	
	for(int i = 0; i < 4; ++i)
		pthread_join(tid[i], NULL);	

	return 0;
}
