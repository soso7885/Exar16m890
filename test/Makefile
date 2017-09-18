CC = arm-linux-gnueabihf-

all: mctrl stress rw4port

mctrl: mctrl.c
	$(CC)gcc -Wall -std=c99 $^ -o $@

stress: stress.c
	$(CC)gcc -Wall -std=c99 $^ -o $@

rw4port: rw4port.c
	$(CC)gcc -Wall -std=c99 $^ -o $@ -pthread

clean:
	rm -f stress
	rm -f mctrl
	rm -f rw4port
