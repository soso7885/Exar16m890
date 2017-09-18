#ifndef _8250_EXAR_16M890_H
#define _8250_EXAR_16M890_H

#define XR_RS422	0100000
#define XR_RS485	0200000

#ifndef CDTRDSR
#define CDTRDSR		004000000000  /* DTR/DSR flow control */
#endif

#define NR_PORTS CONFIG_SERIAL_EXAR_16M890_NR_UARTS

/*
 * The special register set for XR16M890 UARTs.
*/
#define XR_16M890_DLD			2
#define XR_16M890_SHR			5
#define XR_16M890_SFR			6
#define XR_FCTR_DIR_CTRL	0x08  /* Auto RS-485 Direction Control */
#define XR_MCR_PRESCALER	0x80
#define XR_16M890_SFR_TXDIS	0x10	

/*
 * The ioctl command
*/
#define ADVTIOCSETRTL			0x5460
#define ADVTIOCSETTTL			0x5461
#define ADVTIOCSETFCL			0x5462
#define ADVTIOCSETFCH			0x5463
#define ADVTIOCSETFIFOSIZE		0x5464
#define ADVTIOCSETCHANGFLAG		0x5465
#define ADVTIOCSERGTTYFLIP		0x5468
#define ADVSENDXON				0x546C
#define ADVSENDXOFF				0x546D
#define ADVTIOCSETBAUDRATE		0x546E
#define ADVTIOCSERGCHARTIMEOUT	0x5470
#define ADVTIOCRTURDFRAME		0x5471
#define ADVTIOLOOPBACK			0x5473
#define ADVTIOCTRAPDCDDSR		0x5475

#define XR_PASS_LIMIT			256

enum porttype { PORT_RS485=1, PORT_RS232, PORT_RS422 };

struct exar_priv{
	int gpio_sel;
	int line;
	unsigned int baud;
	unsigned int RTL;
	unsigned int TTL;
	unsigned int FCL;
	unsigned int FCH;
	unsigned int fifosize;
	bool throttle;
	bool mbusreadmode;
	bool loopback;
	bool rts;
	bool xon;
	bool dtr;
	unsigned long rx;
	unsigned long tx;
	unsigned long charto;
	unsigned int data;
	unsigned int stop;
	unsigned int pary;
	enum porttype type;
};	
#endif
