#ifndef _8250_EXAR_16M890_H
#define _8250_EXAR_16M890_H

#include <linux/serial_8250.h>
#include <linux/serial_reg.h>

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

#define UART_CAP_FIFO	(1 << 8)	/* UART has FIFO */
#define UART_CAP_EFR	(1 << 9)	/* UART has EFR */
#define UART_CAP_SLEEP	(1 << 10)	/* UART has IER sleep */
#define UART_CAP_AFE	(1 << 11)	/* MCR-based hw flow control */
#define UART_CAP_UUE	(1 << 12)	/* UART needs IER bit 6 set (Xscale) */
#define UART_CAP_RTOIE	(1 << 13)	/* UART needs IER bit 4 set (Xscale, Tegra) */
#define UART_CAP_HFIFO	(1 << 14)	/* UART has a "hidden" FIFO */
#define UART_CAP_RPM	(1 << 15)	/* Runtime PM is active while idle */

#define UART_BUG_QUOT	(1 << 0)	/* UART has buggy quot LSB */
#define UART_BUG_TXEN	(1 << 1)	/* UART has buggy TX IIR status */
#define UART_BUG_NOMSR	(1 << 2)	/* UART has buggy MSR status bits (Au1x00) */
#define UART_BUG_THRE	(1 << 3)	/* UART has buggy THRE reassertion */
#define UART_BUG_PARITY	(1 << 4)	/* UART mishandles parity if FIFO enabled */

#if defined(__alpha__) && !defined(CONFIG_PCI)
/*
 * Digital did something really horribly wrong with the OUT1 and OUT2
 * lines on at least some ALPHA's.  The failure mode is that if either
 * is cleared, the machine locks up with endless interrupts.
 */
#define ALPHA_KLUDGE_MCR  (UART_MCR_OUT2 | UART_MCR_OUT1)
#else
#define ALPHA_KLUDGE_MCR 0
#endif

enum porttype { PORT_RS485=1, PORT_RS232, PORT_RS422 };

struct serialxr_config {
	const char	*name;
	unsigned short	fifo_size;
	unsigned short	tx_loadsz;
	unsigned char	fcr;
	unsigned char	rxtrig_bytes[UART_FCR_R_TRIG_MAX_STATE];
	unsigned int	flags;
};


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

static inline int serial_in(struct uart_8250_port *up, int offset)
{
	return up->port.serial_in(&up->port, offset);
}

static inline void serial_out(struct uart_8250_port *up, int offset, int value)
{
	up->port.serial_out(&up->port, offset, value);
}

static inline int serial_index(struct uart_port *port)
{
	return port->minor - 64;
}

#if 0
#define DEBUG_INTR(fmt...)	printk(fmt)
#else
#define DEBUG_INTR(fmt...)	do { } while (0)
#endif

#endif
