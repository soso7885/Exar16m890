/**
*	Driver for EXAR 16m890 uart controller.
*	Based on driver/serial/8250.c
*
*	This program is free software; you can redistribute it and/or modify
*	it under the terms of the GNU General Public License as published by
*	the Free Software Foundation; either version 2 of the License, or
*	(at your option) any later version.
*
*	This program is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY; without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*	GNU General Public License for more details.
*
*	reference by linux-2.4.x/driver/char/adv950.c, type = PORT_16850
*/

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/console.h>
#include <linux/sysrq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/tty.h>
#include <linux/ratelimit.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_8250.h>
#include <linux/nmi.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/types.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>

#include "8250_exar_16m890.h"

//#define __ADV_DEBUG_INFO__
//#define __ADV_DEBUG_IRQ__

#ifdef __ADV_DEBUG_IRQ__
int db_txstat[16];
int db_re_ier_cnt[16];
unsigned int db_thr_cnt[16];
uint64_t db_irq_record[16];
#define ADV_DEBUG_IRQ(EX) (void)(EX)
#else
#define ADV_DEBUG_IRQ(EX) do{}while(0)
#endif

#ifdef __ADV_DEBUG_INFO__
#define adv_info(str, args...) pr_info("" str, ##args)
#else
#define adv_info(str, args...) do{}while(0)
#endif

#define LOWBAUD_THRESHOLD		2400
#define MIDBAUD_THRESHOLD		9600
#define LOWBAUD_TTL		UART_TRG_1
#define LOWBAUD_RTL		UART_TRG_2
#define MIDBAUD_TTL		UART_TRG_1
#define MIDBAUD_RTL		UART_TRG_5
#define HIGHBAUD_TTL	UART_TRG_8
#define HIGHBAUD_RTL	UART_TRG_48

extern void set_io_from_upio(struct uart_port *p);

static const struct serialxr_config xr_uart_config[] = {
	[PORT_UNKNOWN] = {
		.name		= "unknown",
		.fifo_size	= 1,
		.tx_loadsz	= 1,
	},
	[PORT_XR16M890] = {
		.name		= "XR16M890",
		.fifo_size	= 128,
		.tx_loadsz	= 128,
		.fcr		= UART_FCR_ENABLE_FIFO,
		.flags		= UART_CAP_FIFO | UART_CAP_EFR, /* uart has fifo */
	},
};

static inline const char *serialxr_portname(struct uart_port *port)
{
	return to_platform_device(port->dev)->name;
}

static inline void private_data_init(struct uart_port *port)
{
	struct exar_priv *priv = port->private_data;
	
	priv->throttle = false;
	priv->mbusreadmode = false;
	priv->loopback = false;
	priv->charto = 0;
	priv->baud = 9600;
}

/* 
 * Caller holds uart port lock 
 */
static unsigned int serialxr_modem_status(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port); 
	unsigned int status = serial_in(up, UART_MSR);

	status |= up->msr_saved_flags;
	up->msr_saved_flags = 0;
	if(status & UART_MSR_ANY_DELTA && 
		up->ier & UART_IER_MSI &&
	    port->state != NULL)
	{
		if(status & UART_MSR_TERI)
			port->icount.rng++;
		if(status & UART_MSR_DDSR)
			port->icount.dsr++;
		if(status & UART_MSR_DDCD)
			uart_handle_dcd_change(port, status & UART_MSR_DCD);
		if(status & UART_MSR_DCTS)
			uart_handle_cts_change(port, status & UART_MSR_CTS);

		wake_up_interruptible(&port->state->port.delta_msr_wait);
	}

	return status;
}

/*
 * FIFO support.
 */
static void serialxr_clear_fifos(struct uart_8250_port *p)
{
	if (p->capabilities & UART_CAP_FIFO) {
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO);
		serial_out(p, UART_FCR, UART_FCR_ENABLE_FIFO |
			       UART_FCR_CLEAR_RCVR | UART_FCR_CLEAR_XMIT);
		serial_out(p, UART_FCR, 0);
	}
}

#define BOTH_EMPTY  (UART_LSR_TEMT | UART_LSR_THRE)
static unsigned int serialxr_tx_empty(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned long flags;
	unsigned int lsr;

	spin_lock_irqsave(&port->lock, flags);
	lsr = serial_port_in(port, UART_LSR);
	up->lsr_saved_flags |= lsr & LSR_SAVE_FLAGS;
	spin_unlock_irqrestore(&port->lock, flags);

	return (lsr & BOTH_EMPTY) == BOTH_EMPTY ? TIOCSER_TEMT : 0;
}
#undef BOTH_EMPTY

static void serialxr_stop_tx(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	
	if (up->ier & UART_IER_THRI) {
		up->ier &= ~UART_IER_THRI;
		serial_port_out(port, UART_IER, up->ier);
	}
}

static void serialxr_start_tx(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;

	if (!(up->ier & UART_IER_THRI)) {
		up->ier |= UART_IER_THRI;
		serial_port_out(port, UART_IER, up->ier);
	}
	
	if (priv->loopback) {
		unsigned int lcr;
		unsigned int mcr;

		lcr = serial_port_in(port, UART_LCR);
		serial_port_out(port, UART_LCR, 0);
		mcr = serial_port_in(port, UART_MCR);
		serial_port_out(port, UART_MCR, (mcr|0x10));
		serial_port_out(port, UART_LCR, lcr);
	}
}

static void serialxr_stop_rx(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	up->ier &= ~UART_IER_RLSI;
	up->port.read_status_mask &= ~UART_LSR_DR;
	serial_port_out(port, UART_IER, up->ier);
}

static void serialxr_rx_chars_RDI(struct uart_port *port)
{
	int i;
	unsigned int bytes_in_fifo;
	unsigned char ch;
	struct tty_port *tty = &port->state->port;
	struct exar_priv *priv = port->private_data;

	if(priv->throttle) return; //need?

	bytes_in_fifo = priv->RTL-1;

	for(i = 0; i < bytes_in_fifo; ++i){
		ch = serial_port_in(port, UART_RX);
		tty_insert_flip_char(tty, ch, TTY_NORMAL);
		port->icount.rx++;
	}
	priv->rx = port->icount.rx;

	if(priv->mbusreadmode) return;

	spin_unlock(&port->lock);
	tty_flip_buffer_push(tty);
	spin_lock(&port->lock);
	
	DEBUG_INTR(" LSR_DR...");
}

static void serialxr_rx_chars(struct uart_port *port, unsigned char *lsr)
{
	unsigned char ch;
	int max_count = 256;
	struct tty_port *tty = &port->state->port;
	struct exar_priv *priv = port->private_data;

	if(priv->throttle) return; //need?

	do{
		if(likely(*lsr & UART_LSR_DR))
			ch = serial_port_in(port, UART_RX);

		port->icount.rx++;

		if (uart_handle_sysrq_char(port, ch))
			goto ignore_char;

		tty_insert_flip_char(tty, ch, TTY_NORMAL);

ignore_char:
		*lsr = serial_port_in(port, UART_LSR);
	}while((*lsr & UART_LSR_DR) && (max_count-- > 0));
	
	priv->rx = port->icount.rx;

	spin_unlock(&port->lock);
	tty_flip_buffer_push(tty);
	spin_lock(&port->lock);

	DEBUG_INTR(" LSR_DR...");
}

static void serialxr_tx_chars(struct uart_port *port)
{
	unsigned int count;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct circ_buf *xmit = &port->state->xmit;
	struct exar_priv *priv = port->private_data;

	if(port->x_char){
		serial_port_out(port, UART_TX, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		priv->tx = port->icount.tx;
		ADV_DEBUG_IRQ(db_txstat[priv->line] = -1);
		return;
	}
	
	if(uart_tx_stopped(port)){
		serialxr_stop_tx(port);
		ADV_DEBUG_IRQ(db_txstat[priv->line] = -2);
		return;
	}
	
	if(uart_circ_empty(xmit)){
		serialxr_stop_tx(port);
		ADV_DEBUG_IRQ(db_txstat[priv->line] = -3);
		return;
	}
	
	count = (unsigned int)(port->fifosize - priv->TTL);

	do{
		serial_port_out(port, UART_TX, xmit->buf[xmit->tail]);
		xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		port->icount.tx++;
		if (uart_circ_empty(xmit)) 
			break;
	} while (--count > 0);
	priv->tx = port->icount.tx;

	if(uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

	DEBUG_INTR("THRE...");

	if(uart_circ_empty(xmit)){
		serialxr_stop_tx(port);
	}else{
		unsigned int THR;

		THR = serial_port_in(port, UART_SCR);
		if(THR < priv->TTL){
			/* Try to re-trigger Enable Transmitter holding */
			pr_info("re-trigger tx empty interrupt, THR=%d\n", THR);
			serial_port_out(port, UART_IER, up->ier & (~UART_IER_THRI));
			serial_port_out(port, UART_IER, up->ier);
			ADV_DEBUG_IRQ(db_re_ier_cnt[priv->line]++);
		}
	}
	ADV_DEBUG_IRQ(db_txstat[priv->line] = uart_circ_chars_pending(xmit));
}

static void serialxr_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS)
		mcr |= UART_MCR_RTS;
	if (mctrl & TIOCM_DTR)
		mcr |= UART_MCR_DTR;
	if (mctrl & TIOCM_OUT1)
		mcr |= UART_MCR_OUT1;
	if (mctrl & TIOCM_OUT2)
		mcr |= UART_MCR_OUT2;
	if (mctrl & TIOCM_LOOP)
		mcr |= UART_MCR_LOOP;

	mcr = (mcr & up->mcr_mask) | up->mcr_force | up->mcr;
	serial_port_out(port, UART_MCR, mcr);
}

static unsigned int serialxr_get_mctrl(struct uart_port *port)
{
	unsigned int status;
	unsigned int ret = 0;

	status = serialxr_modem_status(port);

	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;

	return ret;
}

static void check_line_status(struct uart_port *port, unsigned char *lsr)
{
	while(*lsr & UART_LSR_BRK_ERROR_BITS) {
		if (*lsr & UART_LSR_BI) {
			*lsr &= ~(UART_LSR_FE | UART_LSR_PE);
			port->icount.brk++;
		} else if (*lsr & UART_LSR_PE) {
			port->icount.parity++;
		} else if (*lsr & UART_LSR_FE) {
			port->icount.frame++;
		}
		if (*lsr & UART_LSR_OE) {
			port->icount.overrun++;
		}
		*lsr = serial_port_in(port, UART_LSR);
	}
}

unsigned char serialxr_compute_lcr(struct uart_8250_port *up,
									tcflag_t c_cflag)              
{
	unsigned char cval;
	struct exar_priv *priv = up->port.private_data;

	switch (c_cflag & CSIZE) {
		case CS5:
			cval = UART_LCR_WLEN5;
			priv->data = 5;
			break;
		case CS6:
			cval = UART_LCR_WLEN6;
			priv->data = 6;
			break;
		case CS7:
			cval = UART_LCR_WLEN7;
			priv->data = 7;
			break;
		default:
		case CS8:
			cval = UART_LCR_WLEN8;
			priv->data = 8;
			break;                
	}

	if (c_cflag & CSTOPB){    
		cval |= UART_LCR_STOP;
		priv->stop = 2;
	}else{
		priv->stop = 1;
	}

	if (c_cflag & PARENB) {   
		cval |= UART_LCR_PARITY;
		if(c_cflag & PARODD){
			if(c_cflag & CMSPAR){
				priv->pary = 4;
			}else{
				priv->pary = 2;
			}
		}else if(c_cflag & CMSPAR){
			priv->pary = 5;
		}else{
			priv->pary = 3;
		}
	}else{
		priv->pary = 1;
	}
		  
	if (!(c_cflag & PARODD))  
		cval |= UART_LCR_EPAR;
#ifdef CMSPAR
	if (c_cflag & CMSPAR)
		cval |= UART_LCR_SPAR;
#endif 

	return cval;
}

/*
 * The exar 16m890 uart driver interrupt handle routine
*/
#define MASK 0xfffffff0
static irqreturn_t serialxr_isr(int irq, void *dev_id)
{
	unsigned int iir;
	unsigned char lsr;
	int max_count = XR_PASS_LIMIT;
	int handled = 0;
	struct uart_port *port = dev_id;
	struct exar_priv *priv = port->private_data;

	DEBUG_INTR("serialxr_interrupt(%d)...", irq);

	spin_lock(&port->lock);

	do {
		iir = serial_port_in(port, UART_IIR);
		if(iir & UART_IIR_NO_INT)
			break;

		handled = 1;
		switch(iir & 0x3F){
			case UART_IIR_RDI: /* Receiver data interrupt */
				ADV_DEBUG_IRQ(
							db_irq_record[priv->line] = 
							((db_irq_record[priv->line]<<4)&MASK)|1
							);
				lsr = (unsigned char)serial_port_in(port, UART_LSR);
				if(lsr & UART_LSR_DR)
					serialxr_rx_chars_RDI(port);
				break;
			case UART_IIR_MSI: /* Modem status interrupt */
				ADV_DEBUG_IRQ(
							db_irq_record[priv->line] = 
							((db_irq_record[priv->line]<<4)&MASK)|2
							);
				serialxr_modem_status(port);
				break;
			case UART_IIR_RX_TIMEOUT: /* Receiver data time-out */
				ADV_DEBUG_IRQ(
							db_irq_record[priv->line] = 
							((db_irq_record[priv->line]<<4)&MASK)|3
							);
				lsr = (unsigned char)serial_port_in(port, UART_LSR);
				if(lsr & UART_LSR_DR)
					serialxr_rx_chars(port, &lsr);
				priv->charto++;
				break;
			case UART_IIR_RLSI: /* Receiver line status interrupt */
				ADV_DEBUG_IRQ(
							db_irq_record[priv->line] = 
							((db_irq_record[priv->line]<<4)&MASK)|4
							);
				lsr = (unsigned char)serial_port_in(port, UART_LSR);
				check_line_status(port, &lsr);
				break;
			case UART_IIR_THRI: /* Transmitter holding register empty */
				ADV_DEBUG_IRQ(
							db_irq_record[priv->line] = 
							((db_irq_record[priv->line]<<4)&MASK)|5
							);
				serialxr_tx_chars(port);
				break;
			default:
				break;
		}
	}while (!(iir & UART_IIR_NO_INT) && max_count--);

	spin_unlock(&port->lock);

	DEBUG_INTR("end.\n");

	return IRQ_RETVAL(handled);
}
#undef MASK

static int serialxr_startup(struct uart_port *port)
{
	int retval;
	unsigned long flags;
	unsigned char fctr = 0;
	struct uart_8250_port *up = up_to_u8250p(port);

	port->fifosize = xr_uart_config[port->type].fifo_size;
	up->tx_loadsz = xr_uart_config[port->type].tx_loadsz;
	up->capabilities = xr_uart_config[port->type].flags;
	
	private_data_init(port);
	up->mcr = 0;

	if (port->iotype != up->cur_iotype)
		set_io_from_upio(port);

	serialxr_clear_fifos(up);
	serial_port_out(port, UART_IER, 0);
	/*
	 * Clear the interrupt registers.
	 */
	serial_port_in(port, UART_LSR);
	serial_port_in(port, UART_RX);
	serial_port_in(port, UART_IIR);
	serial_port_in(port, UART_MSR);
	/*
	 * At this point, there's no way the LSR could still be 0xff;
	 * if it is, then bail out, because there's likely no UART
	 * here.
	 */
	if (!(port->flags & UPF_BUGGY_UART) &&
	    (serial_port_in(port, UART_LSR) == 0xff)) 
	{
		printk_ratelimited(KERN_INFO "ttyS%d: LSR safety check engaged!\n",
						   serial_index(port));
		retval = -ENODEV;
		goto out;
	}

	/* Enable enhance mode */ 
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, UART_EFR_ECB);
	/* Set tx/rx fifo trigger level */
	fctr = serial_port_in(port, UART_FCTR);
	serial_port_out(port, UART_FCTR, UART_FCTR_TRGD | 
										UART_FCTR_TX | fctr);
	serial_port_out(port, UART_TRG, UART_TRG_1);
	serial_port_out(port, UART_FCTR, UART_FCTR_TRGD |
									(~UART_FCTR_TX & fctr));
	serial_port_out(port, UART_TRG, UART_TRG_5);
	serial_port_out(port, UART_LCR, 0);
	/*
	 * Wakeup and initilaize UART
	 * enable TX FIFO count(SPR),
	 * to let tx read how many data in tx-fifo dirctily
	 *
	 * First: FCTR[6]=1, and use table-D FCTR[5:4]
	 * Second: LCR=0 and SFR[0]=0
	 * Finally: config EMSR
	 */
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, UART_EFR_ECB);
	serial_port_out(port, UART_LCR, 0);
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_FCTR, UART_FCTR_SCR_SWAP | UART_FCTR_TRGD);
	serial_port_out(port, UART_LCR, 0);
	up->sfr &= ~0x01;
	serial_port_out(port, XR_16M890_SFR, up->sfr);
	serial_port_out(port, UART_EMSR, 0x01);

	/* NO support shared IRQ */
	retval = request_irq(port->irq, serialxr_isr, 
						port->irqflags, serialxr_portname(port), port);
	if(retval)
		goto out;
	
	/*
	 * Now, initialize the UART
	 */
	serial_port_out(port, UART_LCR, UART_LCR_WLEN8);

	spin_lock_irqsave(&port->lock, flags);
	/* Most PC uarts need OUT2 raised to enable interrupts */
	if (port->irq)
		up->port.mctrl |= TIOCM_OUT2;

	serialxr_set_mctrl(port, port->mctrl);

	spin_unlock_irqrestore(&port->lock, flags);
	/*
	 * Clear the interrupt registers again for luck, and clear the
	 * saved flags to avoid getting false values from polling
	 * routines or the previous session.
	 */
	serial_port_in(port, UART_LSR);
	serial_port_in(port, UART_RX);
	serial_port_in(port, UART_IIR);
	serial_port_in(port, UART_MSR);
	up->lsr_saved_flags = 0;
	up->msr_saved_flags = 0;
	/*
	 * Set the IER shadow for rx interrupts but defer actual interrupt
	 * enable until after the FIFOs are enabled; otherwise, an already-
	 * active sender can swamp the interrupt handler with "too much work".
	 */
	up->ier = UART_IER_RLSI | UART_IER_RDI | UART_IER_RDITO;
	serial_port_out(port, UART_IER, up->ier);

	retval = 0;

out:
	return retval;
}

static void serialxr_shutdown(struct uart_port *port)
{
	unsigned long flags;
	struct uart_8250_port *up = up_to_u8250p(port);

	/*
	 * Disable interrupts from this port
	 */
	up->ier = 0;
	serial_port_out(port, UART_IER, 0);

	spin_lock_irqsave(&port->lock, flags);

	port->mctrl &= ~TIOCM_OUT2;
	serialxr_set_mctrl(port, port->mctrl);
	
	spin_unlock_irqrestore(&port->lock, flags);
	/*
	 * Disable break condition and FIFOs
	 */
	serial_port_out(port, UART_LCR,
			serial_port_in(port, UART_LCR) & ~UART_LCR_SBC);
	serialxr_clear_fifos(up);
	/*
	 * Read data port to reset things, and then unlink from
	 * the IRQ chain.
	 */
	serial_port_in(port, UART_RX);
	/* free irq */
	free_irq(port->irq, port);
}
/*
 * flow control, modem status and rs232/422/485 interrupt
*/
static void flow_control_handle(struct uart_port *port,
								struct exar_priv *priv,
								struct ktermios *termios)
{
	unsigned char efr;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct tty_struct *tty = port->state->port.tty;

	if(termios->c_iflag & CLOCAL)
		port->flags &= ~ASYNC_CHECK_CD;
	else
		port->flags |= ASYNC_CHECK_CD;

	efr = UART_EFR_ECB;

	if(I_IXOFF(tty)){
		/*
		 * Enable xon/xoff, LCR=0xBF & SFR[0]=0
		 * make sure SFR[0] = 0
		*/
		efr |= 0xB;
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_XON1, START_CHAR(tty));
		serial_port_out(port, UART_XON2, START_CHAR(tty));
		serial_port_out(port, UART_XOFF1, STOP_CHAR(tty));
		serial_port_out(port, UART_XOFF2, STOP_CHAR(tty));
		serial_port_out(port, UART_LCR, 0);
	}

	if(I_IXANY(tty))
		up->mcr |= UART_MCR_XONANY;

	if(termios->c_iflag & (XR_RS422 | XR_RS485)){
		adv_info("Port ttyS%d under rs422/485 mode, gpio(%d) set to 0\n", 
									port->line, priv->gpio_sel);
		/* rs422/485 mode pull the relative GPIO to low */
		gpio_set_value(priv->gpio_sel, 0);

		if(termios->c_iflag & XR_RS485){
			unsigned char fctr;
			/* set RS485 auto direction control */
			serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
			fctr = serial_port_in(port, UART_FCTR);
			serial_port_out(port, UART_FCTR, fctr | XR_FCTR_DIR_CTRL);
			serial_port_out(port, UART_LCR, 0);
			priv->type = PORT_RS485;
		}else{
			priv->type = PORT_RS422;
		}
		up->ier &= ~UART_IER_MSI;
	}else{
		adv_info("Port ttyS%d under rs232 mode, gpio(%d) set to 1\n", 
									port->line, priv->gpio_sel);
		/* rs232 mode, pull the relative GPIO to high */
		gpio_set_value(priv->gpio_sel, 1);
		
		/* enable/disable RTS/CTS */
		if(termios->c_cflag & CRTSCTS){
			efr |= UART_EFR_CTS | UART_EFR_RTS;
			up->ier &= ~UART_IER_MSI;
		}else{
			up->ier |= UART_IER_MSI;
		}
		efr |= UART_EFR_ECB;
		priv->type = PORT_RS232;
	}
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	serial_port_out(port, UART_EFR, efr);
	serial_port_out(port, UART_LCR, 0);

	if(termios->c_iflag & (XR_RS422 | XR_RS485)){
		/* set RS-485 Turn-Around Delay */
		serial_port_out(port, XR_16M890_SHR, 0);
	}else{
		/* set FCL and FCH by Hysteresis Level */
		serial_port_out(port, XR_16M890_SHR, 0x09);
	}
	
	priv->rts = false;
	priv->xon = false;
	/* no support DSR/DTR */
	priv->dtr = false;
	
	if(termios->c_cflag & CRTSCTS)
		priv->rts = true;
	if(termios->c_cflag & (IXON | IXOFF))
		priv->xon = true;
}
/*
 * Programmable Baud Rate
 * retrun the divisor and set to DLL/DLD/DLM
*/
static int anybaudrate_get_divisor(struct uart_port *port, 
									unsigned int baudrate)
{
	int i, j;
	const int div_min = 1;
	const int div_max = 0xffff;
	const int ps[] = {1, 4};
	const int sp[] = {8, 16};
	const int dld_min = 0;
	const int dld_max = 15;
	int baud = (int)baudrate;
	int min_delta = 0;
	int dld;
	unsigned int prescaler;
	unsigned int sc;
	unsigned int divisor;
	unsigned int frag_divisor;
	unsigned int tmp_mcr, tmp_efr, tmp_lcr;

	min_delta = baud;	

	/*
	 * Try to try every possible sampling rate,
	 * and find the best!
	*/
	for(i = 0; i < ARRAY_SIZE(ps); i++){
		for(j = 0; j < ARRAY_SIZE(ps); j++){
			/* divisor = (clock/1) / (baudrate*sampling rate) */
			int tmp_div = port->uartclk / (baud*ps[i]*sp[j]);
			int tmp_baud;
			int tmp_delt;
			int head = dld_min;
			int tail = dld_max;
	
			if(tmp_div > div_max || tmp_div < div_min)
				continue;
		
			do{
				dld = head + (tail-head)/2;
				tmp_baud = (16*port->uartclk) / 
							(16*tmp_div*ps[i]*sp[j] + ps[i]*sp[j]*dld);
				tmp_delt = abs(baud-tmp_baud);
				if(tmp_delt < min_delta){
					min_delta = tmp_delt;
					prescaler = (unsigned int)ps[i];
					sc = (unsigned int)sp[j];
					divisor = (unsigned int)tmp_div;
					frag_divisor = (unsigned int)dld;
				}
				if(baud < tmp_baud)
					head = dld + 1;
				else
					tail = dld;
			}while(head < tail);
		}
	}

	adv_info("%s: anybaud %d, DLD %d, divisor %d, SC %d, prescaler %d\n",
				__func__, baud, frag_divisor, divisor, sc, prescaler);
	/* 
	 * MCR: Clock Prescaler Select 
	 * MCR[7]=0 : Divide by one
	 * MCR[7]=1 : Divide by four
	*/
	tmp_mcr = serial_port_in(port, UART_MCR);
	if(prescaler == 0x04)
		serial_port_out(port, UART_MCR, tmp_mcr | XR_MCR_PRESCALER);
	else
		serial_port_out(port, UART_MCR, tmp_mcr & ~(XR_MCR_PRESCALER));

	/* enable enhance mode */	
	tmp_lcr = serial_port_in(port, UART_LCR);
	serial_port_out(port, UART_LCR, UART_LCR_DLAB);
	tmp_efr = serial_port_in(port, UART_EFR);
	serial_port_out(port, UART_EFR, tmp_efr | UART_EFR_ECB);
	/* Set DLD, the sampling rate and fractional baud rate divisor */
	dld = 0;
	dld |= (0xf & (unsigned char)frag_divisor);
	dld &= 0xcf;	//clear DLD[4:5]
	if(sc == 4)
		dld |= 0x20;	//sampling rate 4x
	else if(sc == 8)
		dld |= 0x10;	//sampling rate 8x
	else
		pr_err("%s: sc = 16? %d\n", __func__, sc);
	
	serial_port_out(port, XR_16M890_DLD, dld);
	serial_port_out(port, UART_LCR, tmp_lcr);
	
	return divisor;
}

extern unsigned int uart_get_baud_rate(struct uart_port *port, 
										struct ktermios *termios,
										struct ktermios *old, 
										unsigned int min, 
										unsigned int max);

static void serialxr_set_termios(struct uart_port *port, 
								struct ktermios *termios,
			         			struct ktermios *old)
{
	unsigned char cval;
	unsigned char fctr = 0, lcr = 0;
	unsigned long flags;
	unsigned int baud, quot;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;

	cval = serialxr_compute_lcr(up, termios->c_cflag);
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/16);
	if(unlikely(baud == 0))
		baud = priv->baud;
	else
		priv->baud = baud;	

	adv_info("%s: ttyAP%d set baudrate = %d\n",
		 					__func__, priv->line, baud);
	up->lcr = cval;		/* Save computed LCR */

	/*
	 * Ok, we're now changing the port state. Do it with
	 * interrupts disabled.
	 */
	spin_lock_irqsave(&port->lock, flags);
	
	/* Update the per-port timeout */
	uart_update_timeout(port, termios->c_cflag, baud);

	port->read_status_mask = UART_LSR_OE | UART_LSR_THRE | UART_LSR_DR;
	if(termios->c_iflag & INPCK)
		port->read_status_mask |= UART_LSR_FE | UART_LSR_PE;
	if(termios->c_iflag & (IGNBRK | BRKINT | PARMRK))
		port->read_status_mask |= UART_LSR_BI;
	
	/* Characteres to ignore */
	port->ignore_status_mask = 0;
	if(termios->c_iflag & IGNPAR)
		port->ignore_status_mask |= UART_LSR_PE | UART_LSR_FE;
	if(termios->c_iflag & IGNBRK){
		port->ignore_status_mask |= UART_LSR_BI;
		/*
		 * If we're ignoring parity and break indicators,
		 * ignore overruns too (for real raw support).
		 */
		if(termios->c_iflag & IGNPAR)
			port->ignore_status_mask |= UART_LSR_OE;
	}

	/* ignore all characters if CREAD is not set */
	if((termios->c_cflag & CREAD) == 0)
		port->ignore_status_mask |= UART_LSR_DR;

	flow_control_handle(port, priv, termios);

	adv_info("ttyAP%d set mcr = 0x%0X, ier = 0x%0X\n", 
										priv->line, up->mcr, up->ier);

	serial_port_out(port, UART_MCR, up->mcr);
	serial_port_out(port, UART_IER, up->ier);
	/*
	 * Set RTL/TTL
	 */
	up->fcr = UART_FCR_ENABLE_FIFO;
	if(baud <= LOWBAUD_THRESHOLD){
		up->fcr |= UART_FCR_TRIGGER_1;
		priv->TTL = LOWBAUD_TTL;
		priv->RTL = LOWBAUD_RTL;
	}else if(baud < MIDBAUD_THRESHOLD){
		up->fcr |= UART_FCR_TRIGGER_1;
		priv->TTL = MIDBAUD_TTL;;
		priv->RTL = MIDBAUD_RTL;
	}else{
		up->fcr |= UART_FCR_TRIGGER_8;
		priv->TTL = HIGHBAUD_TTL;
		priv->RTL = HIGHBAUD_RTL;
	}
	serial_port_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
	lcr = serial_port_in(port, UART_LCR);
	serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
	fctr = serial_port_in(port, UART_FCTR);
	/*
	 * Set TTL:
	 * first, set the tx fifo trigger use table-D FCTR[5:4]=1
	 * and FCTR[7]=1 means setup tx
	 * Then, set TTL to TRIG register
	*/
	serial_port_out(port, UART_FCTR, UART_FCTR_TRGD | 
									UART_FCTR_TX | fctr);
	serial_port_out(port, UART_TRG, priv->TTL);
	/*
	 * Set RTL:
	 * first, set the rx fifo trigger use table-D FCTR[5:4]=1, 
	 * and FCTR[7]=0 means setup rx
	 * Then, set RTL to TRIG register
	*/
	serial_port_out(port, UART_FCTR, UART_FCTR_TRGD | 
										(~UART_FCTR_TX & fctr));
	serial_port_out(port, UART_TRG, priv->RTL);
	serial_port_out(port, UART_LCR, lcr);
	/* emulated UARTs (Lucent Venus 167x) need two steps */
	serial_port_out(port, UART_FCR, UART_FCR_ENABLE_FIFO);
	serial_port_out(port, UART_FCR, up->fcr);	/* set fcr */
	
	quot = anybaudrate_get_divisor(port, baud);
	if(unlikely(quot == 0)) 
		quot = (port->uartclk/16/2) / 9600;

	serial_port_out(port, UART_LCR, UART_LCR_DLAB);
	serial_port_out(port, UART_DLL, quot & 0xff);
	serial_port_out(port, UART_DLM, quot >> 8);
	serial_port_out(port, UART_LCR, up->lcr);	/* reset DLAB */

	serialxr_set_mctrl(port, port->mctrl);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void serialxr_throttle(struct uart_port *port)
{
	unsigned long flags;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;	

	spin_lock_irqsave(&port->lock, flags);

	if(!priv->throttle){
		up->ier &= ~(UART_IER_RLSI | UART_IER_RDI);
		serial_port_out(port, UART_IER, up->ier);
		priv->throttle = true;
		ADV_DEBUG_IRQ(db_thr_cnt[priv->line]++);
	}
	
	spin_unlock_irqrestore(&port->lock, flags);
}

static void serialxr_unthrottle(struct uart_port *port)
{
	unsigned long flags;
	struct uart_8250_port *up = up_to_u8250p(port);
	struct exar_priv *priv = port->private_data;
	
	spin_lock_irqsave(&port->lock, flags);

	if(priv->throttle){
		up->ier |= UART_IER_RLSI | UART_IER_RDI;
		serial_port_out(port, UART_IER, up->ier);
		priv->throttle = false;
	}
	
	spin_unlock_irqrestore(&port->lock, flags);
}

static inline int serialxr_send1char(struct uart_port *port,
									unsigned char ch)
{
	port->x_char = ch;
	serialxr_tx_chars(port);
	
	return 0;
}

static int serialxr_ioctl(struct uart_port *port, unsigned int cmd, 
						unsigned long arg)
{
	int ret = -ENOIOCTLCMD;
	unsigned long flags;
	struct tty_struct *tty = port->state->port.tty;
	struct exar_priv *priv = port->private_data;

	switch(cmd){
		case ADVSENDXON:
			adv_info("Ioctl: ttyAP%d send XON char\n", priv->line);
			ret = serialxr_send1char(port, tty->termios.c_cc[VSTART]);
			break;
		
		case ADVSENDXOFF:
			adv_info("Ioctl: ttyAP%d send XOFF char\n", priv->line);
			ret = serialxr_send1char(port, tty->termios.c_cc[VSTOP]);
			break;
	
		case ADVTIOCSERGCHARTIMEOUT:
			if(copy_to_user((void *)arg, &(priv->charto), 
									sizeof(unsigned long)))
				return -EFAULT;

			if(arg != 0){
				spin_lock_irqsave(&port->lock, flags);
				priv->charto = 0;
				spin_unlock_irqrestore(&port->lock, flags);
			}
			ret = 0;
			break;
		
		case ADVTIOCRTURDFRAME:
		{
			unsigned long tmp;
			
			if(copy_from_user((void *)(&tmp), (void *)arg,
								sizeof(unsigned long)))
			{
				ret = -EFAULT;
				break;
			}
			if(tmp == 1)
				priv->mbusreadmode = true;
			else
				priv->mbusreadmode = false;
			
			ret = 0;
			break;
		}
		case ADVTIOLOOPBACK:
		{
			unsigned long tmp;
			
			if(copy_from_user((void *)(&tmp), (void *)arg,
								sizeof(unsigned long)))
			{
				ret = -EFAULT;
				break;
			}
			if(tmp == 1)
				priv->loopback = true;
			else
				priv->loopback = false;
			
			ret = 0;
			break;
		}
		/* no supprot change RTL/TTL/FIFOSIZE via ioctl */
		case ADVTIOCSETRTL:
		case ADVTIOCSETTTL:
		case ADVTIOCSETFIFOSIZE:
		case ADVTIOCSETCHANGFLAG:
			ret = 0;
			break;
	}

	return ret;
}

static void serialxr_enable_ms(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	up->ier |= UART_IER_MSI;
	serial_port_out(port, UART_IER, up->ier);
}

static void serialxr_break_ctl(struct uart_port *port, int break_state)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);

	if (break_state == -1)
		up->lcr |= UART_LCR_SBC;
	else
		up->lcr &= ~UART_LCR_SBC;
	serial_port_out(port, UART_LCR, up->lcr);

	spin_unlock_irqrestore(&port->lock, flags);
}

static void serialxr_set_ldisc(struct uart_port *port, 
								struct ktermios *termios)
{
	struct uart_8250_port *up = up_to_u8250p(port);

	if(termios->c_line == N_PPS){
		port->flags |= UPF_HARDPPS_CD;
		spin_lock_irq(&port->lock);
		serialxr_enable_ms(port);
		spin_unlock_irq(&port->lock);
	}else{
		port->flags &= ~UPF_HARDPPS_CD;
		if(!UART_ENABLE_MS(port, termios->c_cflag)){
			up->ier &= ~UART_IER_MSI;
			spin_lock_irq(&port->lock);
			serial_port_out(port, UART_IER, up->ier);
			spin_unlock_irq(&port->lock);
		}
	}
}

static const char * serialxr_type(struct uart_port *port)
{
	int type = port->type;

	if(type >= ARRAY_SIZE(xr_uart_config)) 
		type = 0;

	return xr_uart_config[type].name;
}

static unsigned int serialxr_port_size(struct uart_8250_port *pt)
{
	if (pt->port.mapsize)
		return pt->port.mapsize;

	if (pt->port.iotype == UPIO_AU) 
		return 0x1000;

	return 8 << pt->port.regshift;
}

static void serialxr_release_port(struct uart_port *port)
{
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned int size = serialxr_port_size(up);

	switch(port->iotype){
		case UPIO_AU:
		case UPIO_TSI:
		case UPIO_MEM32:
		case UPIO_MEM32BE:
		case UPIO_MEM:
			if(!port->mapbase) break;

			if(port->flags & UPF_IOREMAP){
				iounmap(port->membase);
				port->membase = NULL;
			}

			release_mem_region(port->mapbase, size);
			break;

		case UPIO_HUB6:
		case UPIO_PORT:
			release_region(port->iobase, size);
			break;
	}
}

static int serialxr_request_port(struct uart_port *port)
{
	int ret = 0;
	struct uart_8250_port *up = up_to_u8250p(port);
	unsigned int size = serialxr_port_size(up);

	switch(port->iotype){
		case UPIO_AU:
		case UPIO_TSI:
		case UPIO_MEM32:
		case UPIO_MEM32BE:
		case UPIO_MEM:
			if(!port->mapbase) break;

			if(!request_mem_region(port->mapbase, 
					size, "eaxr_serial")) 
			{
				ret = -EBUSY;
				break;
			}

			if(port->flags & UPF_IOREMAP){
				port->membase = ioremap_nocache(port->mapbase, size);
				if(!port->membase){
					release_mem_region(port->mapbase, size);
					ret = -ENOMEM;
				}
			}
			break;

		case UPIO_HUB6:
		case UPIO_PORT:
			if(!request_region(port->iobase, size, "exar_serial"))
				ret = -EBUSY;
			break;
	}
	return ret;
}

static void serialxr_config_port(struct uart_port *port, int flags)
{
	int ret;
	struct uart_8250_port *up = up_to_u8250p(port);
	
	ret = serialxr_request_port(port);
	if(ret < 0) return;
	
	private_data_init(port);	

	if(port->iotype != up->cur_iotype)
		set_io_from_upio(port);
	
	up->port.flags |= UPF_HARD_FLOW | UPF_SOFT_FLOW;
	up->fcr = xr_uart_config[up->port.type].fcr;
}

static void serialxr_pm(struct uart_port *port, unsigned int state,
						unsigned int old)
{
	unsigned int lcr;

	lcr = serial_port_in(port, UART_LCR);
	/*
	 * To enable sleep mode, EFR[4]=1, LCR[7]=0
	*/
	if(state){
		/* sleep mode IER[4]=1 */
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, UART_EFR_ECB);
		serial_port_out(port, UART_LCR, 0);
		serial_port_out(port, UART_IER, UART_IERX_SLEEP);
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, 0);
		serial_port_out(port, UART_LCR, lcr);
	}else{
		/* wake up mode IER[4]=0 */
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, UART_EFR_ECB);
		serial_port_out(port, UART_LCR, 0);
		serial_port_out(port, UART_IER, 0);
		serial_port_out(port, UART_LCR, UART_LCR_CONF_MODE_B);
		serial_port_out(port, UART_EFR, 0);
		serial_port_out(port, UART_LCR, lcr);
	}	
}

static struct uart_ops serialxr_pops = {
	.tx_empty		= serialxr_tx_empty,
	.set_mctrl		= serialxr_set_mctrl,
	.get_mctrl		= serialxr_get_mctrl,
	.stop_tx		= serialxr_stop_tx,
	.start_tx		= serialxr_start_tx,
	.throttle		= serialxr_throttle,
	.unthrottle		= serialxr_unthrottle,
	.stop_rx		= serialxr_stop_rx,
	.enable_ms		= serialxr_enable_ms,
	.break_ctl		= serialxr_break_ctl,
	.startup		= serialxr_startup,
	.shutdown		= serialxr_shutdown,
	.set_termios	= serialxr_set_termios,
	.set_ldisc		= serialxr_set_ldisc,
	.pm				= serialxr_pm,
	.type			= serialxr_type,
	.release_port	= serialxr_release_port,
	.request_port	= serialxr_request_port,
	.config_port	= serialxr_config_port,
	.ioctl			= serialxr_ioctl, 
};

static void serialxr_handle_one_port(struct uart_8250_port *up)
{
	unsigned char lsr;
	unsigned long flags;
	struct uart_port *port = &up->port;
	
	lsr = (unsigned char)serial_port_in(port, UART_LSR);
	DEBUG_INTR("timeout irq status = %x...", status);

	spin_lock_irqsave(&port->lock, flags);

	if(lsr & (UART_LSR_DR | UART_LSR_BI))
		serialxr_rx_chars(port, &lsr);
	
	serialxr_modem_status(port);
	
	if(lsr & UART_LSR_THRE)
		serialxr_tx_chars(port);
	
	spin_unlock_irqrestore(&port->lock, flags);
}	

static void serialxr_timeout(unsigned long data)
{
	unsigned int iir;
	struct uart_8250_port *up = (struct uart_8250_port *)data;

	iir = serial_in(up, UART_IIR);
	if(!(iir & UART_IIR_NO_INT))
		serialxr_handle_one_port(up);

	mod_timer(&up->timer, jiffies + uart_poll_timeout(&up->port));
}

/*
 * The exar 16m890 uart chip default mode setting
 * default set to RS232 mode
 *
 * gpio low  : rs422/485 mode
 * gpio high : rs232 mode
*/
static int XR_set_default_mode(struct uart_8250_port *uart)
{
	int err;
	int gpio;
	enum of_gpio_flags flags;
	struct device_node *np = uart->port.dev->of_node;
	
	if(!np){
		pr_err("%s: No device tree node!\n", __func__);
		return -1;
	}

	gpio = of_get_named_gpio_flags(np, "mode-sel-gpio", 0, &flags);
	if(!gpio_is_valid(gpio)){
		pr_err("Cannot find %s GPIO defined!\n", np->name);
		return -1;
	}

	/* GPIO get number request */
	err = devm_gpio_request(uart->port.dev, gpio, "exar_uart_sel");
	if(err){
		pr_err("%s: GPIO(%d) request failed!\n", np->name, gpio);
		return -1;
	}

	/* Set GPIO as high, uart default mode as RS232 */
	err = gpio_direction_output(gpio, 1);
	if(err < 0){
		pr_err("%s: GPIO(%d) set high failed!\n", np->name, gpio);
		return -1;
	}

	adv_info("ttyAP%d set to %s mode\n", uart->port.line,
			gpio_get_value_cansleep(gpio)? "rs232" : "rs422/485");
	gpio_export(gpio, 1);

	return gpio;
}

static void * 
XR_private_data_alloc(struct uart_8250_port *uart, int line, int gpio)
{
	struct exar_priv *priv;

	priv = devm_kzalloc(uart->port.dev,
						sizeof(struct exar_priv), 
						GFP_KERNEL);
	if(!priv){
		pr_err("ttyAP%d alloc private data failed!\n", line);
		return NULL;
	}
	priv->line = line;
	priv->gpio_sel = gpio;
	
	return (void *)priv;
}

#define XR_MAJOR	30
#define XR_MINOR	0
#define SERIAL_NAME "ttyAP"
static struct uart_driver serialxr_drv = {
	.owner			= THIS_MODULE,
	.driver_name 	= "serialxr",
	.dev_name		= SERIAL_NAME,
	.major			= XR_MAJOR,
	.minor			= XR_MINOR,
	.cons			= NULL,
	.nr 			= NR_PORTS,
};

static DEFINE_MUTEX(serial_mutex);
static struct uart_8250_port serialxr_ports[NR_PORTS];

static struct uart_8250_port 
*serialxr_find_match_or_unused(struct uart_port *port)
{
	int i;
	/*
	 * We didn't find a matching entry, so look for the first
	 * free entry.  We look for one which hasn't been previously
	 * used (indicated by zero iobase).
	 */
	for (i = 0; i < NR_PORTS; i++){
		if (serialxr_ports[i].port.type == PORT_UNKNOWN &&
		    serialxr_ports[i].port.iobase == 0)
		{
			serialxr_ports[i].port.line = i;
			return &serialxr_ports[i];
		}
	}

	return NULL;
}
/**
 *	serialxr_register_exar_port - register a serial port
 *	@up: serial port template
 *
 *	Configure the serial port specified by the request. If the
 *	port exists and is in use, it is hung up and unregistered
 *	first.
 *
 *	The port is then probed and if necessary the IRQ is autodetected
 *	If this fails an error is returned.
 *
 *	On success the port is ready to use and the line number is returned.
 */
int serialxr_register_16m890_port(struct uart_8250_port *up)
{
	int gpio;
	int ret = -ENOSPC;
	struct uart_8250_port *uart;
	unsigned int type = up->port.type;

	if (up->port.uartclk == 0)
		return -EINVAL;

	mutex_lock(&serial_mutex);

	uart = serialxr_find_match_or_unused(&up->port);
	if(uart){
		uart->port.type			= up->port.type;
		uart->port.iobase       = up->port.iobase;
		uart->port.membase      = up->port.membase;
		uart->port.irq          = up->port.irq;
		uart->port.irqflags     = up->port.irqflags;
		uart->port.uartclk      = up->port.uartclk;
		uart->port.fifosize     = up->port.fifosize;
		uart->port.regshift     = up->port.regshift;
		uart->port.iotype       = up->port.iotype;
		uart->port.flags        = up->port.flags | UPF_BOOT_AUTOCONF;
		uart->port.mapbase      = up->port.mapbase;
		uart->port.mapsize      = up->port.mapsize;
		uart->tx_loadsz			= up->tx_loadsz;
		uart->capabilities		= up->capabilities;
		uart->port.dev			= up->port.dev;
		/* The driver's ops */
		uart->port.ops 			= &serialxr_pops;
	
		if(!uart->port.fifosize)
			uart->port.fifosize = xr_uart_config[type].fifo_size;
		if(!uart->tx_loadsz)
			uart->tx_loadsz = xr_uart_config[type].tx_loadsz;
		if(!uart->capabilities)
			uart->capabilities = xr_uart_config[type].flags;

		spin_lock_init(&uart->port.lock);
		uart->cur_iotype = 0xFF;
		/* ALPHA_KLUDGE_MCR needs to be killed */
		uart->mcr_mask = ~ALPHA_KLUDGE_MCR;
		uart->mcr_force = ALPHA_KLUDGE_MCR;	
	
		set_io_from_upio(&uart->port);

		init_timer(&uart->timer);
		uart->timer.function = serialxr_timeout;

		gpio = XR_set_default_mode(uart);
		if(gpio == -1)
			return -ENOMEM;
		
		uart->port.private_data = XR_private_data_alloc(up, uart->port.line, gpio);
		if(uart->port.private_data == NULL)
			return -ENOMEM;

		ret = uart_add_one_port(&serialxr_drv, &uart->port);
		if(ret == 0)
			ret = uart->port.line;
	}
	mutex_unlock(&serial_mutex);

	return ret;
}
EXPORT_SYMBOL(serialxr_register_16m890_port);

/**
 *	serialxr_unregister_16m890_port - 
 *	remove a exar 16m890 serial port at runtime
 *	@line: serial line number
 *
 *	Remove one serial port.  This may not be called from interrupt
 *	context.  We hand the port back to the our control.
 */
void serialxr_unregister_16m890_port(int line)
{
	struct uart_8250_port *uart = &serialxr_ports[line];

	mutex_lock(&serial_mutex);

	uart_remove_one_port(&serialxr_drv, &uart->port);
	uart->port.dev = NULL;

	mutex_unlock(&serial_mutex);
}
EXPORT_SYMBOL(serialxr_unregister_16m890_port);

static void 
serialxr_line_info(struct seq_file *m, struct uart_driver *drv, int i)
{
	struct uart_state *state = drv->state + i;
	struct uart_port *port = state->uart_port;
	struct exar_priv *priv = port->private_data;
	
	seq_printf(m, "Port%d TTX:%lu\n", i, (unsigned long)port->icount.tx);
	seq_printf(m, "Port%d TRX:%lu\n", i, (unsigned long)port->icount.rx);
	seq_printf(m, "Port%d TX:%lu\n", i, priv->tx);
	seq_printf(m, "Port%d RX:%lu\n", i, priv->rx);
	seq_printf(m, "Port%d TYPE%d\n", i, priv->type);
	seq_printf(m, "Port%d FLOW:RTS:%d XON:%d DTR:%d\n", 
				i, priv->rts, priv->xon, priv->dtr);
	seq_printf(m, "Port%d PARM:DATA:%u STOP:%u PARY:%u\n", 
				i, priv->data, priv->stop, priv->pary);
}

static int serialxr_proc_show(struct seq_file *m, void *v)
{
	int i;
	struct uart_driver *drv = m->private;

	for (i = 0; i < drv->nr; i++)
		serialxr_line_info(m, drv, i);	
	
	return 0;
}

static int adv_proc_open(struct inode *inode, struct file *file)
{
	/* link to uart_driver */
	return single_open(file, serialxr_proc_show, &serialxr_drv);
}

static const struct file_operations serialxr_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= adv_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifdef __ADV_DEBUG_IRQ__
static void debug_info(struct seq_file *m, struct uart_driver *drv, int i)
{
	unsigned int ier;
	struct uart_state *state = drv->state + i;
	struct uart_port *port = state->uart_port;
	struct exar_priv *priv = port->private_data;

	ier = serial_port_in(port, UART_IER);	
	seq_printf(m, "ttyAP%d: irq(%llx), ResetIERcnt(%d), "
					"IER_THRI(%s), IER_RLSI(%s), txStatus(%d), "
					"thrCnt(%d)\n", 
					priv->line, 
					db_irq_record[i], 
					db_re_ier_cnt[i],
					ier&UART_IER_THRI ? "o":"x",
					ier&UART_IER_RLSI ? "o":"x",
					db_txstat[i],
					db_thr_cnt[i]);
}

static int adv_debug_show(struct seq_file *m, void *v)
{
	int i;
	struct uart_driver *drv = m->private;

	seq_printf(m, "\n<IRQ>\n\t"
					"1: Rx data interrupt\n\t"
					"2: Ms interrupt\n\t"
					"3: Rx data timeout\n\t"
					"4: Rx line status\n\t"
					"5: Tx empty\n");
	seq_printf(m, "<Tx Status>\n\t"
					"-1: xon/xoff\n\t"
					"-2: uart_tx_stopped\n\t"
					"-3: ring buffer empty\n\t"
					">=0: data pending in buffer\n");
	seq_printf(m, "--------------------------------------------------\n");
	for (i = 0; i < drv->nr; i++){
		debug_info(m, drv, i);
	}

	return 0;
}

static int adv_debug_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, adv_debug_show, &serialxr_drv);
}

static const struct file_operations adv_debug_irq_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= adv_debug_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif
static int __init serialxr_init(void)
{
	int ret = 0;
	struct proc_dir_entry *proc_file_entry;

	do{
		ret = uart_register_driver(&serialxr_drv);
		if(ret){
			pr_err("Exar 16m890 serial driver register failed!\n");
			ret = -ENOMEM;
			break;
		}
		pr_info("Exar 16m890 serial driver Ver 1.0 loaded\n");

		proc_file_entry = proc_create("driver/serial", 0, NULL, &serialxr_proc_fops);
		if(proc_file_entry == NULL){
			pr_err("Exar 16m890 serial driver proc register failed!\n");
			ret = -ENOMEM;
			break;
		}
		pr_info("Exar 16m890 serial infomation proc created\n");
#ifdef __ADV_DEBUG_IRQ__
		proc_file_entry = proc_create("advSerialDebug", 0, NULL, &adv_debug_irq_proc_fops);
		if(proc_file_entry == NULL){
			//if failed, we don't care
			pr_err("Warnning: Create adv serial debug infomation failed!\n");
		}
#endif

	}while(0);

	return ret;
}

static void __exit serialxr_exit(void)
{
	uart_unregister_driver(&serialxr_drv);
	remove_proc_entry("driver/serial", NULL);
#ifdef __ADV_DEBUG_IRQ__
	remove_proc_entry("advSerialDebug", NULL);
#endif
}

module_init(serialxr_init);
module_exit(serialxr_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Exar 16m890 uart driver for Advantech EKI-series");
