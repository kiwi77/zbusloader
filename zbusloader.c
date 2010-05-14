/*
 * Copyright(C) 2007,2008 Stefan Siegl <stesie@brokenpipe.de>
 * Copyright(C) 2008 Christian Dietrich <stettberger@dokucode.de>

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
 */

/* Fuses:       Extended        0xFF
 *              High            0x9A
 *              Low             0xFF
 *http://frank.circleofcurrent.com/fusecalc/fusecalc.php?chip=atmega169&LOW=FF&HIGH=9A&EXTENDED=FF&LOCKBIT=FF
 */

#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <util/delay.h>

#include "pinconfig.h"

#define HR20

#ifdef HR20
	#include "../ethersex/pinning.c"
	#include "../ethersex/hardware/lcd/hr20.h"
#endif

#define noinline __attribute__((noinline))
#define naked    __attribute__((naked))

/* We need to store magic byte + page-number + a whole page + crc */
#define BUFSZ (SPM_PAGESIZE + 3)
unsigned char zbusloader_buf[BUFSZ];
unsigned char zbusloader_tx_buf[2];

#define MAGIC_FLASH_PAGE 0x23
#define MAGIC_LAUNCH_APP 0x42
#define ZBUS_UBRR 12

uint8_t TXLine ;
uint8_t RXLine ;

static void
timer_init (void)
{
  /* select clk/256 prescaler,
     at 8 MHz this means 31250 ticks per seconds, i.e. total timeout
     of 2.09 seconds. */
  TCCR1B = _BV (CS12);		// TCCR1B(CS12) = 1

  /* enable overflow interrupt of Timer 1 */
#ifdef HR20
  TIMSK1 = _BV (TOIE1);		// TIMSK1(TOIE1) = 1
#else
  TIMSK = _BV (TOIE1);		// TIMSK(TOIE1) = 1
#endif

  sei ();
}

#define USE_2

static void uart_xxx(void) {
	#define BAUD 76800
	#include <util/setbaud.h>
	UBRRH = UBRRH_VALUE;
	UBRRL = UBRRL_VALUE;
	#if USE_2X
		UCSRA |= (1 << U2X);
	#else
		UCSRA &= ~(1 << U2X);
	#endif
}

void
zbus_init(void)
{
    /* set baud rate */
    //UBRRH = ZBUS_UBRR >> 8;
    //UBRRL = ZBUS_UBRR;
    uart_xxx();

    ZBUS_TX_DDR |= _BV(ZBUS_TX_PIN);
    ZBUS_TX_PORT &= ~_BV(ZBUS_TX_PIN);
    /* set mode: 8 bits, 1 stop, no parity, asynchronous usart
       and Set URSEL so we write UCSRC and not UBRRH */
    //UCSRC = _BV(UCSZ0) | _BV(UCSZ1) | _BV(URSEL);
    UCSRC = _BV(UCSZ0) | _BV(UCSZ1);
    /* Enable the RX interrupt and receiver and transmitter */
    UCSRB |= _BV(TXEN) | _BV(RXEN); 

}

#ifdef HR20
	void hr20_lcd_init (void) {
		LCDCCR |= 15 ;
		LCDCRB = (1<<LCDMUX1) | (1<<LCDPM2)| (1<<LCDPM0) ;
		LCDFRR = (1<<LCDCD2) | (1<<LCDCD1) | (1<<LCDCD0) | (1<<LCDPS2) | (1<<LCDPS1) | (1<<LCDPS0) ;
		LCDCRA = (1<<LCDEN) | (1<<LCDAB);
		LCD_SEG_SET (LCD_SEG_PROG);
		LCD_SEG_SET (LCD_SEG_B11);
		LCD_SEG_SET (LCD_SEG_B12);
		RXLine = 4 ;
		TXLine = 44 ;
	}

	void rx_bar(void) {
		LCD_SEG_CLEAR ( RXLine / 4 );
		RXLine ++ ;
		if ( RXLine >= 41 ) { RXLine = 4 ; };
		LCD_SEG_SET ( RXLine / 4 );
	}

	void tx_bar(void) {
		LCD_SEG_CLEAR ( TXLine / 4 );
		TXLine ++ ;
		if ( TXLine >= 81 ) { TXLine = 44 ; };
		LCD_SEG_SET ( TXLine / 4 );
	}
#endif

static void
flash_page (void)
{
#if SPM_PAGESIZE < 256
  uint8_t i;
#else
  uint16_t i;
#endif

  uint16_t page = zbusloader_buf[1] * SPM_PAGESIZE;

  eeprom_busy_wait();

  boot_page_erase(page);
  boot_spm_busy_wait();

  for(i = 0; i < SPM_PAGESIZE; i += 2) {
    /* Set up little-endian word. */
    uint16_t w = zbusloader_buf[2 + i];
    w += zbusloader_buf[3 + i] << 8;
        
    boot_page_fill (page + i, w);
  }

  boot_page_write (page);
  boot_spm_busy_wait();

  /* Reenable RWW-section again. */
  boot_rww_enable ();
}


void
zbusloader_rx ()
{
  uint8_t last = 0;
  uint8_t current, i = 0;
  uint8_t started = 0;
  while (1) {
    /* While an byte is recieved */
    while ( !(UCSRA & _BV(RXC)) );
      if ((UCSRA & _BV(DOR)) || (UCSRA & _BV(FE))) {
	current = UDR;
	continue;
      }
      current = UDR;
      if (last == '\\') {
	if (current == '0') {
	  started = 1;

#ifdef STATUS_LED_RX
	  STATUS_LED_PORT |= _BV (STATUS_LED_RX);
#endif
#ifdef HR20
	rx_bar();
#endif
	}
	else if (current == '1') 
	  break;
	else 
	  goto append_data;
      } else {
	if (current == '\\') goto save_current;
append_data:
	if(started == 0) goto save_current;
	zbusloader_buf[i++] = current;
      }
save_current:
      last = current;
  }
#ifdef STATUS_LED_RX
	  STATUS_LED_PORT &= ~_BV (STATUS_LED_RX);
#endif
  if (i <= BUFSZ)
    return;
  zbusloader_buf[0] = 0;
}

void
zbus_send_byte(uint8_t data) {
  UDR = data;
  while ( !( UCSRA & _BV(TXC)) );
  UCSRA |= _BV(TXC);
}

void
zbusloader_tx_reply(void) 
{
#ifdef STATUS_LED_TX
  STATUS_LED_PORT |= _BV (STATUS_LED_TX);
#endif
#ifdef HR20
	tx_bar();
#endif
  ZBUS_TX_PORT |= _BV(ZBUS_TX_PIN);
  /* Start Conditon */
  zbus_send_byte('\\');
  zbus_send_byte('0');
  /* byte one */
  zbus_send_byte(zbusloader_tx_buf[0]);
  /* byte two: can be \ */
  if (zbusloader_tx_buf[1] == '\\') {
    zbus_send_byte('\\');
  }
  zbus_send_byte(zbusloader_tx_buf[1]);
  /* Stop Condition */
  zbus_send_byte('\\');
  zbus_send_byte('1');

  ZBUS_TX_PORT &= ~_BV(ZBUS_TX_PIN);
#ifdef STATUS_LED_TX
  STATUS_LED_PORT &= ~_BV (STATUS_LED_TX);
#endif
}


static void
crc_update (unsigned char *crc, uint8_t data)
{
  for (uint8_t j = 0; j < 8; j ++)
    {
      if ((*crc ^ data) & 1)
	*crc = (*crc >> 1) ^ 0x8c;
      else
        *crc = (*crc >> 1);

      data = data >> 1;
    }
}


static uint8_t
crc_check (void)
{
  unsigned char crc_chk = 0;
  unsigned char *ptr = zbusloader_buf + 2;

  for (uint8_t i = 0; i < SPM_PAGESIZE; i ++)
    crc_update (&crc_chk, *(ptr ++));

  /* subtract one from the other, this is far cheaper than comparation */
  crc_chk -= *ptr;
  return crc_chk;
}

naked void
zbusloader_main (void)
{
  timer_init ();
  zbus_init();
#ifdef HR20
	hr20_lcd_init();
#endif
  UCSRA |= _BV(TXC);
#ifdef STATUS_LED_RX
  STATUS_LED_DDR |= _BV (STATUS_LED_RX);
#endif

#ifdef STATUS_LED_TX
  STATUS_LED_DDR |= _BV (STATUS_LED_TX);
#endif

  for (;;) 
    {
      /* try to receive a packet */
      zbusloader_rx ();

      /* check packet validity */
      if (zbusloader_buf[0] == MAGIC_LAUNCH_APP) {
	_delay_us(50);
        zbusloader_tx_buf[0] = 0x42;
        zbusloader_tx_buf[1] = 0;
        zbusloader_tx_reply ();
	break;
      }

      if (zbusloader_buf[0] != MAGIC_FLASH_PAGE)
	continue;		/* unknown magic, ignore. */

      if (crc_check ())
	continue;		/* crc invalid */

      /* clear global interrupt flag, so timer interrupt cannot
         call the application any longer. */
      cli ();

      /* flash page */
      flash_page ();

      /* transmit reply */
      zbusloader_tx_buf[0] = 0x23;
      zbusloader_tx_buf[1] = zbusloader_buf[BUFSZ - 1];
      zbusloader_tx_reply ();
    }
  
  /* leave here, thusly jump into application now */
  __asm volatile ("ret");
#ifdef STATUS_LED_TX
  STATUS_LED_PORT |= _BV (STATUS_LED_TX);
#endif
}

