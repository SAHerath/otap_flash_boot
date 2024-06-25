/*****************************************************************************
Title:     STK500v2 compatible bootloader
           Modified for Wiring board ATMega128-16MHz
Author:    Peter Fleury <pfleury@gmx.ch>   http://jump.to/fleury
Compiler:  avr-gcc 3.4.5 or 4.1 / avr-libc 1.4.3
Hardware:  All AVRs with bootloader support, tested with ATmega8
License:   GNU General Public License

Modified:  Worapoht Kornkaewwattanakul <dev@avride.com>   http://www.avride.com
Date:      17 October 2007
Update:    1st, 29 Dec 2007 : Enable CMD_SPI_MULTI but ignore unused command by return 0x00 byte response..
Compiler:  WINAVR20060421
Description: add timeout feature like previous Wiring bootloader

Modified:  Supun Herath <supun@utequip.com> 
Date:      20 Aug 2019
Update:
Compiler:  avr-gcc (AVR_8_bit_GNU_Toolchain_3.6.1_1750) 5.4.0
Description: add capability to write internal flash through external flash chip using spi protocol

DESCRIPTION:
    This program allows an AVR with bootloader capabilities to
    read/write its own Flash/EEprom. To enter Programming mode
    an input pin is checked. If this pin is pulled low, programming mode
    is entered. If not, normal execution is done from $0000
    "reset" vector in Application area.
    Size fits into a 1024 word bootloader section
	when compiled with avr-gcc 4.1
	(direct replace on Wiring Board without fuse setting changed)

USAGE:
    - Set AVR MCU type and clock-frequency (F_CPU) in the Makefile.
    - Set baud rate below (AVRISP only works with 115200 bps)
    - compile/link the bootloader with the supplied Makefile
    - program the "Boot Flash section size" (BOOTSZ fuses),
      for boot-size 1024 words:  program BOOTSZ01
    - enable the BOOT Reset Vector (program BOOTRST)
    - Upload the hex file to the AVR using any ISP programmer
    - Program Boot Lock Mode 3 (program BootLock 11 and BootLock 12 lock bits) // (leave them)
    - Reset your AVR while keeping PROG_PIN pulled low // (for enter bootloader by switch)
    - Start AVRISP Programmer (AVRStudio/Tools/Program AVR)
    - AVRISP will detect the bootloader
    - Program your application FLASH file and optional EEPROM file using AVRISP

Note:
    Erasing the device without flashing, through AVRISP GUI button "Erase Device"
    is not implemented, due to AVRStudio limitations.
    Flash is always erased before programming.

	AVRdude:
	Please uncomment #define REMOVE_CMD_SPI_MULTI when using AVRdude.
	Comment #define REMOVE_PROGRAM_LOCK_BIT_SUPPORT to reduce code size
	Read Fuse Bits and Read/Write Lock Bits is not supported

NOTES:
    Based on Atmel Application Note AVR109 - Self-programming
    Based on Atmel Application Note AVR068 - STK500v2 Protocol

LICENSE:
    Copyright (C) 2006 Peter Fleury

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*****************************************************************************/

//************************************************************************
//*	Edit History
//************************************************************************
//*	Jul  7,	2010	<MLS> = Mark Sproul msproul@skycharoit.com
//*	Jul  7,	2010	<MLS> Working on mega2560. No Auto-restart
//*	Jul  7,	2010	<MLS> Switched to 8K bytes (4K words) so that we have room for the monitor
//*	Jul  8,	2010	<MLS> Found older version of source that had auto restart, put that code back in
//*	Jul  8,	2010	<MLS> Adding monitor code
//*	Jul 11,	2010	<MLS> Added blinking LED while waiting for download to start
//*	Jul 11,	2010	<MLS> Added EEPROM test
//*	Jul 29,	2010	<MLS> Added recchar_timeout for timing out on bootloading
//*	Aug 23,	2010	<MLS> Added support for atmega2561
//*	Aug 26,	2010	<MLS> Removed support for BOOT_BY_SWITCH
//*	Sep  8,	2010	<MLS> Added support for atmega16
//*	Nov  9,	2010	<MLS> Issue 392:Fixed bug that 3 !!! in code would cause it to jump to monitor
//*	Jun 24,	2011	<MLS> Removed analogRead (was not used)
//*	Dec 29,	2011	<MLS> Issue 181: added watch dog timmer support
//*	Dec 29,	2011	<MLS> Issue 505:  bootloader is comparing the seqNum to 1 or the current sequence 
//*	Jan  1,	2012	<MLS> Issue 543: CMD_CHIP_ERASE_ISP now returns STATUS_CMD_FAILED instead of STATUS_CMD_OK
//*	Jan  1,	2012	<MLS> Issue 543: Write EEPROM now does something (NOT TESTED)
//*	Jan  1,	2012	<MLS> Issue 544: stk500v2 bootloader doesn't support reading fuses
//************************************************************************

//************************************************************************
//*	these are used to test issues
//*	http://code.google.com/p/arduino/issues/detail?id=505
//*	Reported by mark.stubbs, Mar 14, 2011
//*	The STK500V2 bootloader is comparing the seqNum to 1 or the current sequence 
//*	(IE: Requiring the sequence to be 1 or match seqNum before continuing).  
//*	The correct behavior is for the STK500V2 to accept the PC's sequence number, and echo it back for the reply message.
#define	_FIX_ISSUE_505_
//************************************************************************

#include	<inttypes.h>
#include	<avr/io.h>
#include	<avr/interrupt.h>
#include	<avr/boot.h>
#include	<avr/pgmspace.h>
#include	<util/delay.h>
#include	<avr/eeprom.h>
#include	<avr/common.h>
#include	<stdlib.h>
#include	"command.h"


// #define	_DEBUG_WITH_LEDS_


/*
 * Uncomment the following lines to save code space
 */
//#define	REMOVE_PROGRAM_LOCK_BIT_SUPPORT		// disable program lock bits
//#define	REMOVE_BOOTLOADER_LED				// no LED to show active bootloader
//#define	REMOVE_CMD_SPI_MULTI				// disable processing of SPI_MULTI commands, Remark this line for AVRDUDE <Worapoht>
//




//************************************************************************
//*	LED on pin "PROGLED_PIN" on port "PROG_PORT"
//*	indicates that bootloader is active
//*	PG2 -> LED on Wiring board
//************************************************************************
#define	BLINK_LED_WHILE_WAITING

#define PROG_PORT		PORTB
#define PROG_DDR		DDRB
#define PROGLED_PIN		PINB6
#define PROGCS_PIN		PINB0

#define EXMEM_JEDEC		0xEF4017
#define EXMEM_SECT_SIZE	4096		// bytes
#define EXMEM_PAGE_SIZE 256			// bytes

#define BOOT_EROR_ADDR  256
#define BOOT_STAT_ADDR  4096    // byte addr
#define BOOT_CSUM_ADDR  4352    // byte addr
#define BOOT_DATA_ADDR  8192    // byte addr

/*
 * define CPU frequency in Mhz here if not defined in Makefile
 */
#ifndef F_CPU
	#define F_CPU 16000000UL
#endif

#define	_BLINK_LOOP_COUNT_	(F_CPU / 2250)


/*
 * Calculate the address where the bootloader starts from FLASHEND and BOOTSIZE
 * (adjust BOOTSIZE below and BOOTLOADER_ADDRESS in Makefile if you want to change the size of the bootloader)
 */
//#define BOOTSIZE 1024
#if FLASHEND > 0x0F000
	#define BOOTSIZE 8192
#else
	#define BOOTSIZE 2048
#endif

#define APP_END  (FLASHEND -(2*BOOTSIZE) + 1)


/*
 * use 16bit address variable for ATmegas with <= 64K flash
 */
#if defined(RAMPZ)
	typedef uint32_t address_t;
#else
	typedef uint16_t address_t;
#endif


/*
 * since this bootloader is not linked against the avr-gcc crt1 functions,
 * to reduce the code size, we need to provide our own initialization
 */
void __jumpMain	(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
#include <avr/sfr_defs.h>

//#define	SPH_REG	0x3E
//#define	SPL_REG	0x3D

//*****************************************************************************
void __jumpMain(void)
{
//*	July 17, 2010	<MLS> Added stack pointer initialzation
//*	the first line did not do the job on the ATmega128

	asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );

//*	set stack pointer to top of RAM

	asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );

	asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );

	asm volatile ( "clr __zero_reg__" );									// GCC depends on register r1 set to 0
	asm volatile ( "out %0, __zero_reg__" :: "I" (_SFR_IO_ADDR(SREG)) );	// set SREG to 0
	asm volatile ( "jmp main");												// jump to main()
}


//*****************************************************************************
void delay_ms(unsigned int timedelay)
{
	unsigned int i;
	for (i=0;i<timedelay;i++)
	{
		_delay_ms(0.5);
	}
}



#define	MAX_TIME_COUNT	(F_CPU >> 1)
//*****************************************************************************

			
//*	for watch dog timer startup
void (*app_start)(void) = 0x0000;

//function prototypes//
uint8_t FLASH_Init(void);
void FLASH_WriteEnable(void);
void FLASH_ResetEnable(void);
void FLASH_Reset(void);
void FLASH_UntilReady(void);
void FLASH_ReadByte(uint32_t faddr, uint8_t *rdata);
void FLASH_ReadWord(uint32_t faddr, uint16_t *rdata);
void FLASH_WriteByte(uint32_t faddr, uint8_t wdata);
void FLASH_EraseSector(uint32_t faddr);

void SPI_Init(void);
uint8_t SPI_Transfer(uint8_t spidata);



//*****************************************************************************
int main(void)
{
	address_t		address			=	0;
	address_t		eraseAddress	=	0;
	unsigned char	msgParseState;
	unsigned int	ii				=	0;
	unsigned char	checksum		=	0;
	unsigned char	seqNum			=	0;
	unsigned int	msgLength		=	0;
	unsigned char	msgBuffer[285];
	unsigned char	c, *p;
	unsigned char   isLeave = 0;

	unsigned long	boot_timeout;
	unsigned long	boot_timer;
	unsigned int	boot_state;

	//*	some chips dont set the stack properly
	asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );


	//*	handle the watch dog timer
	uint8_t	mcuStatusReg;
	mcuStatusReg	=	MCUSR;

	__asm__ __volatile__ ("cli");
	__asm__ __volatile__ ("wdr");
	MCUSR	=	0;
	WDTCSR	|=	_BV(WDCE) | _BV(WDE);
	WDTCSR	=	0;
	__asm__ __volatile__ ("sei");
	// check if WDT generated the reset, if so, go straight to app
	if (mcuStatusReg & _BV(WDRF))
	{
		app_start();
	}



	boot_timer	=	0;
	boot_state	=	0;

#ifdef BLINK_LED_WHILE_WAITING
//	boot_timeout	=	 90000;		//*	should be about 4 seconds
//	boot_timeout	=	170000;
	boot_timeout	=	 20000;		//*	should be about 1 second
#else
	boot_timeout	=	3500000; // 7 seconds , approx 2us per step when optimize "s"
#endif
	/*
	 * Branch to bootloader or application code ?
	 */

#ifndef REMOVE_BOOTLOADER_LED
	/* PROG_PIN pulled low, indicate with LED that bootloader is active */
	PROG_DDR	|=	(1<<PROGLED_PIN);
//	PROG_PORT	&=	~(1<<PROGLED_PIN);	// active low LED ON
	PROG_PORT	|=	(1<<PROGLED_PIN);	// active high LED ON

#ifdef _DEBUG_WITH_LEDS_
	for (ii=0; ii<3; ii++)
	{
		PROG_PORT	&=	~(1<<PROGLED_PIN);	// turn LED off
		delay_ms(100);
		PROG_PORT	|=	(1<<PROGLED_PIN);	// turn LED on
		delay_ms(100);
	}
#endif

#endif

	asm volatile ("nop");			// wait until port has changed

	SPI_Init();
	if(FLASH_Init())
	{
		  
		uint32_t fDataAddr = BOOT_DATA_ADDR;
		uint32_t fCsumAddr = BOOT_CSUM_ADDR;
		uint8_t  fRdTries  = 0;
		uint16_t fii, fjj;
		uint16_t fBootStat = 0;
		uint16_t fDataLen  = 0;
		uint16_t fChSumCal = 0;
		uint16_t fChSumRev = 0;
		uint32_t bootPageAddr = 0;

		FLASH_ReadWord(BOOT_STAT_ADDR, &fBootStat);
		  
		if(fBootStat == 0x2323)
		{
			FLASH_ReadWord((BOOT_STAT_ADDR+2), &fDataLen);

			for(fjj=0; fjj<fDataLen; fjj++)
			{
				fRdTries  = 0;
				fChSumCal = 0;
				PROG_PORT &= ~(1<<PROGCS_PIN);
				SPI_Transfer(0x03);
				SPI_Transfer((fDataAddr >> 16) & 0xff);
				SPI_Transfer((fDataAddr >> 8) & 0xff);
				SPI_Transfer(fDataAddr & 0xff);
				for(fii=0; fii<256; fii++)
				{
					msgBuffer[fii] = SPI_Transfer(0);
					fChSumCal += msgBuffer[fii];
				}
				PROG_PORT |= (1<<PROGCS_PIN);
				fDataAddr += EXMEM_PAGE_SIZE;
				_delay_us(10);

				FLASH_ReadWord(fCsumAddr, &fChSumRev);
				  
				if(fChSumRev == fChSumCal)
				{
					/////////////////checksum Matched
					uint8_t  fLSByte, fMSByte;
					uint16_t fData;
					
					address = bootPageAddr;
								
					if(bootPageAddr < APP_END)				// erase only main section (bootloader protection)
					{
					    boot_page_erase(bootPageAddr);		// Perform page erase
					    boot_spm_busy_wait();				// Wait until the memory is erased
					}
					
					for(fii=0; fii<256; fii+=2)
					{
						fLSByte = msgBuffer[fii];
						fMSByte = msgBuffer[fii+1];
						fData   = (fMSByte << 8) | fLSByte;
						          
						boot_page_fill(address, fData);		// fill boot page
						address += 2;						// inc boot page address
					}

					boot_page_write(bootPageAddr);			// write boot page
					boot_spm_busy_wait();
					boot_rww_enable();						// Re-enable the RWW section
					
					bootPageAddr += SPM_PAGESIZE;			// SPM_PAGESIZE=256  point to next page to be erase
					fCsumAddr += 2;

				}
				else
				{
					///////////////////checksum Wrong
					fRdTries++;
					if(fRdTries>3)
					{
						////////////fail to read page
						FLASH_WriteByte(BOOT_EROR_ADDR, (fjj >> 8));
						FLASH_WriteByte(BOOT_EROR_ADDR, (fjj & 0x00ff));
						break;
					}
				}
			}
			
			FLASH_EraseSector(BOOT_STAT_ADDR);		// erase exmem page
			//app_start();
		}
		  
	}
	



#ifdef _DEBUG_WITH_LEDS_
	//*	this is for debugging it can be removed
	for (ii=0; ii<10; ii++)
	{
		PROG_PORT	&=	~(1<<PROGLED_PIN);	// turn LED off
		delay_ms(200);
		PROG_PORT	|=	(1<<PROGLED_PIN);	// turn LED on
		delay_ms(200);
	}
	PROG_PORT	&=	~(1<<PROGLED_PIN);	// turn LED off
#endif


#ifndef REMOVE_BOOTLOADER_LED
	PROG_DDR	&=	~(1<<PROGLED_PIN);	// set to default
	PROG_PORT	&=	~(1<<PROGLED_PIN);	// active low LED OFF
//	PROG_PORT	|=	(1<<PROGLED_PIN);	// active high LED OFf
	delay_ms(100);							// delay after exit
#endif


	asm volatile ("nop");			// wait until port has changed

	/*
	 * Now leave bootloader
	 */

//	UART_STATUS_REG	&=	0xfd;
	boot_rww_enable();				// enable application section


	asm volatile(
			"clr	r30		\n\t"
			"clr	r31		\n\t"
			"ijmp	\n\t"
			);

/*
* Never return to stop GCC to generate exit return code
* Actually we will never reach this point, but the compiler doesn't
* understand this
*/
	for(;;);
}


uint8_t FLASH_Init()
{
	PROG_DDR  |=  (1<<PROGCS_PIN); // chip select as output
	PROG_PORT |=  (1<<PROGCS_PIN); // pull high
	
	_delay_us(10);
	FLASH_UntilReady();
	FLASH_ResetEnable();
	FLASH_Reset();
	uint32_t temp=0;
	PROG_PORT  &=  ~(1<<PROGCS_PIN);    // low
	SPI_Transfer(0x9F);
	temp = (uint32_t)SPI_Transfer(0) << 16;
	temp |= (uint32_t)SPI_Transfer(0) << 8;
	temp |= (uint32_t)SPI_Transfer(0);
	PROG_PORT  |=  (1<<PROGCS_PIN);   // high

	if(temp == EXMEM_JEDEC)
	return 1;
	
	return 0;
}

void FLASH_WriteEnable()
{
	PROG_PORT  &=  ~(1<<PROGCS_PIN);
	SPI_Transfer(0x06);
	PROG_PORT  |=  (1<<PROGCS_PIN);
	_delay_us(10);
}

void FLASH_ResetEnable()
{
	PROG_PORT  &=  ~(1<<PROGCS_PIN);
	SPI_Transfer(0x66);
	PROG_PORT  |=  (1<<PROGCS_PIN);
	_delay_us(10);
}

void FLASH_Reset()
{
	PROG_PORT  &=  ~(1<<PROGCS_PIN);
	SPI_Transfer(0x99);
	PROG_PORT  |=  (1<<PROGCS_PIN);
	_delay_us(10);
}

void FLASH_UntilReady()
{
	uint8_t stat=0xff;
	PROG_PORT  &=  ~(1<<PROGCS_PIN);
	SPI_Transfer(0x05);
	while((stat & 0x01) == 0x01)
	{
		stat = SPI_Transfer(0);
	}
	PROG_PORT  |=  (1<<PROGCS_PIN);
	_delay_us(10);
}

void FLASH_ReadByte(uint32_t faddr, uint8_t *rdata)
{
	PROG_PORT  &=  ~(1<<PROGCS_PIN);
	SPI_Transfer(0x03);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	//  SPI_Transfer(0);
	*rdata = SPI_Transfer(0);
	PROG_PORT  |=  (1<<PROGCS_PIN);
	_delay_us(10);
}

void FLASH_ReadWord(uint32_t faddr, uint16_t *rdata)
{
	uint8_t temp1=0, temp2=0;
	PROG_PORT &= ~(1<<PROGCS_PIN);
	SPI_Transfer(0x03);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	temp1 = SPI_Transfer(0);
	temp2 = SPI_Transfer(0);
	PROG_PORT |= (1<<PROGCS_PIN);
	*rdata = ((uint16_t)temp1 << 8) | temp2;
	_delay_us(10);
}

void FLASH_WriteByte(uint32_t faddr, uint8_t wdata)
{
	FLASH_UntilReady();
	FLASH_WriteEnable();
	PROG_PORT &= ~(1<<PROGCS_PIN);
	SPI_Transfer(0x02);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	SPI_Transfer(wdata & 0xff);
	PROG_PORT |= (1<<PROGCS_PIN);
	_delay_us(10);
	FLASH_UntilReady();
}

void FLASH_EraseSector(uint32_t faddr)
{
	FLASH_UntilReady();
	FLASH_WriteEnable();
	PROG_PORT  &=  ~(1<<PROGCS_PIN);
	SPI_Transfer(0x20);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	PROG_PORT  |=  (1<<PROGCS_PIN);
	_delay_us(10);
	FLASH_UntilReady();
}

void SPI_Init(void)
{
	DDRB = (1<<PB2)|(1<<PB1)|(1<<PB0);

	SPCR = ((1<<SPE)|               // SPI Enable
	(0<<SPIE)|              // SPI Interupt Enable
	(0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
	(1<<MSTR)|              // Master/Slave select
	(0<<SPR1)|(1<<SPR0)|    // SPI Clock Rate
	(0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
	(0<<CPHA));             // Clock Phase (0:leading / 1:trailing edge sampling)
}

uint8_t SPI_Transfer(uint8_t spidata)
{
	SPDR = spidata;
	asm volatile("nop");    // small delay
	while(!(SPSR & (1<<SPIF)));   // Wait for transmission complete

	return SPDR;
}
