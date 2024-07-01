/*
 * bootSAH_9.c		==>  bootSAH_7.c
 *
 * Created: 19/09/15 1:21:06 AM
 * Author : supun
 */ 

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


#define	BLINK_LED_WHILE_WAITING

#define	_DEBUG_SERIAL_

#ifndef F_CPU
	#define F_CPU 16000000UL
#endif

#ifdef __AVR_ATmega2560__
	#define PROG_PORT		PORTB
	#define	PROG_DDR		DDRB
	#define	PROG_SPI_MISO	PB3
	#define	PROG_SPI_MOSI	PB2
	#define	PROG_SPI_SCK	PB1
	#define PROG_SPI_CS		PB0
	#define PROG_LED_PIN	PB6
#endif

#define	_BLINK_LOOP_COUNT_	(F_CPU / 2250)

#define EXMEM_JEDEC		0xEF4017
#define EXMEM_SECT_SIZE	4096		// bytes
#define EXMEM_PAGE_SIZE 256			// bytes

#define BOOT_EROR_ADDR  256
#define BOOT_STAT_ADDR  4096    // byte addr
#define BOOT_CSUM_ADDR  4352    // byte addr
#define BOOT_DATA_ADDR  8192    // byte addr

#ifndef EEWE
	#define EEWE    1
#endif
#ifndef EEMWE
	#define EEMWE   2
#endif

#ifndef BAUDRATE
	#define BAUDRATE 115200
#endif

#ifndef UART_BAUDRATE_DOUBLE_SPEED
	#if defined (__AVR_ATmega32__)
		#define UART_BAUDRATE_DOUBLE_SPEED 0
	#else
		#define UART_BAUDRATE_DOUBLE_SPEED 1
	#endif
#endif

/* 
 * ATMega with two USART, use UART0 
 * configured for atmega2560 
 */
#define	UART_BAUD_RATE_LOW			UBRR0L
#define	UART_STATUS_REG				UCSR0A
#define	UART_CONTROL_REG			UCSR0B
#define	UART_ENABLE_TRANSMITTER		TXEN0
#define	UART_ENABLE_RECEIVER		RXEN0
#define	UART_TRANSMIT_COMPLETE		TXC0
#define	UART_RECEIVE_COMPLETE		RXC0
#define	UART_DATA_REG				UDR0
#define	UART_DOUBLE_SPEED			U2X0


/*
 * Macro to calculate UBBR from XTAL and baudrate
 */
#if defined(__AVR_ATmega32__) && UART_BAUDRATE_DOUBLE_SPEED
	#define UART_BAUD_SELECT(baudRate,xtalCpu) ((xtalCpu / 4 / baudRate - 1) / 2)
#elif defined(__AVR_ATmega32__)
	#define UART_BAUD_SELECT(baudRate,xtalCpu) ((xtalCpu / 8 / baudRate - 1) / 2)
#elif UART_BAUDRATE_DOUBLE_SPEED
	#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*8.0)-1.0+0.5)
#else
	#define UART_BAUD_SELECT(baudRate,xtalCpu) (((float)(xtalCpu))/(((float)(baudRate))*16.0)-1.0+0.5)
#endif

/*
 * HW and SW version, reported to AVRISP, must match version of AVRStudio
 */
#define CONFIG_PARAM_BUILD_NUMBER_LOW	0
#define CONFIG_PARAM_BUILD_NUMBER_HIGH	0
#define CONFIG_PARAM_HW_VER				0x0F
#define CONFIG_PARAM_SW_MAJOR			2
#define CONFIG_PARAM_SW_MINOR			0x0A

/*
 * Signature bytes are not available in avr-gcc io_xxx.h
 * configured for atmega2560
 */
#ifndef SIGNATURE_BYTES
	#define SIGNATURE_BYTES 0x1E9801
#endif

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
 * States used in the receive state machine
 */
#define	ST_START		0
#define	ST_GET_SEQ_NUM	1
#define ST_MSG_SIZE_1	2
#define ST_MSG_SIZE_2	3
#define ST_GET_TOKEN	4
#define ST_GET_DATA		5
#define	ST_GET_CHECK	6
#define	ST_PROCESS		7

//char hexArr[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

/*
 * function prototypes
 */
/*
void delay_ms(unsigned int timedelay);
void tiny_delay(void);		
static uint8_t SPI_Transfer(uint8_t spidata);
void FLASH_UntilReady(void);
void FLASH_ResetEnable(void);
void FLASH_Reset(void);
void FLASH_WriteEnable(void);
uint16_t FLASH_ReadWord(uint32_t faddr);
void FLASH_EraseSector(uint32_t faddr);
void FLASH_WriteByte(uint32_t faddr, uint8_t wdata);
*/

//*******************************************************************
void delay_ms(unsigned int timedelay)
{
	unsigned int i;
	for (i=0;i<timedelay;i++)
	{
		_delay_ms(0.5);
	}
}
//** small delay
void tiny_delay(void)		
{
	asm volatile(
					"nop	\n\t"
					"nop	\n\t"
					"nop	\n\t"
					"nop	\n\t"
					"nop	\n\t"
				); 
}
//*******************************************************************
static uint8_t SPI_Transfer(uint8_t spidata)
{
	SPDR = spidata;
	asm volatile("nop");			// small delay
	while(!(SPSR & (1<<SPIF)));		// Wait for transmission complete

	return SPDR;
}

void FLASH_UntilReady(void)
{
	uint8_t stat=0xff;
	PROG_PORT  &=  ~(1<<PROG_SPI_CS);
	SPI_Transfer(0x05);
	while((stat & 0x01) == 0x01)
	{
		stat = SPI_Transfer(0);
	}
	PROG_PORT  |=   (1<<PROG_SPI_CS);
	tiny_delay();
}

void FLASH_ResetEnable(void)
{
	PROG_PORT  &=  ~(1<<PROG_SPI_CS);
	SPI_Transfer(0x66);
	PROG_PORT  |=   (1<<PROG_SPI_CS);
	tiny_delay();
}

void FLASH_Reset(void)
{
	PROG_PORT  &=  ~(1<<PROG_SPI_CS);
	SPI_Transfer(0x99);
	PROG_PORT  |=   (1<<PROG_SPI_CS);
	tiny_delay();
}

void FLASH_WriteEnable(void)
{
	PROG_PORT  &=  ~(1<<PROG_SPI_CS);
	SPI_Transfer(0x06);
	PROG_PORT  |=   (1<<PROG_SPI_CS);
	tiny_delay();
}

/*
static uint16_t FLASH_ReadWord(uint32_t faddr)
{
	uint16_t fRdTemp=0;
	FLASH_UntilReady();
	PROG_PORT  &=  ~(1<<PROG_SPI_CS);
	SPI_Transfer(0x03);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	fRdTemp = (uint16_t)SPI_Transfer(0) << 8;
	fRdTemp |= (uint16_t)SPI_Transfer(0);
	PROG_PORT  |=   (1<<PROG_SPI_CS);
	tiny_delay();
	return fRdTemp;
}
*/

void FLASH_EraseSector(uint32_t faddr)
{
	FLASH_UntilReady();
	FLASH_WriteEnable();
	PROG_PORT  &=  ~(1<<PROG_SPI_CS);
	SPI_Transfer(0x20);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	PROG_PORT  |=   (1<<PROG_SPI_CS);
	tiny_delay();
	FLASH_UntilReady();
}

void FLASH_WriteByte(uint32_t faddr, uint8_t wdata)
{
	FLASH_UntilReady();
	FLASH_WriteEnable();
	PROG_PORT  &=  ~(1<<PROG_SPI_CS);
	SPI_Transfer(0x02);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	SPI_Transfer(wdata & 0xff);
	PROG_PORT  |=   (1<<PROG_SPI_CS);
	tiny_delay();
	FLASH_UntilReady();
}

//******************************************************************
static void sendchar(char c)
{
	UART_DATA_REG	=	c;										// prepare transmission
	while (!(UART_STATUS_REG & (1 << UART_TRANSMIT_COMPLETE)));	// wait until byte sent
	UART_STATUS_REG |= (1 << UART_TRANSMIT_COMPLETE);			// delete TXCflag
}

static int	Serial_Available(void)
{
	return(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE));	// wait for data
}

/*
static unsigned char recchar(void)
{
	while (!(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE)))
	{
		// wait for data
	}
	return UART_DATA_REG;
}
*/

#define	MAX_TIME_COUNT	(F_CPU >> 1)
//*****************************************************************************
static unsigned char recchar_timeout(void)
{
uint32_t count = 0;

	while (!(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE)))
	{
		// wait for data
		count++;
		if (count > MAX_TIME_COUNT)
		{
		unsigned int	data;
		#if (FLASHEND > 0x10000)
			data	=	pgm_read_word_far(0);	//*	get the first word of the user program
		#else
			data	=	pgm_read_word_near(0);	//*	get the first word of the user program
		#endif
			if (data != 0xffff)					//*	make sure its valid before jumping to it.
			{
				asm volatile(
						"clr	r30		\n\t"
						"clr	r31		\n\t"
						"ijmp	\n\t"
						);
			}
			count	=	0;
		}
	}
	return UART_DATA_REG;
}

/*
static void printByte(uint8_t numData)
{
	char chData1, chData2;
	
	chData1 = hexArr[(numData >> 4)];
	sendchar(chData1);
	chData2 = hexArr[(numData & 0xF)];
	sendchar(chData2);
}
*/

/*
 * since this bootloader is not linked against the avr-gcc crt1 functions,
 * to reduce the code size, we need to provide our own initialization
 */
void __jumpMain	(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
#include <avr/sfr_defs.h>

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

//*	for watch dog timer startup
void (*app_start)(void) = 0x0000;

int main(void)
{
	uint32_t exfMemStat   = 0;
	
	uint32_t exfBStatAddr = 0;
	uint16_t exfBootStat  = 0;
	
	uint32_t exfDLenAddr  = 0;
	uint16_t exfDataLen   = 0;
	
	uint32_t exfDataAddr  = 0;
	uint8_t  serialData	  = 0;			// c
	//uint8_t  fRdTries   = 0;
	
	uint32_t exfCsumAddr  = 0;
	uint16_t exfChSumCal  = 0;
	uint16_t exfChSumRev  = 0;
	
	//address_t address	= 0;
	//address_t fPageAddr = 0;
	
	uint32_t fTmpPgAddr	= 0;	// address
	uint32_t fPageAddr  = 0;	// eraseAddress
	
	
	uint8_t  fLSByte, fMSByte;
	uint16_t fDataWord;
	//uint16_t fExMemSize, fInMemSize;
	//uint16_t fExMemInc, fInMemInc;
	uint16_t fMemSize;
	uint16_t fIndex;		// ii
	
	uint8_t  fMsgBuff[285];
	
	uint8_t	 msgParseState;
	uint8_t	 isLeave	= 0;
	uint8_t	 checksum	= 0;
	uint8_t	 seqNum		= 0;
	uint8_t	 *p;
	uint16_t msgLength	= 0;	
	
	uint32_t boot_timeout;
	uint32_t boot_timer;
	uint8_t  boot_state;	

//************************************************************************
	
	//*	some chips dont set the stack properly
	asm volatile ( ".set __stack, %0" :: "i" (RAMEND) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND >> 8) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_HI_ADDR) );
	asm volatile ( "ldi	16, %0" :: "i" (RAMEND & 0x0ff) );
	asm volatile ( "out %0,16" :: "i" (AVR_STACK_POINTER_LO_ADDR) );
	
//************************************************************************

//*	handle the watch dog timer
	uint8_t	mcuStatusReg;
	mcuStatusReg	=	MCUSR;

	__asm__ __volatile__ ("cli");
	__asm__ __volatile__ ("wdr");
	
	//MCUSR	=	0;
	MCUSR &= ~(1<<WDRF);
	
	WDTCSR	|=	_BV(WDCE) | _BV(WDE);
	WDTCSR	=	0;
	__asm__ __volatile__ ("sei");
	// check if WDT generated the reset, if so, go straight to app
	if (mcuStatusReg & _BV(WDRF))
	{
		app_start();
	}
	
//************************************************************************

// Initialize SPI
	PROG_DDR = ((0<<PROG_SPI_MISO)|
				(1<<PROG_SPI_MOSI)|
				(1<<PROG_SPI_SCK) |
				(1<<PROG_SPI_CS)
			   );

	SPCR = (	(1<<SPE) |              // SPI Enable
				(0<<SPIE)|              // SPI Interrupt Enable
				(0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
				(1<<MSTR)|              // Master/Slave select
				(0<<SPR1)|(1<<SPR0)|    // SPI Clock Rate
				(0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
				(0<<CPHA)               // Clock Phase (0:leading / 1:trailing edge sampling)
			);
			
	PROG_PORT |=  (1<<PROG_SPI_CS); 			// pull high
	
	asm volatile ("nop");				// wait until port has changed
	
//************************************************************************

	exfBStatAddr = BOOT_STAT_ADDR;
	exfDLenAddr  = BOOT_STAT_ADDR + 2;
	exfDataAddr  = BOOT_DATA_ADDR;
	exfCsumAddr  = BOOT_CSUM_ADDR;
	fPageAddr	 = 0;
	
//************************************************************************	
// Initialize ExFlash
	FLASH_UntilReady();
	FLASH_ResetEnable();
	FLASH_Reset();
	
	PROG_PORT  &=  ~(1<<PROG_SPI_CS);
	SPI_Transfer(0x9F);
	exfMemStat = (uint32_t)SPI_Transfer(0) << 16;
	exfMemStat |= (uint32_t)SPI_Transfer(0) << 8;
	exfMemStat |= (uint32_t)SPI_Transfer(0);
	PROG_PORT |=  (1<<PROG_SPI_CS);

	if(exfMemStat == EXMEM_JEDEC)
	{
		PROG_PORT  &=  ~(1<<PROG_SPI_CS);
		SPI_Transfer(0x03);
		SPI_Transfer((exfBStatAddr >> 16) & 0xff);
		SPI_Transfer((exfBStatAddr >> 8) & 0xff);
		SPI_Transfer(exfBStatAddr & 0xff);
		exfBootStat = (uint16_t)SPI_Transfer(0) << 8;
		exfBootStat |= (uint16_t)SPI_Transfer(0);
		PROG_PORT  |=   (1<<PROG_SPI_CS);
		tiny_delay();
		
		/*exfBootStat = FLASH_ReadWord(exfBStatAddr);*/

		if(exfBootStat == 0x2323)
		{			
			PROG_PORT  &=  ~(1<<PROG_SPI_CS);
			SPI_Transfer(0x03);
			SPI_Transfer((exfDLenAddr >> 16) & 0xff);
			SPI_Transfer((exfDLenAddr >> 8) & 0xff);
			SPI_Transfer(exfDLenAddr & 0xff);
			exfDataLen = (uint16_t)SPI_Transfer(0) << 8;
			exfDataLen |= (uint16_t)SPI_Transfer(0);
			PROG_PORT  |=   (1<<PROG_SPI_CS);
			tiny_delay();
			
			/*exfDataLen = FLASH_ReadWord(exfDLenAddr);*/

			do 
			{				
				fMemSize = EXMEM_PAGE_SIZE;
				fIndex = 0;
				//fRdTries  = 0;
				exfChSumCal = 0;
				PROG_PORT  &=  ~(1<<PROG_SPI_CS);
				SPI_Transfer(0x03);
				SPI_Transfer((exfDataAddr >> 16) & 0xff);
				SPI_Transfer((exfDataAddr >> 8) & 0xff);
				SPI_Transfer(exfDataAddr & 0xff);			
				do 
				{
					serialData = SPI_Transfer(0);
					fMsgBuff[fIndex++] = serialData;
					exfChSumCal += serialData;				
					fMemSize--;
				} while (fMemSize);		
				PROG_PORT  |=   (1<<PROG_SPI_CS);
				tiny_delay();

		
				PROG_PORT  &=  ~(1<<PROG_SPI_CS);
				SPI_Transfer(0x03);
				SPI_Transfer((exfCsumAddr >> 16) & 0xff);
				SPI_Transfer((exfCsumAddr >> 8) & 0xff);
				SPI_Transfer(exfCsumAddr & 0xff);				
				exfChSumRev = (uint16_t)SPI_Transfer(0) << 8;
				exfChSumRev |= (uint16_t)SPI_Transfer(0);			
				PROG_PORT  |=   (1<<PROG_SPI_CS);
				tiny_delay();
				
		
				if(exfChSumRev != exfChSumCal)
				{

					break;
				}
				
 				fTmpPgAddr  = fPageAddr;
 				fMemSize = SPM_PAGESIZE;
 				fIndex  = 0;
				 
				if (fPageAddr >= APP_END )
				{
					break;
				}
					
				boot_page_erase_safe(fPageAddr);
				boot_spm_busy_wait();			
					
				do 
				{
					fLSByte = fMsgBuff[fIndex];
					fIndex++;
					fMSByte = fMsgBuff[fIndex];
					fIndex++;

					fDataWord = (fMSByte << 8) | fLSByte;
						
					boot_page_fill_safe(fTmpPgAddr,fDataWord);
										
					fTmpPgAddr  = fTmpPgAddr + 2;
					asm volatile ("nop");	
					fMemSize = fMemSize - 2;
				} while (fMemSize);
					
				boot_page_write_safe(fPageAddr);
				boot_spm_busy_wait();
				boot_rww_enable_safe();
					
				fPageAddr += SPM_PAGESIZE;
				exfDataAddr += EXMEM_PAGE_SIZE;
				exfCsumAddr += 2;
				asm volatile ("nop");
				
				exfDataLen--;
			} while (exfDataLen);
			
			tiny_delay();
			FLASH_EraseSector(exfBStatAddr);	
			asm volatile ("nop");
			FLASH_WriteByte(512, 0x4F);
			FLASH_WriteByte(513, 0x4B);					
		}
	}
	
//**************************************************************************	
	fIndex		 = 0;
	boot_timer	 = 0;
	boot_state	 = 0;
	//boot_timeout = 20000;
	
#ifdef BLINK_LED_WHILE_WAITING
	boot_timeout	=	 20000;		//*	should be about 1 second
#else
	boot_timeout	=	3500000; // 7 seconds , approx 2us per step when optimize "s"
#endif

#ifndef REMOVE_BOOTLOADER_LED
/* PROG_PIN pulled low, indicate with LED that bootloader is active */
	PROG_DDR	|=	(1<<PROG_LED_PIN);
	//	PROGLED_PORT	&=	~(1<<PROGLED_PIN);	// active low LED ON
	PROG_PORT	|=	(1<<PROG_LED_PIN);	// active high LED ON
	delay_ms(100);
#endif
	
/*
* Initialize UART
* set baudrate and enable USART receiver and transmitter without interrupts
*/
#if UART_BAUDRATE_DOUBLE_SPEED
	UART_STATUS_REG		|=	(1 <<UART_DOUBLE_SPEED);
#endif
	UART_BAUD_RATE_LOW	=	UART_BAUD_SELECT(BAUDRATE,F_CPU);
	UART_CONTROL_REG	=	(1 << UART_ENABLE_RECEIVER) | (1 << UART_ENABLE_TRANSMITTER);

	asm volatile ("nop");			// wait until port has changed
	

	sendchar('u');
	sendchar('t');
	sendchar('e');
	sendchar('c');
	sendchar('h');
	sendchar('_');
	sendchar('B');
	sendchar('B');

	sendchar(0x0d);
	sendchar(0x0a);
	
	
	while (boot_state==0)
	{
		while ((!(Serial_Available())) && (boot_state == 0))		// wait for data
		{
			_delay_ms(0.001);
			boot_timer++;
			if (boot_timer > boot_timeout)
			{
				boot_state	=	1; // (after ++ -> boot_state=2 bootloader timeout, jump to main 0x00000 )
			}
		#ifdef BLINK_LED_WHILE_WAITING
			if ((boot_timer % _BLINK_LOOP_COUNT_) == 0)
			{
				//*	toggle the LED
				PROG_PORT	^=	(1<<PROG_LED_PIN);	// turn LED ON
			}
		#endif
		}
		boot_state++; // ( if boot_state=1 bootloader received byte from UART, enter bootloader mode)
	}
	
	//****************************************************************************************************************
	
	if (boot_state==1)
	{
		//*	main loop
		while (!isLeave)
		{
			/*
			 * Collect received bytes to a complete message
			 */
			msgParseState	=	ST_START;
			while ( msgParseState != ST_PROCESS )
			{
				if (boot_state==1)
				{
					boot_state	=	0;
					serialData	=	UART_DATA_REG;
				}
				else
				{
				//	serialData	=	recchar();
					serialData	=	recchar_timeout();
					
				}		
				
			//***************************************************************************************
		
				switch (msgParseState)
				{
					case ST_START:
						if ( serialData == MESSAGE_START )
						{
							msgParseState	=	ST_GET_SEQ_NUM;
							checksum		=	MESSAGE_START^0;
						}
						break;

					case ST_GET_SEQ_NUM:
//					#ifdef _FIX_ISSUE_505_
						seqNum			=	serialData;
						msgParseState	=	ST_MSG_SIZE_1;
						checksum		^=	serialData;
// 					#else
// 						if ( (serialData == 1) || (serialData == seqNum) )
// 						{
// 							seqNum			=	serialData;
// 							msgParseState	=	ST_MSG_SIZE_1;
// 							checksum		^=	serialData;
// 						}
// 						else
// 						{
// 							msgParseState	=	ST_START;
// 						}
// 					#endif
						break;

					case ST_MSG_SIZE_1:
						msgLength		=	serialData<<8;
						msgParseState	=	ST_MSG_SIZE_2;
						checksum		^=	serialData;
						break;

					case ST_MSG_SIZE_2:
						msgLength		|=	serialData;
						msgParseState	=	ST_GET_TOKEN;
						checksum		^=	serialData;
						break;

					case ST_GET_TOKEN:
						if ( serialData == TOKEN )
						{
							msgParseState	=	ST_GET_DATA;
							checksum		^=	serialData;
							fIndex			=	0;
						}
						else
						{
							msgParseState	=	ST_START;
						}
						break;

					case ST_GET_DATA:
						fMsgBuff[fIndex++]	=	serialData;
						checksum			^=	serialData;
						if (fIndex == msgLength )
						{
							msgParseState	=	ST_GET_CHECK;
						}
						break;

					case ST_GET_CHECK:
						if ( serialData == checksum )
						{
							msgParseState	=	ST_PROCESS;
						}
						else
						{
							msgParseState	=	ST_START;
						}
						break;
				}	//	switch
			}	//	while(msgParseState)
			
		//*******************************************************************************************
			/*
			 * Now process the STK500 commands, see Atmel Appnote AVR068
			 */

			switch (fMsgBuff[0])
			{
/*
	#ifndef REMOVE_CMD_SPI_MULTI
				case CMD_SPI_MULTI:
					{
						unsigned char answerByte;
						unsigned char flag=0;

						if ( msgBuffer[4]== 0x30 )
						{
							unsigned char signatureIndex	=	msgBuffer[6];

							if ( signatureIndex == 0 )
							{
								answerByte	=	(SIGNATURE_BYTES >> 16) & 0x000000FF;
							}
							else if ( signatureIndex == 1 )
							{
								answerByte	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
							}
							else
							{
								answerByte	=	SIGNATURE_BYTES & 0x000000FF;
							}
						}
						else if ( msgBuffer[4] & 0x50 )
						{
						//	Issue 544: 	stk500v2 bootloader doesn't support reading fuses
						//	I cant find the docs that say what these are supposed to be but this was figured out by trial and error
						//	answerByte	=	boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
						//	answerByte	=	boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
						//	answerByte	=	boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS);
							if (msgBuffer[4] == 0x50)
							{
								answerByte	=	boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS);
							}
							else if (msgBuffer[4] == 0x58)
							{
								answerByte	=	boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS);
							}
							else
							{
								answerByte	=	0;
							}
						}
						else
						{
							answerByte	=	0; // for all others command are not implemented, return dummy value for AVRDUDE happy <Worapoht>
						}
						if ( !flag )
						{
							msgLength		=	7;
							msgBuffer[1]	=	STATUS_CMD_OK;
							msgBuffer[2]	=	0;
							msgBuffer[3]	=	msgBuffer[4];
							msgBuffer[4]	=	0;
							msgBuffer[5]	=	answerByte;
							msgBuffer[6]	=	STATUS_CMD_OK;
						}
					}
					break;
	#endif
*/
				case CMD_SIGN_ON:
					msgLength		=	11;
					fMsgBuff[1] 	=	STATUS_CMD_OK;
					fMsgBuff[2] 	=	8;
					fMsgBuff[3] 	=	'A';
					fMsgBuff[4] 	=	'V';
					fMsgBuff[5] 	=	'R';
					fMsgBuff[6] 	=	'I';
					fMsgBuff[7] 	=	'S';
					fMsgBuff[8] 	=	'P';
					fMsgBuff[9] 	=	'_';
					fMsgBuff[10]	=	'2';
					break;

				case CMD_GET_PARAMETER:
					{
						unsigned char value;

						switch(fMsgBuff[1])
						{
						case PARAM_BUILD_NUMBER_LOW:
							value	=	CONFIG_PARAM_BUILD_NUMBER_LOW;
							break;
						case PARAM_BUILD_NUMBER_HIGH:
							value	=	CONFIG_PARAM_BUILD_NUMBER_HIGH;
							break;
						case PARAM_HW_VER:
							value	=	CONFIG_PARAM_HW_VER;
							break;
						case PARAM_SW_MAJOR:
							value	=	CONFIG_PARAM_SW_MAJOR;
							break;
						case PARAM_SW_MINOR:
							value	=	CONFIG_PARAM_SW_MINOR;
							break;
						default:
							value	=	0;
							break;
						}
						msgLength	=	3;
						fMsgBuff[1]	=	STATUS_CMD_OK;
						fMsgBuff[2]	=	value;
					}
					break;

				case CMD_LEAVE_PROGMODE_ISP:
					isLeave	=	1;
					//*	fall thru

				case CMD_SET_PARAMETER:
				case CMD_ENTER_PROGMODE_ISP:
					msgLength	=	2;
					fMsgBuff[1]	=	STATUS_CMD_OK;
					break;

				case CMD_READ_SIGNATURE_ISP:
					{
						unsigned char signatureIndex	=	fMsgBuff[4];
						unsigned char signature;

						if ( signatureIndex == 0 )
							signature	=	(SIGNATURE_BYTES >>16) & 0x000000FF;
						else if ( signatureIndex == 1 )
							signature	=	(SIGNATURE_BYTES >> 8) & 0x000000FF;
						else
							signature	=	SIGNATURE_BYTES & 0x000000FF;

						msgLength	=	4;
						fMsgBuff[1]	=	STATUS_CMD_OK;
						fMsgBuff[2]	=	signature;
						fMsgBuff[3]	=	STATUS_CMD_OK;
					}
					break;

				case CMD_READ_LOCK_ISP:
					msgLength		=	4;
					fMsgBuff[1]	=	STATUS_CMD_OK;
					fMsgBuff[2]	=	boot_lock_fuse_bits_get( GET_LOCK_BITS );
					fMsgBuff[3]	=	STATUS_CMD_OK;
					break;

				case CMD_READ_FUSE_ISP:
					{
						unsigned char fuseBits;

						if ( fMsgBuff[2] == 0x50 )
						{
							if ( fMsgBuff[3] == 0x08 )
								fuseBits	=	boot_lock_fuse_bits_get( GET_EXTENDED_FUSE_BITS );
							else
								fuseBits	=	boot_lock_fuse_bits_get( GET_LOW_FUSE_BITS );
						}
						else
						{
							fuseBits	=	boot_lock_fuse_bits_get( GET_HIGH_FUSE_BITS );
						}
						msgLength		=	4;
						fMsgBuff[1]	=	STATUS_CMD_OK;
						fMsgBuff[2]	=	fuseBits;
						fMsgBuff[3]	=	STATUS_CMD_OK;
					}
					break;

	#ifndef REMOVE_PROGRAM_LOCK_BIT_SUPPORT
				case CMD_PROGRAM_LOCK_ISP:
					{
						unsigned char lockBits	=	fMsgBuff[4];

						lockBits	=	(~lockBits) & 0x3C;	// mask BLBxx bits
						boot_lock_bits_set(lockBits);		// and program it
						boot_spm_busy_wait();

						msgLength		=	3;
						fMsgBuff[1]	=	STATUS_CMD_OK;
						fMsgBuff[2]	=	STATUS_CMD_OK;
					}
					break;
	#endif
				case CMD_CHIP_ERASE_ISP:
					fPageAddr		=	0;
					msgLength		=	2;
				//	fMsgBuff[1]	=	STATUS_CMD_OK;
					fMsgBuff[1]	=	STATUS_CMD_FAILED;	//*	isue 543, return FAILED instead of OK
					break;

				case CMD_LOAD_ADDRESS:
	//#if defined(RAMPZ)
					fTmpPgAddr	=	( ((uint32_t)(fMsgBuff[1])<<24)|((uint32_t)(fMsgBuff[2])<<16)|((uint32_t)(fMsgBuff[3])<<8)|(fMsgBuff[4]) )<<1;
	//#else
				//	address	=	( ((msgBuffer[3])<<8)|(msgBuffer[4]) )<<1;		//convert word to byte address
	//#endif
			
					msgLength		=	2;
					fMsgBuff[1]	=	STATUS_CMD_OK;
					break;

				case CMD_PROGRAM_FLASH_ISP:
				case CMD_PROGRAM_EEPROM_ISP:
					{
						unsigned int	size	=	((fMsgBuff[1])<<8) | fMsgBuff[2];
						unsigned char	*p	=	fMsgBuff+10;
						unsigned int	data;
						unsigned char	highByte, lowByte;
						uint32_t		tempaddress	=	fTmpPgAddr;


						if ( fMsgBuff[0] == CMD_PROGRAM_FLASH_ISP )
						{
							// erase only main section (bootloader protection)
							if (fPageAddr < APP_END )
							{
								boot_page_erase_safe(fPageAddr);	// Perform page erase
								boot_spm_busy_wait();		// Wait until the memory is erased.
								fPageAddr += SPM_PAGESIZE;	// point to next page to be erase
							}

							/* Write FLASH */
							do {
								lowByte		=	*p++;
								highByte 	=	*p++;

								data		=	(highByte << 8) | lowByte;
								boot_page_fill_safe(fTmpPgAddr,data);

								fTmpPgAddr	=	fTmpPgAddr + 2;	// Select next word in memory
								size	-=	2;				// Reduce number of bytes to write by two
							} while (size);					// Loop until all bytes written

							boot_page_write_safe(tempaddress);
							boot_spm_busy_wait();
							boot_rww_enable_safe();				// Re-enable the RWW section
						}
						else
						{
							//*	issue 543, this should work, It has not been tested.
							uint16_t ii = fTmpPgAddr >> 1;
							/* write EEPROM */
							while (size) {
								eeprom_write_byte((uint8_t*)ii, *p++);
								fTmpPgAddr+=2;						// Select next EEPROM byte
								ii++;
								size--;
							}
						}
						msgLength		=	2;
						fMsgBuff[1]	=	STATUS_CMD_OK;
					}
					break;

				case CMD_READ_FLASH_ISP:
				case CMD_READ_EEPROM_ISP:
					{
						unsigned int	size	=	((fMsgBuff[1])<<8) | fMsgBuff[2];
						unsigned char	*p		=	fMsgBuff+1;
						msgLength				=	size+3;

						*p++	=	STATUS_CMD_OK;
						if (fMsgBuff[0] == CMD_READ_FLASH_ISP )
						{
							unsigned int data;

							// Read FLASH
							do {
						//#if defined(RAMPZ)
						#if (FLASHEND > 0x10000)
								data	=	pgm_read_word_far(fTmpPgAddr);
						#else
								data	=	pgm_read_word_near(fTmpPgAddr);
						#endif
								*p++	=	(unsigned char)data;		//LSB
								*p++	=	(unsigned char)(data >> 8);	//MSB
								fTmpPgAddr	+=	2;							// Select next word in memory
								size	-=	2;
							}while (size);
						}
						else
						{
							/* Read EEPROM */
							do {
								EEARL	=	fTmpPgAddr;			// Setup EEPROM address
								EEARH	=	((fTmpPgAddr >> 8));
								fTmpPgAddr++;					// Select next EEPROM byte
								EECR	|=	(1<<EERE);			// Read EEPROM
								*p++	=	EEDR;				// Send EEPROM data
								size--;
							} while (size);
						}
						*p++	=	STATUS_CMD_OK;
					}
					break;

				default:
					msgLength		=	2;
					fMsgBuff[1]	=	STATUS_CMD_FAILED;
					break;
			}

			/*
			 * Now send answer message back
			 */
			sendchar(MESSAGE_START);
			checksum	=	MESSAGE_START^0;

			sendchar(seqNum);
			checksum	^=	seqNum;

			serialData	=	((msgLength>>8) & 0xFF);
			sendchar(serialData);
			checksum	^=	serialData;

			serialData	=	msgLength & 0x00FF;
			sendchar(serialData);
			checksum ^= serialData;

			sendchar(TOKEN);
			checksum ^= TOKEN;

			p	=	fMsgBuff;
			while ( msgLength )
			{
				serialData	=	*p++;
				sendchar(serialData);
				checksum ^= serialData;
				msgLength--;
			}
			sendchar(checksum);
			seqNum++;
	
		#ifndef REMOVE_BOOTLOADER_LED
			//*	<MLS>	toggle the LED
			PROG_PORT	^=	(1<<PROG_LED_PIN);	// active high LED ON
		#endif

		}
	}
	
	
	
#ifndef REMOVE_BOOTLOADER_LED
	PROG_DDR	&=	~(1<<PROG_LED_PIN);		// set to default
	PROG_PORT	&=	~(1<<PROG_LED_PIN);		// active low LED OFF
//	PROG_PORT	|=	(1<<PROG_LED_PIN);		// active high LED OFf
	delay_ms(100);							// delay after exit
#endif	
	
//*************************************************************************
	tiny_delay();
	UART_STATUS_REG	&=	0xfd;
	boot_rww_enable_safe();				// enable application section
	// leaving bootloader  # bye_bye # sayonara # suba_gaman #
	asm volatile(
					"clr	r30		\n\t"
					"clr	r31		\n\t"
					"ijmp			\n\t"
				);
	
    for(;;);	
}

//*****************************************************************************
