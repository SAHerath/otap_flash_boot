/*
 * bootSAH_7.c
 *
 * Created: 19/09/16 9:29:06 AM
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


#ifdef __AVR_ATmega2560__
	#define PROG_PORT		PORTB
	#define	PROG_DDR		DDRB
	#define	PROG_SPI_MISO	PB3
	#define	PROG_SPI_MOSI	PB2
	#define	PROG_SPI_SCK	PB1
	#define PROG_SPI_CS		PB0
	#define PROG_LED_PIN	PB6
#endif

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

#define	_DEBUG_SERIAL_

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

static unsigned char recchar(void)
{
	while (!(UART_STATUS_REG & (1 << UART_RECEIVE_COMPLETE)))
	{
		// wait for data
	}
	return UART_DATA_REG;
}

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
	uint32_t exfMemStat = 0;
	
	uint32_t exfBStatAddr = 0;
	uint16_t exfBootStat  = 0;
	
	uint32_t exfDLenAddr  = 0;
	uint16_t exfDataLen   = 0;
	
	uint32_t exfDataAddr  = 0;
	uint8_t  serialData		  = 0;			// c
	//uint8_t  fRdTries   = 0;
	
	uint32_t exfCsumAddr  = 0;
	uint16_t exfChSumCal  = 0;
	uint16_t exfChSumRev  = 0;
	
	//address_t address	= 0;
	//address_t fPageAddr = 0;
	
	uint32_t fTmpPgAddr	= 0;	// address
	uint32_t fPageAddr  = 0;	// eraseAddress
	
	//unsigned char	msgBuffer[285];
	uint8_t fMsgBuff[285];		// msgBuffer[285];
	//uint8_t	*p;
	
	uint8_t  fLSByte, fMSByte;
	uint16_t fDataWord;
	//uint16_t fExMemSize, fInMemSize;
	//uint16_t fExMemInc, fInMemInc;
	uint16_t fMemSize;
	uint16_t fIndex;		// ii
		
	uint32_t boot_timeout;
	uint32_t boot_timer;
	uint8_t boot_state;	

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

//* Initialize SPI
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
//* Initialize ExFlash
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
	
	boot_timer	 =	0;
	boot_state	 =	0;
	boot_timeout =	3500000;
	
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
	
    for(;;)
    {		
		tiny_delay();
    }
}

//*****************************************************************************
