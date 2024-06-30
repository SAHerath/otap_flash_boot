/*
 * bootSAH_5.c
 *
 * Created: 19/09/11 07:50:32 PM
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
	#define SPI_PORT	PORTB
	#define	SPI_DDR		DDRB
	#define	SPI_MISO	PB3
	#define	SPI_MOSI	PB2
	#define	SPI_SCK		PB1
	#define SPI_CS		PB0
#endif

#define EXMEM_JEDEC		0xEF4017
#define EXMEM_SECT_SIZE	4096		// bytes
#define EXMEM_PAGE_SIZE 256			// bytes

#define BOOT_EROR_ADDR  256
#define BOOT_STAT_ADDR  4096    // byte addr
#define BOOT_CSUM_ADDR  4352    // byte addr
#define BOOT_DATA_ADDR  8192    // byte addr

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

//char hexArr[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

/*
 * function prototypes
 */
//void sendchar(char c);
//void printByte(uint8_t numData);
//static int	Serial_Available(void);
//static unsigned char recchar(void);
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

/*
static void sendchar(char c)
{
	UART_DATA_REG	=	c;										// prepare transmission
	while (!(UART_STATUS_REG & (1 << UART_TRANSMIT_COMPLETE)));	// wait until byte sent
	UART_STATUS_REG |= (1 << UART_TRANSMIT_COMPLETE);			// delete TXCflag
}

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
	uint32_t fMemStatus = 0;
	
	uint32_t fBStatAddr = 0;
	uint16_t fBootStat  = 0;
	
	uint32_t fDLenAddr  = 0;
	uint16_t fDataLen   = 0;
	
	uint32_t fDataAddr  = 0;
	uint8_t  fRdData    = 0;
	//uint8_t  fRdTries   = 0;
	
	uint32_t fCsumAddr  = 0;
	uint16_t fChSumCal  = 0;
	uint16_t fChSumRev  = 0;
	
	//address_t address	= 0;
	//address_t fPageAddr = 0;
	
	uint32_t ftempAddr	= 0;
	uint32_t fPageAddr  = 0;
	
	//unsigned char	msgBuffer[285];
	uint8_t flashBuff[256];
	//uint8_t	*p;
	
	uint8_t  fLSByte, fMSByte;
	uint16_t fDataWord;
	uint16_t fExMemSize, fInMemSize;
	uint16_t fExMemInc, fInMemInc;
		
		

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
	WDTCSR	|=	_BV(WDCE) | _BV(WDE);
	WDTCSR	=	0;
	__asm__ __volatile__ ("sei");
	// check if WDT generated the reset, if so, go straight to app
	if (mcuStatusReg & _BV(WDRF))
	{
		app_start();
	}
	
//************************************************************************

//* Init UART
//*set baudrate and enable USART receiver and transmiter without interrupts

// #if UART_BAUDRATE_DOUBLE_SPEED
// 	UART_STATUS_REG		|=	(1 <<UART_DOUBLE_SPEED);
// #endif
// 	UART_BAUD_RATE_LOW	=	UART_BAUD_SELECT(BAUDRATE,F_CPU);
// 	UART_CONTROL_REG	=	(1 << UART_ENABLE_RECEIVER) | (1 << UART_ENABLE_TRANSMITTER);
// 
// 	asm volatile ("nop");			// wait until port has changed

//************************************************************************

//* Init SPI
	SPI_DDR = (	(0<<SPI_MISO)|
				(1<<SPI_MOSI)|
				(1<<SPI_SCK) |
				(1<<SPI_CS)
			  );

	SPCR = (	(1<<SPE) |              // SPI Enable
				(0<<SPIE)|              // SPI Interupt Enable
				(0<<DORD)|              // Data Order (0:MSB first / 1:LSB first)
				(1<<MSTR)|              // Master/Slave select
				(0<<SPR1)|(1<<SPR0)|    // SPI Clock Rate
				(0<<CPOL)|              // Clock Polarity (0:SCK low / 1:SCK hi when idle)
				(0<<CPHA)               // Clock Phase (0:leading / 1:trailing edge sampling)
			);
			
	SPI_PORT |=  (1<<SPI_CS); 			// pull high
	
	asm volatile ("nop");				// wait until port has changed
	
//************************************************************************

/*	
#ifdef _DEBUG_SERIAL_

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

	delay_ms(100);
#endif
*/

	fBStatAddr = 4096;
	fDLenAddr  = 4098;
	fDataAddr  = 8192;
	fCsumAddr  = 4352;
	fPageAddr  = 0;
	
//************************************************************************	
//* Init ExFlash
	FLASH_UntilReady();
	FLASH_ResetEnable();
	FLASH_Reset();
	
	SPI_PORT  &=  ~(1<<SPI_CS);
	SPI_Transfer(0x9F);
	fMemStatus = (uint32_t)SPI_Transfer(0) << 16;
	fMemStatus |= (uint32_t)SPI_Transfer(0) << 8;
	fMemStatus |= (uint32_t)SPI_Transfer(0);
	SPI_PORT |=  (1<<SPI_CS);

	if(fMemStatus == EXMEM_JEDEC)
	{
		SPI_PORT  &=  ~(1<<SPI_CS);
		SPI_Transfer(0x03);
		SPI_Transfer((fBStatAddr >> 16) & 0xff);
		SPI_Transfer((fBStatAddr >> 8) & 0xff);
		SPI_Transfer(fBStatAddr & 0xff);
		fBootStat = (uint16_t)SPI_Transfer(0) << 8;
		fBootStat |= (uint16_t)SPI_Transfer(0);
		SPI_PORT  |=   (1<<SPI_CS);
		tiny_delay();


		if(fBootStat == 0x2323)
		{
			
			SPI_PORT  &=  ~(1<<SPI_CS);
			SPI_Transfer(0x03);
			SPI_Transfer((fDLenAddr >> 16) & 0xff);
			SPI_Transfer((fDLenAddr >> 8) & 0xff);
			SPI_Transfer(fDLenAddr & 0xff);
			fDataLen = (uint16_t)SPI_Transfer(0) << 8;
			fDataLen |= (uint16_t)SPI_Transfer(0);
			SPI_PORT  |=   (1<<SPI_CS);
			tiny_delay();

			do 
			{
				
				fExMemSize = EXMEM_PAGE_SIZE;
				fExMemInc = 0;
				//fRdTries  = 0;
				fChSumCal = 0;
				SPI_PORT  &=  ~(1<<SPI_CS);
				SPI_Transfer(0x03);
				SPI_Transfer((fDataAddr >> 16) & 0xff);
				SPI_Transfer((fDataAddr >> 8) & 0xff);
				SPI_Transfer(fDataAddr & 0xff);			
				do 
				{
					fRdData = SPI_Transfer(0);
					flashBuff[fExMemInc++] = fRdData;
					fChSumCal += fRdData;				
					fExMemSize--;
				} while (fExMemSize);		
				SPI_PORT  |=   (1<<SPI_CS);
				tiny_delay();

		
				SPI_PORT  &=  ~(1<<SPI_CS);
				SPI_Transfer(0x03);
				SPI_Transfer((fCsumAddr >> 16) & 0xff);
				SPI_Transfer((fCsumAddr >> 8) & 0xff);
				SPI_Transfer(fCsumAddr & 0xff);				
				fChSumRev = (uint16_t)SPI_Transfer(0) << 8;
				fChSumRev |= (uint16_t)SPI_Transfer(0);			
				SPI_PORT  |=   (1<<SPI_CS);
				tiny_delay();
		
				if(fChSumRev != fChSumCal)
				{

					break;
				}
				
 				ftempAddr  = fPageAddr;
 				fInMemSize = SPM_PAGESIZE;
 				fInMemInc  = 0;
				 
				if (fPageAddr >= APP_END )
				{
					break;
				}
					
				boot_page_erase_safe(fPageAddr);
				boot_spm_busy_wait();			
					
				do 
				{
					fLSByte = flashBuff[fInMemInc];
					fInMemInc++;
					fMSByte = flashBuff[fInMemInc];
					fInMemInc++;

					fDataWord = (fMSByte << 8) | fLSByte;
						
					boot_page_fill_safe(ftempAddr,fDataWord);
					
						
					ftempAddr  = ftempAddr + 2;
					asm volatile ("nop");	
					fInMemSize = fInMemSize - 2;
				} while (fInMemSize);
					
				boot_page_write_safe(fPageAddr);
				boot_spm_busy_wait();
				boot_rww_enable_safe();
					
				fPageAddr += SPM_PAGESIZE;
				fDataAddr += EXMEM_PAGE_SIZE;
				fCsumAddr += 2;
				asm volatile ("nop");
				
				fDataLen--;
			} while (fDataLen);
			
			tiny_delay();
			FLASH_EraseSector(fBStatAddr);						
		}
	}
	
	tiny_delay();
	
	boot_rww_enable_safe();				// enable application section

	// leaving bootloader  # bye bye # sayonara #
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
void delay_ms(unsigned int timedelay)
{
	unsigned int i;
	for (i=0;i<timedelay;i++)
	{
		_delay_ms(0.5);
	}
}
//** small delay
void tiny_delay()		
{
	asm volatile(
					"nop	\n\t"
					"nop	\n\t"
					"nop	\n\t"
					"nop	\n\t"
					"nop	\n\t"
				); 
}
   

//*****************************************************************************



//************************************************************************

/*
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
*/
static uint8_t SPI_Transfer(uint8_t spidata)
{
	SPDR = spidata;
	asm volatile("nop");			// small delay
	while(!(SPSR & (1<<SPIF)));		// Wait for transmission complete

	return SPDR;
}

void FLASH_UntilReady()
{
	uint8_t stat=0xff;
	SPI_PORT  &=  ~(1<<SPI_CS);
	SPI_Transfer(0x05);
	while((stat & 0x01) == 0x01)
	{
		stat = SPI_Transfer(0);
	}
	SPI_PORT  |=   (1<<SPI_CS);
	tiny_delay();
}

void FLASH_ResetEnable()
{
	SPI_PORT  &=  ~(1<<SPI_CS);
	SPI_Transfer(0x66);
	SPI_PORT  |=   (1<<SPI_CS);
	tiny_delay();
}

void FLASH_Reset()
{
	SPI_PORT  &=  ~(1<<SPI_CS);
	SPI_Transfer(0x99);
	SPI_PORT  |=   (1<<SPI_CS);
	tiny_delay();
}

void FLASH_WriteEnable()
{
	SPI_PORT  &=  ~(1<<SPI_CS);
	SPI_Transfer(0x06);
	SPI_PORT  |=   (1<<SPI_CS);
	tiny_delay();
}

uint16_t FLASH_ReadWord(uint32_t faddr)
{
	uint16_t temp1=0, temp2=0;
	SPI_PORT  &=  ~(1<<SPI_CS);
	SPI_Transfer(0x03);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	temp1 = SPI_Transfer(0);
	temp2 = SPI_Transfer(0);
	SPI_PORT  |=   (1<<SPI_CS);
	tiny_delay();
							//sendchar(temp1 & 0xff);
							//sendchar(temp2 & 0xff);
							//sendchar(0x0d);
							//sendchar(0x0a);					
	return ((temp1 << 8) | (temp2 & 0x00ff));

}

void FLASH_EraseSector(uint32_t faddr)
{
	FLASH_UntilReady();
	FLASH_WriteEnable();
	SPI_PORT  &=  ~(1<<SPI_CS);
	SPI_Transfer(0x20);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	SPI_PORT  |=   (1<<SPI_CS);
	tiny_delay();
	FLASH_UntilReady();
}

void FLASH_WriteByte(uint32_t faddr, uint8_t wdata)
{
	FLASH_UntilReady();
	FLASH_WriteEnable();
	SPI_PORT  &=  ~(1<<SPI_CS);
	SPI_Transfer(0x02);
	SPI_Transfer((faddr >> 16) & 0xff);
	SPI_Transfer((faddr >> 8) & 0xff);
	SPI_Transfer(faddr & 0xff);
	SPI_Transfer(wdata & 0xff);
	SPI_PORT  |=   (1<<SPI_CS);
	tiny_delay();
	FLASH_UntilReady();
}
