/*
 * bootSAH_4.c
 *
 * Created: 19/09/09 10:07:13 PM
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

// #ifdef __AVR_ATmega2560__
	// #define PROGLED_PORT	PORTB
	// #define PROGLED_DDR		DDRB
	// #define PROGLED_PIN		PINB7
// #endif

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

/*
 * function prototypes
 */
static void sendchar(char c);
void printByte(uint8_t numData);
static int	Serial_Available(void);
static unsigned char recchar(void);
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
 * since this bootloader is not linked against the avr-gcc crt1 functions,
 * to reduce the code size, we need to provide our own initialization
 */
void __jumpMain	(void) __attribute__ ((naked)) __attribute__ ((section (".init9")));
#include <avr/sfr_defs.h>

//*	for watch dog timer startup
void (*app_start)(void) = 0x0000;

int main(void)
{
	address_t		address			=	0;
	uint32_t fDataAddr = 0;
	uint32_t fCsumAddr = 0;
	uint8_t  fRdTries  = 0;
	uint8_t	 fRdData   = 0;
	uint8_t  fLSByte, fMSByte;
	uint16_t fDataWord;
	uint16_t fii, fjj;
	uint16_t fBootStat = 0;
	uint16_t fDataLen  = 0;
	uint16_t fChSumCal = 0;
	uint16_t fChSumRev = 0;
	uint32_t bootPageAddr = 0;
	unsigned char	msgBuffer[285];
			
	//unsigned long	boot_timeout;
	//unsigned long	boot_timer;
	//unsigned int	boot_state;

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

#if UART_BAUDRATE_DOUBLE_SPEED
	UART_STATUS_REG		|=	(1 <<UART_DOUBLE_SPEED);
#endif
	UART_BAUD_RATE_LOW	=	UART_BAUD_SELECT(BAUDRATE,F_CPU);
	UART_CONTROL_REG	=	(1 << UART_ENABLE_RECEIVER) | (1 << UART_ENABLE_TRANSMITTER);

	asm volatile ("nop");			// wait until port has changed

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

	//boot_timer	 =	0;
	//boot_state	 =	0;
	//boot_timeout =	3500000;		// 7 seconds , approx 2us per step when optimize "s"
	
	fDataAddr = 8192;
	fCsumAddr = 4352;
	
//************************************************************************	
//* Init ExFlash
	FLASH_UntilReady();
	FLASH_ResetEnable();
	FLASH_Reset();
	
	uint32_t temp_0=0;
	SPI_PORT  &=  ~(1<<SPI_CS);			// low
	SPI_Transfer(0x9F);
	temp_0 = (uint32_t)SPI_Transfer(0) << 16;
	temp_0 |= (uint32_t)SPI_Transfer(0) << 8;
	temp_0 |= (uint32_t)SPI_Transfer(0);
	SPI_PORT |=  (1<<SPI_CS);     		// high

	if(temp_0 == EXMEM_JEDEC)
	{
		fBootStat = FLASH_ReadWord(4096);
		//	printByte(fBootStat>>8);
		//	printByte(fBootStat&0xff);
		//	sendchar(0x0d);		sendchar(0x0a);
		if(fBootStat == 0x2323)
		{
			sendchar('S');		sendchar(0x0d);		sendchar(0x0a);
			fDataLen = FLASH_ReadWord(4098);

			if(fDataLen == 0x0B)
			{
				sendchar('L');		sendchar(0x0d);		sendchar(0x0a);
			}
			
			uint16_t lenPage = fDataLen;
			do 
			{
	//////////////////////////////////////////////	start get hex data		
				uint16_t lenByte = EXMEM_PAGE_SIZE;
				fii = 0;
				fRdTries  = 0;
				fChSumCal = 0;
				SPI_PORT  &=  ~(1<<SPI_CS);
				SPI_Transfer(0x03);
				SPI_Transfer((fDataAddr >> 16) & 0xff);
				SPI_Transfer((fDataAddr >> 8) & 0xff);
				SPI_Transfer(fDataAddr & 0xff);
				
				do 
				{
					fRdData = SPI_Transfer(0);
					msgBuffer[fii++] = fRdData;
					fChSumCal += fRdData;
					
					lenByte--;
				} while (lenByte);
				
				SPI_PORT  |=   (1<<SPI_CS);
				tiny_delay();
//	end get hex data	/////////////////////////////////	start get checksum			
				SPI_PORT  &=  ~(1<<SPI_CS);
				SPI_Transfer(0x03);
				SPI_Transfer((fCsumAddr >> 16) & 0xff);
				SPI_Transfer((fCsumAddr >> 8) & 0xff);
				SPI_Transfer(fCsumAddr & 0xff);
				
				fChSumRev = (uint16_t)SPI_Transfer(0) << 8;
				fChSumRev |= (uint16_t)SPI_Transfer(0);
				
				SPI_PORT  |=   (1<<SPI_CS);
				tiny_delay();
//	end get checksum	/////////////////////////////////////////////			
				if(fChSumRev == fChSumCal)
				{
					uint16_t fSize;
					fii=0;
					do 
					{
						fLSByte = msgBuffer[fii++];
						fMSByte = msgBuffer[fii++];
						fDataWord = ((uint16_t)fMSByte << 8) | fLSByte;
						
						printByte(fDataWord >> 8);
						printByte(fDataWord & 0xff);
						sendchar(',');
						
						fSize--;
					} while (fSize);
					sendchar(0x0d);		sendchar(0x0a);
				}
				
				lenPage--;
			} while (lenPage);
/*			
			for(fjj=0; fjj<fDataLen; fjj++)
			{
				fRdTries  = 0;
				fChSumCal = 0;
				SPI_PORT  &=  ~(1<<SPI_CS);
				SPI_Transfer(0x03);
				SPI_Transfer((fDataAddr >> 16) & 0xff);
				SPI_Transfer((fDataAddr >> 8) & 0xff);
				SPI_Transfer(fDataAddr & 0xff);
				for(fii=0; fii<256; fii++)
				{
					msgBuffer[fii] = SPI_Transfer(0);
					fChSumCal += msgBuffer[fii];
				}
				SPI_PORT  |=   (1<<SPI_CS);
				tiny_delay();
				
				fChSumRev = FLASH_ReadWord(fCsumAddr);
				
				if(fChSumRev == fChSumCal)
				{
					/////////////////checksum Matched
					sendchar('C');		sendchar(0x0d);		sendchar(0x0a);
					
					uint8_t  fLSByte, fMSByte;
					uint16_t fData;
					uint16_t fSize;
					
					fSize = 256;
					address = bootPageAddr;
					
					if(bootPageAddr < APP_END)				// erase only main section (bootloader protection)
					{
						
						sendchar('B');		sendchar(0x0d);		sendchar(0x0a);
						asm volatile ("nop");
						
						//boot_page_erase(bootPageAddr);		// Perform page erase
						//boot_spm_busy_wait();				// Wait until the memory is erased

						do
						{
							fLSByte = msgBuffer[fii];
							fMSByte = msgBuffer[fii+1];
							fData   = ((uint16_t)fMSByte << 8) | fLSByte;
							
								printByte(fData>>8);
								printByte(fData&0xff);
								sendchar(',');
							
							//boot_page_fill(address, fData);		// fill boot page
							address += 2;						// inc boot page address
							
							fSize	-=	2;
						} while(fSize);

						
						for(fii=0; fii<256; fii+=2)
						{
							fLSByte = msgBuffer[fii];
							fMSByte = msgBuffer[fii+1];
							fData   = ((uint16_t)fMSByte << 8) | fLSByte;
							
								printByte(fii>>8);
								printByte(fii&0xff);
								sendchar(',');
							
							//boot_page_fill(address, fData);		// fill boot page
							address += 2;						// inc boot page address
							
							
						}
						sendchar(0x0d);		sendchar(0x0a);
						
						//boot_page_write(bootPageAddr);			// write boot page
						//boot_spm_busy_wait();
						
						bootPageAddr += 256;			// SPM_PAGESIZE=256  point to next page to be erase

					}
					else
					{
						// write something on flash status
						sendchar('A');		sendchar(0x0d);		sendchar(0x0a);
						break;
					}					
					
					fDataAddr += 256;					//EXMEM_PAGE_SIZE;
					fCsumAddr += 2;
					tiny_delay();
				}
				else
				{
					///////////////////checksum Wrong
					fRdTries++;
					if(fRdTries>3)
					{
						////////////fail to read page
						sendchar('R');		sendchar(0x0d);		sendchar(0x0a);
						FLASH_WriteByte(BOOT_EROR_ADDR, (fjj >> 8));
						FLASH_WriteByte(BOOT_EROR_ADDR, (fjj & 0x00ff));
						break;
					}
				}
			}
			FLASH_EraseSector(4096);
			tiny_delay();
			FLASH_WriteByte(512, 0x4F);
			FLASH_WriteByte(513, 0x4B);
*/
		}
	}
	
	tiny_delay();
	
	//boot_rww_enable();				// enable application section

	// leaving bootloader  # bye bye # sayonara #
	asm volatile(
					"clr	r30		\n\t"
					"clr	r31		\n\t"
					"ijmp			\n\t"
				);
	
    for(;;)
    {
		sendchar('V');
		sendchar(0x0d);
		sendchar(0x0a);
		delay_ms(200);
    }
}

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

static void sendchar(char c)
{
	UART_DATA_REG	=	c;										// prepare transmission
	while (!(UART_STATUS_REG & (1 << UART_TRANSMIT_COMPLETE)));	// wait until byte sent
	UART_STATUS_REG |= (1 << UART_TRANSMIT_COMPLETE);			// delete TXCflag
}

void printByte(uint8_t numData)
{
	char chData;
	const char hexArr[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};
	chData = hexArr[numData>>4];
	sendchar(chData);
	chData = hexArr[numData&0xF];
	sendchar(chData);
}

//************************************************************************
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
