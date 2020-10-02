/************************************************************************/
/*                           SD Stoplight                               */
/*																		*/
/* This tool is one of a set of forensic tools within the framework of  */
/* SD card CMD42 Interaction via Serial Peripheral Interface (SPI)		*/
/* This SPI interaction is done from a Hardware Serial connection. In   */
/* In this case a custom logic board, SD card I/O shield, LED Lights, a */
/* switch, and an SD card.												*/
/*																		*/
/* This program was developed to validate the presence 	of a CMD42 lock	*/
/* enabled on an SD card. SD Stoplight will setup the hardware lines,	*/
/* LEDs, Switch Button, and SPI protocol. Once the hardware is setup    */
/* the program enters an infinite loop polling the Switch waiting for a */
/* state change. When the button is pressed and the state is changed	*/
/* The program will begin it's process of attempting to initialize an   */
/* SD card, perform the commands that are required for initialization   */
/* If the card cannot initialize the Yellow light is flipped. If the	*/
/* status of the card returns SD_OK then a read of the SD card registers*/
/* is performed determining the status of the CMD42 lock.				*/
/*																		*/ 
/*						Basic Functionality								*/
/*																		*/
/*	Green:  Card Unlocked												*/
/*	Yellow: No Card Detected/Non-Functional Card						*/
/*	Red:	Card is CMD42 Locked										*/
/*																		*/
/*	This idea was created by:											*/
/*	Benjamin Rutledge, Jeremy Dupuis, and Reilley Ford.  				*/
/*																		*/
/*																		*/
/*	Code developed by:													*/
/*	Reilley Ford														*/
/*																		*/
/*  Custom Board and Hardware created by:								*/
/*  Benjamin Rutledge													*/
/*																		*/
/*																		*/
/*  Honourable Mentions and Special Thanks to:							*/
/*  Inspector Heath Crichton											*/
/*  Detective Sergeant Rejean Carriere for support						*/
/*	Tyler Burke, Matt Parker for being my Rubber Ducky					*/
/************************************************************************/

#include <ctype.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#ifndef FALSE
#define FALSE 0
#define TRUE !FALSE
#endif

/*
 * CPU Clock Speed - 16Mhz
 */
#ifdef F_CPU
#undef F_CPU
#define F_CPU 16000000
#endif

/*
 * Baudrate setup.
 */
#define BAUDRATE    38400L
#define BAUDREG     ((unsigned int)((F_CPU/(BAUDRATE*8UL))-1))

/*
 * SD Card Commands 
 */
#define SD_IDLE        (0x40 + 0)   // CMD0: Set SD card to Idle
#define SD_INIT        (0x40 + 1)   // CMD1: Initialize SD Card
#define SD_INTER       (0x40 + 8)   // CMD8: Send Interface - Only for SDHC
#define SD_STATUS      (0x40 + 13)  // CMD13: Send Card Status
#define SD_SET_BLK     (0x40 + 16)  // CMD16: CMD16: Set Block Size (Bytes)
#define CMD55          (0x40 + 55)  // Multi-byte preface command
#define SD_OCR         (0x40 + 58)  // Read OCR
#define SD_ADV_INIT    (0xc0 + 41)  // ACMD41 Advanced Initialization for SDHC

/*
 * Options for Types of SD cards.
 */
#define  SDTYPE_UNKNOWN			0				/* card type not determined */
#define  SDTYPE_SD				1				/* SD v1 (1 MB to 2 GB) */
#define  SDTYPE_SDHC			2				/* SDHC (4 GB to 32 GB) */

// Error codes for functions
#define SD_OK         0
#define SD_NO_DETECT  1
#define SD_TIMEOUT    2
#define SD_RWFAIL    -1

// Bits used by the SPI port
#define MOSI  PIN1_bm // Master Out Slave In
#define MISO  PIN2_bm // Master In Slave Out 
#define SCK   PIN3_bm // Slave Clock
#define CS	  PIN4_bm // Card Select/Slave Select.

// Setting up SPI and DDR (Data Direction Register)
// DDR will decide whether the port is Input (0xFF) or output (Default and 0x00)
// For example. Setting the fifth bit of DDRB to 1 means we are indicating that
// we want to use the pin associated to the fifth bit in PORTB to be used as output.
#define SPI_PORT  SPI0_CTRLA
#define SPI_DDR	  PORTA_DIR
#define SPI_OUT   PORTA_OUT

// Mask for enabling Pullup on a pin and Input Sensing Configuration.
#define PULLUP_MASK (1<<3)
#define ISC_MASK	0x5 // Mask for Level Sensing of Pin (Generally Default)

// Definitions for button port/DDR/PIN
#define SW_DDR		PORTB_DIR
#define SW_PIN		PORTB_PIN0CTRL
#define SW_MASK		PIN0_bm

// Definitions for RED LED
// Pulled low when button is pressed.
#define LED_RED_DDR   PORTB_DIRSET		// Data Direction Register
#define LED_RED_BIT   1				    // Set to HIGH when locked.
#define LED_RED_MASK  (1<<LED_RED_BIT)
#define LED_RED_OFF   (PORTB_OUTCLR = LED_RED_MASK)
#define LED_RED_ON    (PORTB_OUTSET |= LED_RED_MASK)

// Definitions for YELLOW LED
#define LED_YELLOW_DDR   PORTB_DIRSET
#define LED_YELLOW_BIT   2		      // Set to HIGH when unable to read/damaged card.
#define LED_YELLOW_MASK (1<<LED_YELLOW_BIT)
#define LED_YELLOW_OFF  (PORTB_OUTCLR = LED_YELLOW_MASK)
#define LED_YELLOW_ON   (PORTB_OUTSET |= LED_YELLOW_MASK)

// Definitions for GREEN LED
#define LED_GREEN_DDR  PORTB_DIRSET
#define LED_GREEN_BIT  3		       // Set to HIGH when unlocked.
#define LED_GREEN_MASK (1<<LED_GREEN_BIT)
#define LED_GREEN_OFF  (PORTB_OUTCLR = LED_GREEN_MASK)
#define LED_GREEN_ON   (PORTB_OUTSET |= LED_GREEN_MASK)

// CMDs to run against SD Card
#define  CMD_NONE		 3
#define  CMD_INFO		 4
#define  CMD_READBLK	 5
#define  CMD_PWD_CHECK	 8
#define  CMD_LOCK_CHECK  9

// Global variables
uint8_t sdtype;
uint8_t cardstatus[2];

/*
 * Local function declaration
 */
static void     Select(void);
static void     Deselect(void);
static uint8_t  SendByte(uint8_t  c);
static void     ProcessCommand(void);
static int8_t   SendCommand(uint8_t  command, uint32_t  arg);
static int8_t   InitializeSD(void);
static int8_t   ReadSD(void);

int main(void) {
	
  // Enable Global Interrupts
  CPU_SREG = CPU_I_bm;
 
  // Set SPI Data Direction Register. Drive outputs to SPI port MOSI and SCK.
  SPI_DDR |= MOSI;	// MOSI -- Output
  SPI_DDR &= ~MISO; // MISO -- Input
  SPI_DDR |= SCK;	// SCK  -- Output
  SPI_DDR |= CS;	// CS   -- Output 
  Deselect(); // Deselect CS by driving output High.
  PORTA_PIN2CTRL |= ISC_MASK | PULLUP_MASK; // MISO ISC and Pullup enable.

 /**
   * Enabling SPI via SPCR (Serial Peripheral Control Register)
   * SPI_ENABLE_bm  - SPI Enable - Flip bit to enable SPI
   * SPI_MASTER_bm - Master/Slave Select. If set Master mode is enabled.
   * SPI_PRESC_DIV128_gc - Setting Clock Rate - Prescaler, Clock Speed is 16mhz - 20Mhz. 
   * Prescaler divides by 128. Necessary because SPI maximum clock speed limitations.
   * In this configuration Clock Rate is set to fosc/128.
   **/
  SPI_PORT = SPI_ENABLE_bm 
		   | SPI_MASTER_bm 
		   | SPI_PRESC_DIV128_gc; 
		   
  // Setup hardware lines/pins
  // Red LED
  LED_RED_OFF;
  LED_RED_DDR |= LED_RED_MASK;

  // Green LED
  LED_GREEN_OFF;
  LED_GREEN_DDR |= LED_GREEN_MASK;

  // Yellow LED
  LED_YELLOW_OFF;
  LED_YELLOW_DDR |= LED_YELLOW_MASK;

  // Switch setup - Marking as input
  SW_DDR &= ~SW_MASK;
  SW_PIN |= ISC_MASK | PULLUP_MASK; // ISC and Pullup enable. Switch Pin
 
 // Begin Infinite Loop polling switch to begin processing.
  while(1) {
	 if((PORTB_IN & SW_MASK) == 0) { 
		LED_RED_OFF;
		LED_YELLOW_OFF;
		LED_GREEN_OFF;
		_delay_ms(500);
		LED_RED_ON;
		_delay_ms(1000);
		LED_YELLOW_ON;
		_delay_ms(1000);
		LED_GREEN_ON;
		_delay_ms(2500);
		LED_RED_OFF;
		LED_GREEN_OFF;
		LED_YELLOW_OFF;
		ProcessCommand();
	 }
  }
 
  return 0; // Should never happen.
}

/*
 * Flipping CS bit -- Selecting card.
 * Pulled low for selection.
 */
static void Select(void) {
	SPI_OUT &= ~CS;
}

/*
 * Flipping CS bit -- De-selecting card.
 * Pulled High for deselection.
 */
static void Deselect(void) {
	 SPI_OUT |= CS;
}

/*
 * ProcessCommand function
 * Beginning of code flow.
 * Attempt to Initialize SD
 * Read SD for CMD42 Lock.
 */
static void ProcessCommand(void) {
  uint8_t         response;

  response = InitializeSD();
  if(response != SD_OK) {
	LED_YELLOW_ON;
	return;
  }
  
  /*
   * If card passes init vibe check, begin processing command.
   */
   response = ReadSD();
   if(response == SD_OK) {
	   if(!(cardstatus[1] & 0x01)) LED_GREEN_ON; // No CMD42 lock detected.
	   else if ((cardstatus[1] & 0x01) == 1) LED_RED_ON; // CMD42 lock detected and enabled.
   } else {
	  LED_YELLOW_ON;
	  LED_GREEN_OFF;
	  LED_RED_OFF;
   }
}

/*
 * SD Card Initialization function.
 * This will begin by setting SD to idle mode.
 * Then it will probe the card to check for SDHC which requires ACMD41 interface
 * and advanced initialization methods.
 * Returns SD_OK or SD_NO_DETECT.
 */
static int8_t InitializeSD(void) {
  int i;
  int8_t response;

  sdtype = SDTYPE_UNKNOWN;
  
  Deselect();

  // Send bytes while card stabilizes.
  for(i=0; i < 74; i++) SendByte(0xff);
  
  for(i = 0; i < 0x10; i++) {
    response = SendCommand(SD_IDLE, 0); // Try SD_IDLE until success or timeout.
    if(response == 1) break;
  }
  if(response != 1) {
	LED_YELLOW_ON;
	LED_GREEN_OFF;
	LED_RED_OFF;
	return SD_NO_DETECT;
  }

  SendCommand(SD_SET_BLK, 512); // Set block length to 512 bytes.

  // Always attempt ACMD41 first for SDC then drop to CMD1
  response = SendCommand(SD_INTER, 0x1aa);
  if(response == 0x01) {
    for(i = 0; i < 4; i++) SendByte(0xff);          // Clock through 4 bytes to burn 32 bit lower response.
    for(i = 20000; i > 0; i--) {                    // Send Advanced init cmd until initialization complete and response is 0x00
      response = SendCommand(SD_ADV_INIT, 1UL<<30); // Send advanced init with HCS bit 30 set.
      if(response == 0) break;
    }
    sdtype = SDTYPE_SDHC;
  } else { // Begin initializing SDSC -- CMD1
    response = SendCommand(SD_OCR, 0); // Not necessary if voltage is set correctly.
    if(response == 0x01)
      for(i = 0; i < 4; i++) SendByte(0xff); // Burn the next 4 bytes returned (OCR)
    for(i = 20000; i > 0; i--) {
      response = SendCommand(SD_INIT, 0);
      if(response == 0) break;
    }
    SendCommand(SD_SET_BLK, 512); // SDSC might reset block length to 1024, reinit to 512.
    sdtype = SDTYPE_SD;
  }

  SendByte(0xff); // End initialization with 8 clocks.

  // Initialization should be completed. The SPI clock rate can be set to maximum, usually 20MHz. Depends on card.
  return SD_OK;
}

/*
 * ReadSD function
 * Reads the SD Status and returned registers. 
 * Checks for CMD42 Lock.
 */
static int8_t ReadSD(void) {
  cardstatus[0] = SendCommand(SD_STATUS, 0);
  cardstatus[1] = SendByte(0xff);

  SendByte(0xff);
  return  SD_OK;
}

/*
 * SendCommand
 * Function accepts an SD CMD and 4 byte argument.
 * Exchanges CMD and arg with CRC and 0xff filled bytes with card.
 * Returns the response provided by the card.
 * For advanced initialization and commands this will send the required preface CMD55
 * Error codes will be 0xff for no response, 0x01 for OK, or CMD specific responses.
 */
static int8_t SendCommand(uint8_t cmd, uint32_t arg) {
  uint8_t response, crc;

  /*
   * Needed for SDC and advanced initialization.
   * ACMD(n) requires CMD55 to be sent first.
   */
  if(cmd & 0x80) {
    cmd = cmd & 0x7f; // Stripping high bit.
    response = SendCommand(CMD55, 0);
    if (response > 1) return response;
  }

  Deselect();
  SendByte(0xff);
  Select();
  SendByte(0xff);

  /*
   * Begin sending command
   * Command structure is 48 bits??
   */
   SendByte(cmd | 0x40);
   SendByte((unsigned char)(arg>>24));
   SendByte((unsigned char)(arg>>16));
   SendByte((unsigned char)(arg>>8));
   SendByte((unsigned char)(arg&0xff));
   if(cmd == SD_IDLE)  crc = 0x95;
   if(cmd == SD_INTER) crc = 0x87;
   SendByte(crc);

   // Send clocks waiting for timeout. 
	for (int i = 0; i<10; i++) {
		response = SendByte(0xff);
		if((response & 0x80) == 0) break; // High bit flipped means ok.
	}

   // Switch statement with fall through and default. De selecting card if no more R/W operations required.
   switch (cmd) {
     case SD_ADV_INIT :
     case SD_SET_BLK :
     case SD_IDLE :
     case SD_INIT :
     case CMD55 :
       Deselect();
       SendByte(0xff);
     default :
       break;
   }

   return response;
}

/*
 * SendByte function.
 * This function is the core of the SD card SPI interaction. 
 * SendByte will write a Byte passed as an unsigned char c 
 * This byte will be written to the SPI Data register
 * The SPI Transfer Complete flag is then checked via Polling method
 * Until the transfer is complete. The flag is then cleared after the flag and the Data register are read. 
 * The SPI Data register is read for a response and the response is returned by the function. 
 */
static unsigned char SendByte(unsigned char c) {

	SPI0_DATA = c; // Write to SPI Data Register - Writes out to MOSI via Hosts SPI Bus

	while(!(SPI0_INTFLAGS & SPI_IF_bm)); // Wait until Transfer Complete flag is set. 

	return SPI0_DATA;
}