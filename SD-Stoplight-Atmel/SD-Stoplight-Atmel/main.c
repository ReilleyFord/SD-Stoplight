#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#ifndef FALSE
#define FALSE 0
#define TRUE !FALSE
#endif

// Figure this out
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
#define  SDTYPE_UNKNOWN		0				/* card type not determined */
#define  SDTYPE_SD				1				/* SD v1 (1 MB to 2 GB) */
#define  SDTYPE_SDHC			2				/* SDHC (4 GB to 32 GB) */
/*
 * Arduino is split into blocks of pins. Each block needs 3 Registers
 * DDR (Data Direction Register) - Dictates which pins are Input or output
 * PORT - Which block of pins is being used.
 * PIN - Reads input value when a pin is selected as Input mode.
 */

// Setting up SPI and DDR (Data Direction Register)
// DDR will decide whether the port is Input (0xFF) or output (Default and 0x00)
// For example. Setting the fifth bit of DDRB to 1 means we are indicating that
// we want to use the pin associated to the fifth bit in PORTB to be used as output.
#define SPI_PORT  SPI0_CTRLA
#define SPI_DDR   PORTA_DIRSET

// Bits used by the SPI port
#define MOSI  PORTA_PIN1CTRL
#define MISO  PORTA_PIN2CTRL
#define SCK   PORTA_PIN3CTRL
// Fourth called SS for Slave Select, used for multiple slaves.

// Definition for CS, port, and DDR for the SD Card. - Should match chip?
#define SD_PORT     SPI0_CTRLA
#define SD_DDR      PORTA_DIRSET
#define SD_CS       PORTA_PIN4CTRL
#define SD_CS_MASK  (1<<SD_CS)

// Error codes for functions
#define SD_OK         0
#define SD_NO_DETECT  1
#define SD_TIMEOUT    2
#define SD_RWFAIL    -1

// Definitions for button port/DDR/PIN
#define SW_PORT PORTB
#define SW_DDR  PORTB_DIRSET
#define SW_PIN  PORTB_PIN0CTRL

#define SW_BIT  0
#define SW_MASK (1<<SW_BIT)

// Definitions for RED LED
#define LED_RED_PORT  PORTB				// Pulled low when button is pressed.
#define LED_RED_DDR   PORTB_DIR			// Data Direction Register
#define LED_RED_BIT   1				    // Set to HIGH when locked.
#define LED_RED_MASK  (1<<LED_RED_BIT)
#define LED_RED_OFF   (PORTB_OUT &= ~LED_RED_MASK)
#define LED_RED_ON    (PORTB_OUT |= LED_RED_MASK)

// Definitions for GREEN LED
#define LED_GREEN_PORT PORTB
#define LED_GREEN_DDR  PORTB_DIR
#define LED_GREEN_BIT  2		       // Set to HIGH when unlocked.
#define LED_GREEN_MASK (1<<LED_GREEN_BIT)
#define LED_GREEN_OFF  (PORTB_OUT &= ~LED_GREEN_MASK)
#define LED_GREEN_ON   (PORTB_OUT |= LED_GREEN_MASK)

// Definitions for YELLOW LED
#define LED_YELLOW_PORT  PORTB
#define LED_YELLOW_DDR   PORTB_DIR
#define LED_YELLOW_BIT   3		      // Set to HIGH when unable to read/damaged card.
#define LED_YELLOW_MASK (1<<LED_YELLOW_BIT)
#define LED_YELLOW_OFF  (PORTB_OUT &= ~LED_YELLOW_MASK)
#define LED_YELLOW_ON   (PORTB_OUT |= LED_YELLOW_MASK)

// CMDs to run against SD Card
#define  CMD_LOCK		 1
#define  CMD_UNLOCK		 2
#define  CMD_NONE		 3
#define  CMD_INFO		 4
#define  CMD_READBLK	 5
#define  CMD_PWD_LOCK	 6
#define  CMD_PWD_UNLOCK	 7
#define  CMD_PWD_CHECK	 8
#define  CMD_LOCK_CHECK  9
#define  CMD_ERASE		 10
#define  CMD_PWD_CLEAR   11

// Global variables
uint8_t sdtype;
uint8_t block[512];
uint8_t cardstatus[2];

/*
 * Local function declaration
 */
static void     Select(void);
static void     Deselect(void);
static uint8_t  SendByte(uint8_t  c);
static void     ProcessCommand(void);
static uint8_t  ReadCommand(void);
static int8_t   SendCommand(uint8_t  command, uint32_t  arg);
static int8_t   InitializeSD(void);
static int8_t   ReadSD(void);
static int8_t   ReadStatus(void);
static int8_t   WaitForData(void);

int main(void) {

  // First step, enable CS as output.
  //SD_DDR
  SPI_DDR |= SD_CS_MASK; // Setting the 2nd pin of PORTB (Chip Select) as output via DDRB
  Deselect(); // Make sure card is not selected.
	
  SPI_PORT |= ((1<<MOSI) | (1<<SCK));   // Flip bits for MOSI and Serial Clock
  SPI_DDR  |= ((1<<MOSI) | (1<<SCK));   // Mark pins as output
  SPI_PORT |= (1<<MISO);                // Flipping MISO bit.


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
  SW_PIN |= SW_MASK;

  /*
   * Enabling SPI via SPCR (Serial Peripheral Control Register)
   * SPE  - SPI Enable - Flip bit to enable SPI
   * MSTR - Master/Slave Select. If set Master mode is enabled.
   * SPR1 - Setting Clock Rate - Multiple options depending on SPX
   * SPR0 - Setting Clock Rate - SPR0, SPR1 and SPI2X dictate Clock Rate based on which bits are set.
   * In this configuration Clock Rate is set to fosc/128.
   */
  SPI_PORT |= (1<<SPI_ENABLE_bp) | (1<<SPI_MASTER_bp) | (SPI_PRESC_DIV128_gc);

  while(1) ProcessCommand();

  return 0;
}

/*
 * Flipping CS bit -- Selecting card.
 */
static void Select(void) {
  SD_PORT &= ~SD_CS_MASK;
}

/*
 * Flipping CS bit -- De-selecting card.
 */
static void Deselect(void) {
  SD_PORT |= SD_CS_MASK;
}

/*
 * ProcessCommand function
 * Beginning of code flow, kicked off by main(). This process loops awaiting
 * user input, either through switches or UART -- Some form of user input.
 */
static void ProcessCommand(void) {
  uint8_t         cmd, i;
  static uint8_t  prevCMD = 0;
  uint8_t         response;

  cmd = ReadCommand();

  if((cmd != prevCMD) && (prevCMD == CMD_NONE)) {

  response = InitializeSD();
  if(response != SD_OK) printf_P(PSTR("\n\r\n\rUnable to initialize card."));

  /*
   * If card passes init vibe check, begin processing command.
   */
   if(cmd == CMD_INFO) {
     printf_P(PSTR("\r\nCard Type: %d"), sdtype);
     response = ReadSD();
     if(response == SD_OK) {

     } else printf_P(PSTR("\r\nCard Registers could not be read."));
   }
  }
  prevCMD = cmd;
}

/*
 * ReadCommand function
 * This is called during ProcessCommand and is used to determine CMD options/state
 * Returns CMD selected as response.
 */
static uint8_t ReadCommand(void) {
  uint8_t response;

  _delay_ms(50);
  response = CMD_NONE;
  // Wait for data from UART.

  return response;
}

/*
 * SD Card Initialization function.
 * This will begin by setting SD to idle mode.
 * Then it will probe the card to check for SDHC which requires ACMD41 interface
 * and advanced intialization methods.
 * Returns SD_OK or SD_NO_DETECT.
 */
static int8_t InitializeSD(void) {
  int i;
  int8_t response;

  sdtype = SDTYPE_UNKNOWN;

  Deselect();

  // Send bytes while card stabilizes.
  for(i=0; i < 10; i++) SendByte(0xff);

  for(i = 0; i < 0x10; i++) {
    response = SendCommand(SD_IDLE, 0); // Try SD_IDLE until success or timeout.
    if(response == 1) break;
  }
  if(response != 1) return SD_NO_DETECT;

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
    if(response == 0x01) {
      for(i = 0; i < 4; i++) SendByte(0xff); // Burn the next 4 bytes returned (OCR)
    }
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
 * Kicks off a basic read of the available data registers.
 * OCR, CSD, CID.
 */
static int8_t ReadSD(void) {
   int8_t response;

   response = ReadStatus();

   return response;
}

/*
 * ReadStatus function
 * Reads the card status via CMD13
 */
static int8_t ReadStatus(void) {
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
 * For advanced initilization and commands this will send the required preface CMD55
 * Error codes will be 0xff for no response, 0x01 for OK, or CMD specific responses.
 */
static int8_t SendCommand(uint8_t cmd, uint32_t arg) {
  uint8_t response, crc;

  /*
   * Needed for SDC and advanced initilization.
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
   do {
      response = SendByte(0xff);
    } while((response & 0x80) != 0); // High bit cleared means OK

   // Switch statement with fall through and default. Deselecting card if no more R/W operations required.
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
 * ToDo: comment this.
 */
static unsigned char SendByte(unsigned char c) {
  SPI0_DATA = c; // Write to SPI Data Register - Writes out to MOSI via Hosts SPI Bus
  //while((SPSR & (1<<SPIF)) == 0); // Wait for SPSR and SPIF registers to clear.
  return SPI0_DATA;
}

/*
 * WaitForData function
 * Used for commands that require processing and timeouts while awaiting response
 * that is not 0xff.
 */
static int8_t WaitForData(void) {
	int16_t				i;
	uint8_t				response;

	for (i = 0; i < 100; i++) {
		response = SendByte(0xff);
		if (response != 0xff) break;
	}

	return  (int8_t) response;
}
