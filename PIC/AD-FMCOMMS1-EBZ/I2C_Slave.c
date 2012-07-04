#include <p18cxxx.h>
#include "globals.h"
#include "I2C_Slave.h"
#include "descriptors.h"
#include "SPI_Master.h"
#include "DEE Emulation 8-bit.h"

unsigned char I2C_Address = 0xFF;
unsigned char address = 0;
unsigned char I2C_Incoming[64];
unsigned char I2C_Outgoing[64];
unsigned char I2C_Index = 0;
unsigned char I2C_Command = 0;

unsigned short EEPROM_Address;

#define I2C_ADDRESS_MASK 0x08

unsigned char newEepromTransfer = 1;

extern char eepromData[DATA_EE_TOTAL_SIZE];
extern char eepromFlag[DATA_EE_TOTAL_SIZE];

void I2C_Slave_Init(unsigned char slave_address)
{
	SSP1STAT = 0x00;
	SSP1CON1 = 0x16; 			// I2C Slave mode, 7-bit address
#ifdef LEGACY_I2C	
	SSP1CON2 = 0x80; 		// Enable General Call
#else
	SSP1CON2 = 0x80|0x10; 		// Enable General Call. Mask bit 4 so two addresses will match
#endif
	SSP1ADD = (slave_address<<1);
	SSP1CON1bits.SSPEN = 1;		// Enable I2C
}

void I2C_Slave_ISR(void)
{
	int i;
	
	if(SSP1STATbits.D_A == 0) // Indicates that the last byte received or transmitted was address
	{
		newEepromTransfer = 1;
		I2C_Address = (SSP1BUF>>1);
		I2C_Index = 0;
		
#ifndef LEGACY_I2C
		if((I2C_Address&I2C_ADDRESS_MASK))	// Custom commands
		{
#endif
			if(SSP1STATbits.R_W)	// Master wants to read data from slave
			{
				if(I2C_Command == 0x01)		// Get iProduct
				{
					SSP1BUF = iProduct[0];
					I2C_Index++;
				}
				else if(I2C_Command == 0x03)
				{
					SSP1BUF = I2C_Outgoing[0];
					I2C_Index++;
				}
			}
#ifndef LEGACY_I2C
		}
		else	// I2C EEPROM Emulation
		{
			if(SSP1STATbits.R_W)	// Master wants to read data from us
			{
				SSP1BUF = eepromData[EEPROM_Address];
				EEPROM_Address++;
			}
		}
#endif
		SSP1CON1bits.CKP = 1; // ACK
	}
	else if(SSP1STATbits.R_W)	// Read Data
	{
#ifndef LEGACY_I2C
		if(I2C_Address&I2C_ADDRESS_MASK)	// Custom commands
		{
#endif
			switch(I2C_Command)
			{
				case 0x00:	// Return 0
					SSP1BUF = 0x00;
					break;
				case 0x01:	// Get iProduct
					if(iProduct[I2C_Index] != 0)
					{
						SSP1BUF = iProduct[I2C_Index++];
					}
					else
					{
						I2C_Command = 0;
						SSP1BUF = 0x00;
					}
					break;
				case 0x02:	// Blink (no Tx involved)
					break;
				case 0x03:	// Rx SPI (assuming rx count > 0)
					SSP1BUF = I2C_Outgoing[I2C_Index++];
					break;
				case 0x04:	// Write SPI (no Tx involved)
					break;
			}
#ifndef LEGACY_I2C
		}
		else	// EEPROM Emulation
		{
			SSP1BUF = eepromData[EEPROM_Address];
			EEPROM_Address++;
		}
#endif
		SSP1CON1bits.CKP = 1;
	}
	else					// Indicates that the last byte received or transmitted was data (Write)
	{
#ifndef LEGACY_I2C
		if(I2C_Address&I2C_ADDRESS_MASK)	// Custom commands
		{
#endif
			if(I2C_Index == 0)
			{
				I2C_Command = SSP1BUF;
				I2C_Index = 1;
			}
			else
				I2C_Incoming[(I2C_Index++)-1] = SSP1BUF;
			
			if(SSP1STATbits.P)		// Indicates that a Stop bit has been detected last
			{
				unsigned short spi_settings;
				unsigned short spi_selects;
				
				switch(I2C_Command)
				{
					case 0x02:		// Blink
						T0CONbits.TMR0ON = 1;
						break;
					case 0x03:		// Set SPI settings
						spi_settings = (I2C_Incoming[0]);
						spi_settings = spi_settings<<8;
						spi_settings |= I2C_Incoming[1];
						spi_selects = I2C_Incoming[2];
						spi_selects = spi_selects << 8;
						spi_selects |= I2C_Incoming[3];
						SPI_Init(spi_settings,spi_selects);
						if(spi_settings>>10)
						{
							SPI_StartRxDMA((unsigned int)&I2C_Outgoing);
						}
						break;
					case 0x04:		// Write SPI
						SPI_StartTxDMA((unsigned int)&I2C_Incoming, (I2C_Index-1));
						break;
					default:
						break;
				}
			}
#ifndef LEGACY_I2C
		}
		else	// EEPROM Emulation
		{
			if(newEepromTransfer)
			{
				EEPROM_Address = SSP1BUF;
				newEepromTransfer = 0;
			}
			else
			{
				eepromData[EEPROM_Address] = SSP1BUF;
				eepromFlag[EEPROM_Address] = 1;			// Set the flag to announce that a new data has to be written in EEPROM.
				EEPROM_Address++;
			}
		}
#endif
	}
}
