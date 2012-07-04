#include <p18cxxx.h>
#include "SPI_Master.h"
#include "globals.h"

unsigned short ChipSelects = 1;			// This is a bit map, to allow for asserting multiple CS's at the same time
unsigned char ChipSelectState = 1;
unsigned char RxLength = 0;
unsigned short spi_settings = 0;

void SPI_Init(unsigned short settings, unsigned short chip_selects)
{
	SSP2STAT = (settings&0x18)<<3;
	SSP2CON1 = 0x20 | ((settings&0x04)<<2) |(settings&0x03);
	ChipSelectState = (settings&0x20)>>5;
	ChipSelects = chip_selects;
	RxLength = settings>>10;
	spi_settings = settings;
}

void AssertChipSelects(int chip_select_map, int state)
{
	if(chip_select_map&0x0001)	// Slave Select 1
		if(state)
			LATB |= 0x04;		// High
		else
			LATB &= ~0x04;		// Low
	
	if(chip_select_map&0x0002)	// Slave Select 2
		if(state)
			LATC |= 0x40;		// High
		else
			LATC &= ~0x40;		// Low
	
	if(chip_select_map&0x0004)	// Slave Select 3
		if(state)
			LATA |= 0x08;		// High
		else
			LATA &= ~0x08;		// Low
			
	if(chip_select_map&0x0008)	// Slave Select 4
		if(state)
			LATA |= 0x02;		// High
		else
			LATA &= ~0x02;		// Low
			
	if(chip_select_map&0x0010)	// Slave Select 5
		if(state)
			LATA |= 0x01;		// High
		else
			LATA &= ~0x01;		// Low
			
	if(chip_select_map&0x0020)	// Slave Select 6
		if(state)
			LATB |= 0x80;		// High
		else
			LATB &= ~0x80;		// Low
			
	if(chip_select_map&0x0040)	// Slave Select 7	These are inverted since the AD8366 needs CS high during transfers!!!
		if(state)
			LATA &= ~0x04;		// Low
		else
			LATA |= 0x04;		// High
}

void SPI_StartTxDMA(unsigned short BufferAddress, int length)
{
	int i;
	TRISB = 0x32;						// Enable MOSI for output
	AssertChipSelects(ChipSelects, 0);
	DMACON1 = 0x34; // 0011 0100
	DMACON2 = 0x00; // 0000 0000
	DMABCH = 0x00;
	DMABCL = length-1;
	TXADDRH = BufferAddress>>8;
	TXADDRL = BufferAddress; 
	DMACON1bits.DMAEN = 1;
	
	while(DMACON1bits.DMAEN);
	TRISB = 0x33;						// Force MOSI to Hi-Z for safety
	AssertChipSelects(ChipSelects, ChipSelectState);
}

void SPI_StartRxDMA(unsigned short BufferAddress)
{
	int i;
		
	AssertChipSelects(ChipSelects, 0);
	
	if(spi_settings&0x0040)	// 3-Wire mode
	{
		TRISB = 0x33;
		RPOR3 = 0;		// MSSP2 SPI MOSI unassigned
		RPINR21 = 3; 	// MSSP2 SPI MISO assigned to RP3 (RB0)
	}
	
	DMACON1 = 0x30; // 0011 0000
	DMACON2 = 0x00; // 0000 0000
	DMABCH = 0x00;
	DMABCL = RxLength-1;
	TXADDRH = 0x00;
	TXADDRL = 0x00;
	RXADDRH = BufferAddress>>8;
	RXADDRL = BufferAddress;
	DMACON1bits.DMAEN = 1;
	
	while(DMACON1bits.DMAEN);
	AssertChipSelects(ChipSelects, ChipSelectState);
	
	if(spi_settings&0x0040)	// 3-Wire mode
	{
		RPINR21 = 4; 	// MSSP2 SPI MISO assigned to RP4 (RB1)
		RPOR3 = 9;		// MSSP2 SPI MOSI assigned to RP3 (RB0)
	}
}
