#include <p18cxxx.h>
#include "globals.h"
#include "USB.h"
#include "I2C_Slave.h"
#include "DEE Emulation 8-bit.h"

#pragma config WDTEN = OFF, PLLDIV= 2, STVREN=ON, XINST=OFF
#pragma config CPUDIV = OSC1, CP0 = OFF
#pragma config OSC = INTOSCPLL, T1DIG = OFF, FCMEN=OFF, IESO = OFF
#pragma config WDTPS = 1
#pragma config DSWDTOSC = INTOSCREF, RTCOSC = INTOSCREF, DSBOREN = OFF, DSWDTEN = OFF, DSWDTPS = 2
#pragma config IOL1WAY = OFF, MSSP7B_EN = MSK5
#pragma config WPCFG = OFF, WPDIS = OFF

unsigned char BlinkCount = 0;

#pragma udata first_scn
unsigned char eepromData[DATA_EE_TOTAL_SIZE];	// eepromData[] contains the EEPROM data bytes.
#pragma udata

#pragma udata second_scn
unsigned char eepromFlag[DATA_EE_TOTAL_SIZE];	// eepromFlag[] contains the flags that indicate if a new data byte has to be written in the EEPROM
#pragma udata

unsigned short eepromIndex = 0;					// The index in the eepromData[] and eepromFlag[] arrays.

void main(void)
{
	// PLLDIV<2:0> = 110
	// CPDIV<1:0> = 11
	OSCTUNE = bPLLEN;		// 0x40
	OSCCON = 0x70;
	//while(!(OSCCON&0x80));	// Wait for oscillator to be ready
	
	/* Initialize the  */
	DataEEInit();
	for(eepromIndex = 0; eepromIndex < DATA_EE_TOTAL_SIZE; eepromIndex++)
	{
		eepromFlag[eepromIndex] = 0;
		eepromData[eepromIndex] = DataEERead(eepromIndex);
	}

	ANCON0 = 0xFF;		// Disable all ADC channels
	ANCON1 = 0x1F;		// Disable all ADC channels, and turn off 1.2V bandgap reference
	RCONbits.IPEN = 1;	
	
	// Setup interrupts
	INTCON = 0xE0;
	INTCON2 = 0x80;
	INTCON3 = 0x00;
	PIE1 = 0x08;
	PIE2 = 0x90;
	PIE3 = 0x00;
	IPR1bits.SSP1IP = 1;		// I2C is a high priority interrupt
	
	
	// Setup crosspoint (PPS)
	
	RPINR21 = 4; 	// MSSP2 SPI MISO assigned to RP4 (RB1)
	RPOR18 = 10;	// MSSP2 SPI SCK assigned to RP18 (RC7)
	RPINR22 = 18;	// MSSP2 SPI SCK needs to be assigned as input as well
	RPOR3 = 9;		// MSSP2 SPI MOSI assigned to RP3 (RB0)
	RPOR2 = 12;		// MSSP2 SPI SSEL1 assigned to RP5 (RB2) - Note: SPI DMA Slave Select
	
	// Lock PPS registers (datasheet says this should be inline asm)	
	//_asm
	//	//BCF	  INTCON, GIE
	//	MOVLW 0x55
	//	MOVWF EECON2, 0
	//	MOVLW 0xAA
	//	MOVWF EECON2, 0
	//	//BSF   PPSCON, IOLOCK, BANKED
	//_endasm

	// Port A:          | USB_VDD | PWR_MOD_EN | PWR_DEMOD_EN | N/C | SPI_AD9548_CSB | SPI_AD8366_CSB | SPI_AD9523_CSB | SPI_ADF4351_RX_CSB |
	TRISA = 0x80;    //     I           O             O          O          O                 O               O                 O
	LATA = 0x0B;     //     0           0             0          0          1                 0               1                 1
	
	// Port B: 		   | SPI_ADF4351_TX_CSB | PWR_AMP_EN | SDA | SCL | LED | SPI_AD9122_CSB | MISO | MOSI |
	TRISB = 0x33;	//           O                O         I     I     O          O           I      I (to ensure no contention, MOSI gets set to Output when writing)
	LATB = 0xC4;    //           1                1         0     0     0          1           0      0 
	
	// Port C:		   | SCK | SPI_AD9643_CSB | USB+ | USB- | N/C | N/C | GA0 | GA1 |
	TRISC = 0x3F;	// 	  O          O           I      I      I     I     I     I
	LATC = 0x40;	//    0          1           0      0      0     0     0     0
	
	// ONLY AVAILABLE ON THE EVAL BOARD (PIC18F46J50), NOT ON THE ACTUAL BOARD (PIC18F24J50-I/ML)
	// Port E:         | RE7 | RE6 | RE5 | RE4 | RE3 | RE2 | RE1 | RE0 |
	//TRISE = 0xFC;  //     I     I     I     I     I     I     O     O
	//PORTE = 0x01;
	
	// Setup Timer 0 so it can blink the LED if requested
	T0CONbits.TMR0ON = 0;
	T0CONbits.T08BIT = 0;
	T0CONbits.T0CS = 0;
	T0CONbits.T0SE = 0;
	T0CONbits.PSA = 0;
	T0CONbits.T0PS = 0b011;
	
	// Default SPI settings
	SSP2STAT = (((0x0D21)>>2)&0x00C0);
	SSP2CON1 = ((0x0D21)&0x00FF);
	
	
#ifndef LEGACY_I2C	
	I2C_Slave_Init(0x50|((PORTCbits.RC1)<<1)|(PORTCbits.RC0));	// Standard EEPROM I2C address, plus an additional address with bit 4 high
																// ie 0x50 and 0x58, 0x51 and 0x59, 0x52 and 0x5A, 0x53 and 0x5B, depending on GA0 and GA1 bits
#else
	I2C_Slave_Init(0x50);	
#endif

	USB_Init();
	
	// Turn on MOD and DEMOD 5V regulators
	LATA |= 0x60;
	
	eepromIndex = 0;

	while(1)
	{
		while( (!eepromFlag[eepromIndex]) && (eepromIndex != (DATA_EE_TOTAL_SIZE - 1)))
		{
			eepromIndex++;
		}
		if(eepromFlag[eepromIndex])		// A new data byte has to be written in the EEPROM
		{
			eepromFlag[eepromIndex] = 0;						// Clear the flag that indicates that a new data byte has to be written in the EEPROM
			DataEEWrite(eepromData[eepromIndex], eepromIndex);	// Write the new data byte in the EEPROM
		}
		if(eepromIndex < (DATA_EE_TOTAL_SIZE - 1))
		{
			eepromIndex++;		// Increment the index of the eepromData[] and eepromFlag[] arrays.
		}
		else
		{
			eepromIndex = 0;	// Start again from first element.
		}

		USB_Loop();
		
		if(SSP1CON1&0xC0)
		{
			LATB &= ~0x08;	// Turn off LED if there is an I2C error
		}
	}
}

#pragma interrupt ISR_high
void ISR_high(void)
{	
	if(PIR2&bUSBIF)
	{
		PIR2 &= ~bUSBIF;
		USB_ISR();
	}
	if(PIR1bits.SSP1IF)
	{
		I2C_Slave_ISR();
		PIR1bits.SSP1IF = 0;
	}
	if(PIR1bits.TMR1IF)
	{
		PIR1bits.TMR1IF = 0;
	}
}

#pragma interrupt ISR_low
void ISR_low(void)
{
	if(INTCONbits.TMR0IF)
	{
		INTCONbits.TMR0IF = 0;
		
		if(LATB&0x08)
		{
			LATB &= ~0x08;			// LED On
		}
		else
		{
			LATB |= 0x08;
		}
			
		BlinkCount++;
		
		if(BlinkCount > 25)
		{
			T0CONbits.TMR0ON = 0;
			LATB &= ~0x08;
			BlinkCount = 0;	
		}
	}
}

#pragma code high_vector=0x08
void interrupt_at_high_vector(void)
{
_asm GOTO ISR_high _endasm
}
#pragma code low_vector=0x018
void interrupt_at_low_vector(void)
{
_asm GOTO ISR_low _endasm
}
#pragma code
