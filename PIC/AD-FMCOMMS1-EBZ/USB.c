#include <p18cxxx.h>
#include "globals.h"
#include "USB.h"
#include "descriptors.h"
#include "SPI_Master.h"

void Process_SETUP_Packet(void);

int GetDescriptorCount = 0;

unsigned char USB_USTAT;
unsigned char USB_BDSTAT;
unsigned char USB_BDCNT;
unsigned char USB_UADDR = 0;
unsigned char ConfigurationNumber = 0;

unsigned char data_toggle = 0;
unsigned char EP1IN_toggle = 0;

unsigned char iManufacturer[15] = "Analog Devices";


unsigned short Math_Min(unsigned short a, unsigned short b)
{
	return a<b?a:b;
}

void USB_Init(void)
{	
	UCON = 0;			// Disable USB
	
	UIR = 0;
	UIE = 0; //bIDLEIF | bTRNIF | bUERRIF | bURSTIF | bACTVIF;
	UEIR = 0;
	UEIE = 0xFF;

	UCFG = bFSEN | bUPUEN;
	
	// Endpoint 0 configuration
	UEP0 = bEPINEN | bEPOUTEN | bEPHSHK;
	
	BD0CNT	= 64;
	BD0ADRL = 0x00;			// Buffer starts at 0x0500 - 0x053F
	BD0ADRH = 0x05;	
	BD0STAT = bDTSEN | bUOWN;		// BD0 = EP0 OUT
	
	BD1CNT	= 64;
	BD1ADRL = 0x40;			// Buffer starts at 0x0540 - 0x057F
	BD1ADRH = 0x05;	
	BD1STAT = bDTSEN;		// BD1 = EP0 IN

	// Endpoint 1 OUT configuration
	UEP1 = bEPHSHK | bEPOUTEN | bEPINEN;
	BD2CNT = 64;
	BD2ADRL = 0x80;			// Buffer starts at 0x0580 - 0x05BF
	BD2ADRH = 0x05;
	BD2STAT = bUOWN;// BD2 = EP1 OUT
	
	// Endpoint 1 IN configuration
	BD3CNT = 0;
	BD3ADRL = 0xC0;			// Buffer starts at 0x05C0 - 0x05FF
	BD3ADRH = 0x05;
	BD3STAT = bDTSEN; // BD3 = EP1 IN
	
	//UCON = bUSBEN;			// Enable USB
	
	//while(UCON&bSEO);
}

void USB_Loop(void)
{
	// Check for presence of VUSB through GPIO pin, and enable or disable USBEN accordingly
	
	if((PORTAbits.RA7 == 1) && (UCONbits.USBEN == 0))
	{
		UIE = bSTALLIF | bIDLEIF | bTRNIF | bSOFIF | bUERRIF;
		UCON = bUSBEN;			// Enable USB
	}
	else if (PORTAbits.RA7 == 0)
	{
		UCON = 0;	
	}
}


void USB_ISR(void)
{
	if(!UCONbits.SE0)
		UIE |= bURSTIF | bIDLEIF;
	
	if(UIR&bACTVIF)					// Bus Activity Detect Interrupt
	{
		UCONbits.SUSPND = 0;
		while (UIRbits.ACTVIF) { UIRbits.ACTVIF = 0; }
	}
	
	if(UCONbits.SUSPND == 1)
		return;
	
	if(UIR&bURSTIF)					// USB Reset Interrupt
	{
		UIRbits.TRNIF = 0;			// Clear interrupts
		UIRbits.TRNIF = 0;			// Clear interrupts
		UIRbits.TRNIF = 0;			// Clear interrupts
		UIRbits.TRNIF = 0;			// Clear interrupts
		
		//UEP0 = 0;
		//UEP1 = 0;
		//UEP2 = 0;
		
		UADDR = 0;
		USB_UADDR = 0;
	
		//USB_Init();
		//UIE = bSTALLIF | bIDLEIF | bTRNIF | bSOFIF | bUERRIF;
		//UCON = bUSBEN;			// Enable USB
		
		UIRbits.URSTIF = 0;
	}
	
	if(UIR&bIDLEIF)					// Idle Detect Interrupt
	{
		UIRbits.IDLEIF = 0;		// Clear interrupt
		//UCONbits.SUSPND = 1;	// Suspend
	}
	
	if(UEIR)
	{	
		LATB |= 0x08;	// LED Off	
		UEIR = 0x00;	// Clear all errors
	}
	if((UIR&bSOFIF))					// Start-Of-Frame Token Interrupt
	{
		UIR &= ~bSOFIF;
	}
	if(UIR&bSTALLIF)				// A STALL Handshake Interrupt
	{
		UIR &= ~bSTALLIF;
	}
	
	if(UIR&bTRNIF)					// Transaction Complete Interrupt
	{
		USB_USTAT = USTAT;
		USB_BDSTAT = *((unsigned char*)((&BD0STAT)+(USTAT&0x7C)));
		USB_BDCNT = *((unsigned char*)((&BD0CNT)+(USTAT&0x7C)));;
		UIRbits.TRNIF = 0;		// Clear interrupt
		//data_toggle = 1;
		if( (USB_USTAT&bENDP) == 0x00 )	// Endpoint 0
		{
			switch(USB_BDSTAT&0x3C)
			{
				case (9<<2):	// IN Token
					UADDR = USB_UADDR;
					break;
				case (1<<2):	// OUT Token
					BD0CNT = 64;
					BD0STAT = bDTSEN | bUOWN;		// Return control to SIE
					BD1CNT = 0;
					BD1STAT = bDTS | bDTSEN | bUOWN;
					break;
				case (13<<2):	// SETUP Token
					Process_SETUP_Packet();
					break;
				case (3<<2):	// DATA0
					while(1);
					break;
				case (11<<2):	// DATA1
					while(1);
					break;
				default:
					break;	
			}
		}
		else if( (USB_USTAT&bENDP) == (0x01<<3) )	// Endpoint 1
		{
			if(USB_USTAT&bDIR)		// IN
			{
				// A vendor command fills the buffer, so there's nothing to do here
			}
			else
			{
				SPI_StartTxDMA(0x0580, BD2CNT); // BD2 location
				
				BD2CNT = 64;
				BD2STAT = bUOWN;		// BD2 = EP1 OUT
			}
			
		}
		else
		{
			while(1);
		}
		
		
	}
	
	if(UIR&bUERRIF)					// USB Error Condition Interrupt
	{
		UIR &= ~bUERRIF;
	}
	
}


void Process_STANDARD_Request(unsigned char bmRequestType, unsigned char bRequest, unsigned short wValue, unsigned short wIndex, unsigned short wLength)
{
	if(bRequest == 0x00)		// GET_STATUS
	{
		BD1CNT = 2;
		B1DATA[1] = 0x00;
		
		if( (bmRequestType&0x1F) == 0x00 )  // Device
		{
			B1DATA[0] = 0x01;
		}
		else if( (bmRequestType&0x1F) == 0x02 )	// Endpoint
		{
			switch(wIndex&0x7F)
			{
				case 1:
					B1DATA[0] = (UEP1&0x01);
					break;
				case 2:
					B1DATA[0] = (UEP2&0x01);
					break;
			}
		}
		else
		{
			BD1STAT = bBSTALL | bUOWN;		// BD1 = EP0 IN
		}
		
		BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
	}
	else if(bRequest == 0x01)	// CLEAR_FEATURE
	{
		BD1CNT = 0;
		if(wValue == 0x00)		// ENDPOINT_HALT
		{
			switch(wIndex&0x7F)
			{
				case 1:
					UEP1bits.EPSTALL = 0;
					break;
				case 2:
					UEP2bits.EPSTALL = 0;	
			}				
				
			BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
		}
		else
		{
			BD1STAT = bBSTALL | bUOWN;
		}
	}
	else if(bRequest == 0x03)	// SET_FEATURE
	{
		BD1CNT = 0;
		if(wValue == 0x00)		// ENDPOINT_HALT
		{
			switch(wIndex&0x7F)
			{
				case 1:
					UEP1bits.EPSTALL = 1;
					break;
				case 2:
					UEP2bits.EPSTALL = 1;
					break;
			}
				
			BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
		}
		else
		{
			BD1STAT = bBSTALL | bUOWN;
		}
	}
	else if(bRequest == 0x05)	// SET_ADDRESS
	{
		BD1CNT = 0;
		BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
		USB_UADDR = wValue;
	}				
	else if(bRequest == 0x06)	// GET_DESCRIPTOR
	{
		if(wValue == 0x0100)	// DEVICE
		{
			B1DATA[0] = 18;		// bLength
			B1DATA[1] = 0x01;	// bDescriptorType
			B1DATA[2] = 0x00;	// bcdUSB[0]
			B1DATA[3] = 0x02;	// bcdUSB[1]
			B1DATA[4] = 0xFF;	// bDeviceClass
			B1DATA[5] = 0x00;	// bDeviceSubClass
			B1DATA[6] = 0x00;	// bDeviceProtocol
			B1DATA[7] = 64;		// bMaxPacketSize
			B1DATA[8] = 0x56;	// idVendor[0]
			B1DATA[9] = 0x04;	// idVendor[1]
			B1DATA[10] = 0x24;	// idProduct[0]
			B1DATA[11] = 0x70;	// idProduct[1]
			B1DATA[12] = 0x00;	// bcdDevice[0]
			B1DATA[13] = 0x02;	// bcdDevice[1]
			B1DATA[14] = 0x01;	// iManufacturer
			B1DATA[15] = 0x02;	// iProduct
			B1DATA[16] = iSerialNumber[0]==0x00?0x00:0x03;	// iSerialNumber
			B1DATA[17] = 1;	// bNumConfigurations
			
			BD1CNT	= Math_Min(18,wLength);
			BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
		}
		else if( (wValue&0xFF00) == 0x0200)		// CONFIGURATION
		{
			// Configuration Descriptor One
			B1DATA[0] = 9;				// bLength
			B1DATA[1] = 0x02;			// bDescriptorType
			B1DATA[2] = 32;				// wTotalLength[0]
			B1DATA[3] = 0;				// wTotalLength[1]
			B1DATA[4] = 1;				// bNumInterfaces
			B1DATA[5] = 1;				// bConfigurationValue
			B1DATA[6] = 0;				// iConfiguration
			B1DATA[7] = 0b11000000; 	// bmAttributes
			B1DATA[8] = 0; 				// bMaxPower
			// Interface 0 Descriptor
			B1DATA[9]  = 9;				// bLength
			B1DATA[10] = 0x04;			// bDescriptorType
			B1DATA[11] = 0;				// bInterfaceNumber
			B1DATA[12] = 0;				// bAlternateSetting
			B1DATA[13] = 2;				// bNumEndpoints
			B1DATA[14] = 0xFF;			// bInterfaceClass
			B1DATA[15] = 0xFF;			// bInterfaceSubClass
			B1DATA[16] = 0xFF;			// bInterfaceProtocol
			B1DATA[17] = 0;				// iInterface
			// Endpoint 1 OUT Descriptor
			B1DATA[18] = 7;				// bLength
			B1DATA[19] = 0x05;			// bDescriptorLength
			B1DATA[20] = 1;				// bEndpointAddress
			B1DATA[21] = 0b00000010; 	// bmAttributes
			B1DATA[22] = 0x40;			// wMaxPacketSize[0]
			B1DATA[23] = 0x00;			// wMaxPacketSize[1]
			B1DATA[24] = 0;				// bInterval
			// Endpoint 1 IN Descriptor
			B1DATA[25] = 7;				// bLength
			B1DATA[26] = 0x05;			// bDescriptorLength
			B1DATA[27] = 0x80|1;		// bEndpointAddress
			B1DATA[28] = 0b00000010; 	// bmAttributes
			B1DATA[29] = 0x40;			// wMaxPacketSize[0]
			B1DATA[30] = 0x00;			// wMaxPacketSize[1]
			B1DATA[31] = 0;				// bInterval
			
			BD1CNT	= Math_Min(32,wLength);
			BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
		}
		else if( (wValue&0xFF00) == 0x0300)		// STRING
		{
			if( (wValue&0xFF) == 0x00 )	// Language ID
			{
				B1DATA[0] = 4;				// bLength
				B1DATA[1] = 0x03;			// bDescriptorType
				
				B1DATA[2] = 0x09;			// wLANGID[0][1]
				B1DATA[3] = 0x04;			// wLANGID[0][0]
				
				BD1CNT	= Math_Min(B1DATA[0],wLength);
				BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
			}
			else
			{
				int i = 0;
				int j = 2;
				
				B1DATA[1] = 0x03;			// bDescriptorType
				
				if( (wValue&0xFF) == 0x01 ) // iManufacturer
				{
					while(iManufacturer[i] != 0x00)
					{
						B1DATA[j++] = iManufacturer[i];					
						B1DATA[j++] = 0x00;
						i++;
					}
				}
				else if ( (wValue&0xFF) == 0x02 )	// iProduct
				{
					while(iProduct[i] != 0x00)
					{
						B1DATA[j++] = iProduct[i];
						B1DATA[j++] = 0x00;
						i++;
					}
				}
				else if ( (wValue&0xFF) == 0x03 )	// iSerialNumber
				{
					while(iSerialNumber[i] != 0x00)
					{
						B1DATA[j++] = iSerialNumber[i];
						B1DATA[j++] = 0x00;
						i++;
					}
				}
				
				B1DATA[0] = (2*i)+2;				// bLength			
				
				BD1CNT	= Math_Min(B1DATA[0],wLength);
				BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
			}
		}
		else if( (wValue&0xFF00) == 0x0600)		// DEVICE_QUALIFIER
		{
			BD1CNT = 0;
			BD1STAT = bBSTALL | bUOWN;			// Since this is a full-speed only device, it must reply to this with a stall
		}
		else
		{
			// Unknown command, so stall
			BD1CNT = 0;
			BD1STAT = bBSTALL | bUOWN;
		}
	}
	else if(bRequest == 0x08)	// GET_CONFIGURATION
	{
		B1DATA[0] = ConfigurationNumber;
		BD1CNT = 1;
		BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
	}
	else if(bRequest == 0x09)	// SET_CONFIGURATION
	{
		ConfigurationNumber = wValue;
		data_toggle = 0;
		BD1CNT	= 0;
		BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
	}
	else if(bRequest == 0x0A)	// GET_INTERFACE
	{
		BD1STAT = bBSTALL | bUOWN;
	}
	else if(bRequest == 0x0B)	// SET_INTERFACE
	{
		BD1STAT = bBSTALL | bUOWN;
	}
	else
	{
		// Unknown command, so stall
		BD1CNT = 0;
		BD1STAT = bBSTALL | bUOWN;
	}
}

void Process_CLASS_Request(unsigned char bmRequestType, unsigned char bRequest, unsigned short wValue, unsigned short wIndex, unsigned short wLength)
{
}

void Process_VENDOR_Request(unsigned char bmRequestType, unsigned char bRequest, unsigned short wValue, unsigned short wIndex, unsigned short wLength)
{
	unsigned char i=0;
	unsigned char tempLen = 0;
	
	switch(bRequest)
	{
		case 0x01:	// Blink LED
			T0CONbits.TMR0ON = 1;
			BD1CNT	= 0;
			BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
			break;
		
		case 0x10:	// Write SPI (Legacy style)
					/* Bit mappings for wIndex:
			        		Slave Select Enables			  | SSP2STAT     |                        SSPCON1                        |
					|  15  |  14  |  13  |  12  |  11  |  10  |   9  |   8  |   7  |   6  |   5  |   4  |   3  |   2  |   1  |   0  |
					| SS5  |  SS4 | SS3  | SS2  | SS1  | SSH  |  SMP |  CKE |  --  |  --  |SSPEN | CKP	|          SSPM3:0          |
           Default: |   0  |   0  |   0  |   0  |   1  |   1  |   0  |   0  |   0  |   0  |   1  |   0  |   0      0      1      0  |	Fosc/64
																										|   0      0      0      1  |	Fosc/16
																										|   0      0      0      0  |	Fosc/4    */
				AssertChipSelects(((wIndex)&0xF800)>>11,0);

				if((bmRequestType&0x80) == 0x00)	// Control Out (Write)
				{
					SSP2STAT = (((wIndex)>>2)&0x00C0);
					SSP2CON1 = ((wIndex)&0x00F0)|0x02;							// Force to Fosc/64 clock speed
					
					tempLen = (unsigned char)(wLength); //SetupPkt.wLength;
					
					if(tempLen == 0)
					{
						SSP2BUF = ((wValue)>>8);
						while((SSP2STAT&0x01));				
						while(!(SSP2STAT&0x01));
				
						SSP2BUF = ((wValue)&0x00FF);
						while((SSP2STAT&0x01));				
						while(!(SSP2STAT&0x01));

						BD1CNT	= 0;
						BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
					}					
				}
				else	// Control In (Read)
				{
					SSP2STAT = (((wIndex)>>2)&0x00C0);
					SSP2CON1 = ((wIndex)&0x00FF);
					
					tempLen = (unsigned char)(wLength); //SetupPkt.wLength;
					
					if(tempLen == 1)
					{
						i = SSP2BUF;			// Dummy read
						SSP2BUF = ((wValue)&0x00FF);
						while((SSP2STAT&0x01));				
						while(!(SSP2STAT&0x01));
						B1DATA[0] = SSP2BUF;
					}	
					if(tempLen == 2)
					{
						i = SSP2BUF;			// Dummy read
						
						SSP2BUF = ((wValue)>>8);
						while((SSP2STAT&0x01));				
						while(!(SSP2STAT&0x01));
						B1DATA[0] = SSP2BUF;

						SSP2BUF = ((wValue)&0x00FF);
						while((SSP2STAT&0x01));				
						while(!(SSP2STAT&0x01));
						B1DATA[1] = SSP2BUF;
					}
		            
		            BD1CNT	= tempLen;
					BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN				
				}

				if(((wIndex)&0x0400))	// Pull high if SSH is enabled
				{						
					LATB |= 0x04;		// SSEL1 high
					LATC |= 0x40;		// SSEL2 high
				}				
				
			break;
			
			case 0x20:	// Read / Set GPIO (Legacy)
				if((bmRequestType&0x80) == 0x00)	// Control Out (Write)
				{
					//if(wIndex&0x0001)
					//	LATAbits.LATA5 = 1;
					//else
					//	LATAbits.LATA5 = 0;
						
					//if(wIndex&0x0002)
					//	LATAbits.LATA6 = 1;
					//else
					//	LATAbits.LATA6 = 0;
					
//#warning This is temporary, to test out sending out a sync signal over two pins
					if(wIndex&0x0004)
						LATB |= 0xC0;
						//LATBbits.LATB6 = 1;
					else
						LATB &= 0x3F;
						//LATBbits.LATB6 = 0;
						
					// FOR DEBUG ONLY - LEDs ON PIC EVAL BOARD, NOT AVAILABLE ON REAL HARDWARE
					//if(wIndex&0x0008)
					//	LATEbits.LATE0 = 1;
					//else
					//	LATEbits.LATE0 = 0;
					
					// FOR DEBUG ONLY - LEDs ON PIC EVAL BOARD, NOT AVAILABLE ON REAL HARDWARE
					//if(wIndex&0x0010)
					//	LATEbits.LATE1 = 1;
					//else
					//	LATEbits.LATE1 = 0;
						
					BD1CNT	= 0;
					BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
				}
				else	// Control In (Read)
				{
				}
			break;
			
			case 0x21:	// Read / Set GPIO direction (Legacy)

			break;
			
			case 0x50:	// SPI (Rev 2)
				// The SPI settings are set here. Actual data transfer occurs with EP1 and EP2
				
				SPI_Init(wIndex, wValue);
				
				if((wIndex&0xFC00) > 0)		// Prepare for Read
				{
					SPI_StartRxDMA(0x05C0);  // BD3 location
					
					BD3CNT = wIndex>>10;				
					if(EP1IN_toggle)
					{
						BD3STAT = bDTSEN | bDTS | bUOWN;		// BD2 = EP1 OUT
						EP1IN_toggle = 0;
					}
					else
					{
						BD3STAT = bDTSEN | bUOWN;		// BD2 = EP1 OUT
						EP1IN_toggle = 1;
					}
				}
				
				BD1CNT	= 0;
				BD1STAT = bDTS | bDTSEN | bUOWN;		// BD1 = EP0 IN
			break;
	}
}

void Process_SETUP_Packet()
{
	unsigned char bmRequestType;
	unsigned char bRequest;
	unsigned short wValue;
	unsigned short wIndex;
	unsigned short wLength;

	bmRequestType = B0DATA[0];
	bRequest = B0DATA[1];
	wValue = B0DATA[2] | (((unsigned short)B0DATA[3])<<8);
	wIndex = B0DATA[4] | (((unsigned short)B0DATA[5])<<8);
	wLength = ((unsigned short)B0DATA[6]) | (((unsigned short)B0DATA[7])<<8);
	BD0CNT = 64;
	BD1STAT = bDTSEN;
	
	if( !(bmRequestType&0x80) && (wLength > 0))
		BD0STAT = bDTS | bDTSEN | bUOWN;		// Return control to SIE
	else
		BD0STAT = bDTSEN | bUOWN;		// Return control to SIE
	
	UCON &= ~bPKTDIS;	
	
	switch(bmRequestType&0x60)
	{
		case 0x00:	// STANDARD
			Process_STANDARD_Request(bmRequestType, bRequest, wValue, wIndex, wLength);
			break;
		case 0x20:	// CLASS
			Process_CLASS_Request(bmRequestType, bRequest, wValue, wIndex, wLength);
			break;
		case 0x40:	// VENDOR
			Process_VENDOR_Request(bmRequestType, bRequest, wValue, wIndex, wLength);
			break;
		default:
			break;
	}
	
	
}
