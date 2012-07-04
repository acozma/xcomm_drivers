/************************************************************************
*
*   Emulating Data EEPROM for PIC18 Microcontrollers 
*          
*
* This application note provides a standard interface to an efficient
* Data EEPROM emulation algorithm and uses available program memory. 
* It is designed for Microchip Technology 8-bit PIC devices 
* which currently include PIC18FJ products. The
* project is initially configured to use PIC18F87J11-I/PT on the HPC 
* Explorer Development Board. To use a different device, simply select 
* new device in MPLAB, replace C18 linker script and rebuild.
* User must select number pages of program memory, the starting address
* for the pages of program memory, erase/write limit and 
* emulated DEE size. These are defined in "NoFilDEE.inc". Note the 
* starting address used for the emulation pages must be defined on a
* erase page boundary.
* At build-time, the linker reserves the defined number of pages 
* starting at the defined EMULATION_PAGES_START_ADDRESS in program
* memory.
* Compiler error occurs if more than 255 DEE locations are declared,
* less than 2 pages of program memory are reserved, greater than 65,535
* erase/write cycles specified, if insufficient program memory is 
* available or the page address is not defined on a erase page boundary. 
* Call initialization routine and clear status flags before attempting 
* any other DEE operation.
*
*************************************************************************

*************************************************************************
* FileName:     DEE Emulation 8-bit.c
* Dependencies: NoFilDEE.asm
                DEE Emulation 8-bit.h
                NoFilDEE.inc
* Compiler:     MPLAB C18, v3.02 or higher
* Company:      Microchip Technology, Inc.
*
* Software License Agreement
*
* Copyright © 2007 Microchip Technology Inc.  All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and distribute 
* Software only when embedded on a Microchip microcontroller or digital 
* signal controller, which is integrated into your product or third party 
* product (pursuant to the sublicense terms in the accompanying license 
* agreement).  
*
* You should refer to the license agreement accompanying this Software for 
* additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY 
* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
* PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE 
* LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
* CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY 
* DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY 
* INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST 
* PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, 
* SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO 
* ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*
* Author                Date            Comment
*************************************************************************
* S. Cowden             2007/05/24      Version 1.0.0 - Initial Release
* Pradeep Budagutta     2008/10/24      Version 1.1.0 - Supports more than 255 addresses
* Pradeep Budagutta     2009/08/31      Version 1.1.1 - A bug related to initial storage of value 0xFF solved
* Pradeep Budagutta     2010/03/03      Version 1.1.2 - A bug related to disabling interrupts inside UnlockWrite solved
************************************************************************/

#include <p18cxxx.h>
#include "GenericTypeDefs.h"
#include "DEE Emulation 8-bit.h"

// User constant validation
#if DATA_EE_BANKS == 0
    #error Minimum data EE banks is 1
#endif

#if DATA_EE_SIZE > 255
    #error Maximum data EE size is 255
#endif

#if NUM_DATA_EE_PAGES < 2
    #error Minimum number of program memory pages is 2
#endif

#if ERASE_WRITE_CYCLE_MAX > 65535
    #error Maximum number of erase/write cycles is 65,535
#endif

// Insure EMULATION PAGES are on Erase page boundary
#if (EMULATION_PAGES_START_ADDRESS % 1024) != 0
	#error Emulation page start address must be aligned on a 1024 byte boundary.
#endif
 
DATA_EE_FLAGS dataEEFlags;

//  Data EE info stored in PM in following format
//  Status in first four locations of PM page,
//  8-bit DEE Address (even address) 8-bit DEE data (odd address) 
extern far rom unsigned char Emulation_Page[DATA_EE_BANKS * NUM_DATA_EE_PAGES][(NUMBER_OF_INSTRUCTIONS_IN_PAGE*2)];

#define DEE_TBL_PTR(bank, page) (short long)&Emulation_Page[(NUM_DATA_EE_PAGES * (bank)) + (page)]

#pragma code

/************************************************************************
UnlockWrite

This routine saves the current CPU priority and sets it the highest
user level of 7. It calls an assembly routine to perform an unlock 
sequence and sets the WR bit in EECON1. The WR bit is polled until it
clears indicating the flash operation is complete. The previous CPU
priority is restored.

Parameters:		None
Return:			None
Side Effects:	None
************************************************************************/
void UnlockWrite(void)
{
     volatile unsigned char oldGIE;
      
     oldGIE = INTCONbits.GIE;
     INTCONbits.GIE = 0;

     EECON2 = 0x55;
     EECON2 = 0xAA;
     EECON1bits.WR=1;
     while(EECON1bits.WR);                       // Wait for write to complete
     
     EECON1 = 0b00000000;                        // Clear WREN
     INTCONbits.GIE = oldGIE;                    // Restore GIE
     return;
}

/************************************************************************
GetPageStatus

This routine returns the page status for the selected page for the
selected field. The return value is right shifted into LSb position.

Parameters:		Page number, Status Field
Return:			Right justified bit value representing selected Status
                Field value
Side Effects:	None
************************************************************************/
int GetPageStatus(unsigned char bank, unsigned char  page, unsigned char  field)
{
    
    unsigned char  pageStatus;
 
    TBLPTR = DEE_TBL_PTR(bank, page);
 
    
    //Read 1st location of current page.                                
    _asm 
    TBLRD
    _endasm
    pageStatus = TABLAT;                    // store page status
     
    switch(field)
    {
        case STATUS_AVAILABLE:
            pageStatus = (pageStatus & 1);
            break;
        case STATUS_CURRENT:
            pageStatus = ((pageStatus & 2) >> 1);
            break;
        case STATUS_EXPIRED:
            pageStatus = ((pageStatus & 4) >> 2);
            break;
        default:
            pageStatus = 0;
            break;

    }
    
    return(pageStatus);
            
}

/************************************************************************
ErasePage

This routine erases the selected page.

Parameters:		Page number
Return:			None
Side Effects:	None
************************************************************************/
void ErasePage(unsigned char bank, unsigned char page)
{
    // ERASE PROGRAM MEMORY BLOCK (512 Words)
    TBLPTR = DEE_TBL_PTR(bank, page); 
      
    EECON1 =  ERASE;                    // Setup for Erase: FREE=1,WREN=1
    
    UnlockWrite();                      // Call unlock
    
    return;
}

/************************************************************************
GetNextAvailCount

This routine finds the active page and performs a backward search to find
the first available location. The available location is determined by 
reading an LSB (even address) with 0xFF. The returned value can be added
to the first address in page to compute the available address. A return
value of 0 means the entire page was filled which is an error condition.
This routine can be called by the user to determine how full the current
page is prior to a pack.

Parameters:		None
Return:			Page offset to next available location
Side Effects:	None
************************************************************************/
unsigned int GetNextAvailCount(unsigned char bank)
{
    unsigned int            i;
    unsigned int            currentPage; 
    unsigned char           tempAddr;
    unsigned short long     savedTBLPTR;   

    savedTBLPTR = TBLPTR;

    // Find the active page.

    for (currentPage = 0;
         (currentPage < NUM_DATA_EE_PAGES) &&
         (GetPageStatus(bank, currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT);
         currentPage++) {}

    // Point the table page pointer to the active page
    TBLPTR = DEE_TBL_PTR(bank, currentPage);

    i= 2;
    TBLPTR+=2;

    do 
    {
        i+=2;
        TBLPTR+=2;
        _asm                        // look for the next available address 
        TBLRD                       
        _endasm                                 
        tempAddr = TABLAT;
    }
    while ((tempAddr != 0xFF) && (i < NUMBER_OF_INSTRUCTIONS_IN_PAGE*2));

    if (i == NUMBER_OF_INSTRUCTIONS_IN_PAGE * 2)
    {                  
        i=0;  //Error - No available locations     
    }

    TBLPTR = savedTBLPTR;
    return(i);                                        
}

/************************************************************************
PackEE

This routine finds the active page and an unexpired packed page. The most
recent data EEPROM values are located for each address using ReadEE
function and written into write latches. Page status is read from active
page and erase/write count is incremented if page 0 is packed. After all
information is programmed and verified, the current page is erased. The 
packed page becomes the active page. This function can be called at any-
time by the user to schedule the CPU stall.

Parameters:		None
Return:			Status value (0 for pass)
Side Effects:	Generates CPU stall during program/erase operations and
                overwrites program memory write latches. Data EE flags 
                may be updated
************************************************************************/
int PackEE(unsigned char bank)
{
    unsigned int            i;
    unsigned short long     packedPageAddr;         
    unsigned short long     currentPageAddr;       
    unsigned short long     savedTBLPTR;
    unsigned char           currentPage; 
    unsigned char           packedPage;
    unsigned char           latchAddr;
    unsigned char           latchData;
    unsigned char           dataEEFlags_sh;
    
    DWORD_VAL                statusCount;

    savedTBLPTR = TBLPTR;        //Save TBLPTR 

    // Point the table page pointer to the emulation pages
//    TBLPTR = (short long)&Emulation_Page [0];

    // Find the active page.
    for (currentPage = 0;
         (currentPage < NUM_DATA_EE_PAGES) && 
         (GetPageStatus(bank, currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT);
         currentPage++) {}  

    if (currentPage == NUM_DATA_EE_PAGES)
    {
        TBLPTR= savedTBLPTR;
        SetPagePackB4Init(1);
        return(3);      // Error - no active page
    }
    else
    {
        // Find the next inactive page to use
        packedPage = currentPage + 1;
        if (packedPage == NUM_DATA_EE_PAGES)
        {
            packedPage = 0;
        }
        while(GetPageStatus(bank, packedPage, STATUS_EXPIRED) == PAGE_EXPIRED)
        {
            packedPage++;
            if (packedPage == NUM_DATA_EE_PAGES)
            {
                packedPage = 0;
            }
            if(packedPage == currentPage)
            {
                TBLPTR = savedTBLPTR;
                SetPageExpiredPage(1);
                return(1);      // Error - all pages expired
            }
        }
    }

    // Set table page pointer to the current and packed page starting addresses    
    TBLPTR = DEE_TBL_PTR(bank, packedPage);
    packedPageAddr = TBLPTR;
    TBLPTR = DEE_TBL_PTR(bank, currentPage);
    currentPageAddr = TBLPTR;

    if(GetNextAvailCount(bank))
    {
        SetPagePackB4PageFull(1);    // Pack called before the page was full                                        
    }

    //Set TBLPTR 1st location after status in packed page
    TBLPTR = packedPageAddr;
    TABLAT = 0xFF;
    for (i=0; i<4 ;i++)
    {
        _asm
        TBLWTPOSTINC                 // Insure first four location are FFh
        _endasm
    }                     
    
    latchAddr = 0;
    dataEEFlags_sh = dataEEFlags.val;
    SetAddrNotFound(0);              // Intilize flag

    do
    {
        while((latchAddr != (DATA_EE_SIZE)) && (i < NUMBER_OF_INSTRUCTION_IN_ROW))
        {
            latchData = DataEERead((255 * bank) + latchAddr);                         
            if (GetAddrNotFound())             
            {                        //if address is unwritten, skip to next address
                SetAddrNotFound(0);
            }
            else
            {
                TABLAT = latchAddr;
                _asm
                TBLWTPOSTINC
                _endasm
                TABLAT = latchData;       
                _asm
                TBLWTPOSTINC
                _endasm
                i+=2;
            }
            latchAddr++;

            while((latchAddr == DATA_EE_SIZE ) && (i < NUMBER_OF_INSTRUCTION_IN_ROW))
            {
                TABLAT = 0xFF;            
                _asm
                TBLWTPOSTINC
                _endasm
                TABLAT = 0xFF;            
                _asm
                TBLWTPOSTINC
                _endasm
                i+=2;
            }
        }
        TBLPTR--;   
        EECON1 = PROGRAM_ROW;
        UnlockWrite();                            
        TBLPTR++;
        i = 0;
    }
    while(latchAddr != (DATA_EE_SIZE));

    dataEEFlags.val = dataEEFlags_sh;   //Restore status flags

    //Verify the data written correctly into the pack page    

    // Point to first location after status
    TBLPTR = packedPageAddr+4;              
    _asm 
    TBLRDPOSTINC
    _endasm
    latchAddr = TABLAT;
    _asm 
    TBLRDPOSTINC
    _endasm
    latchData = TABLAT;

    while (latchAddr != 0xFF)   // Stop when you get to end of data
    {
       if(DataEERead((255 * bank) + latchAddr) != latchData)
       {
            SetPageWriteError(1);
            return(7);          //Error - data does not match
       }
       _asm 
       TBLRDPOSTINC
       _endasm
       latchAddr = TABLAT;
       _asm 
       TBLRDPOSTINC
       _endasm
       latchData = TABLAT;
    }

    TBLPTR = currentPageAddr;

    // Load the status and EW count into the Array
    _asm
    TBLRDPOSTINC                            
    _endasm
    statusCount.byte.LB = TABLAT;    // EE Status  (8-bits)      
    _asm
    TBLRDPOSTINC                           
    _endasm
    statusCount.byte.HB = TABLAT;    // Not used but available for higher E/W counts
    _asm
    TBLRDPOSTINC                            
    _endasm
    statusCount.byte.UB = TABLAT;    // Lower byte of  E/W count
    _asm
    TBLRD                           
    _endasm
    statusCount.byte.MB = TABLAT;    // Upper byte of E/W count
   
    //Increment E/W counter if packed page is in original active page location
    
    if(packedPage == 0)
    {
        statusCount.word.HW++;
    }

    if(statusCount.word.HW >= ERASE_WRITE_CYCLE_MAX -1)   // Error - E/W cycle count exceeded
    {
        statusCount.byte.LB &= 0b11111011;
        SetPageExpiredPage(1);
    }

    TBLPTR = packedPageAddr;

    TABLAT = statusCount.byte.LB;     // EE Status
    _asm
    TBLWTPOSTINC
    _endasm
    TABLAT = statusCount.byte.HB;     // Not used but available for higher E/W counts
    _asm
    TBLWT
    _endasm
    EECON1 = PROGRAM_WORD;
    UnlockWrite();
    TBLPTR++;
    TABLAT = statusCount.byte.UB;     // Lower byte of  E/W count
    _asm
    TBLWTPOSTINC
    _endasm
    TABLAT = statusCount.byte.MB;     // Upper byte of E/W count
    _asm
    TBLWT
    _endasm
    EECON1 = PROGRAM_WORD;
    UnlockWrite();

    // Verify E/W count and status was written correctly
    _asm
    TBLRDPOSTDEC                            
    _endasm
    if(TABLAT != statusCount.byte.MB)
    {
        SetPageWriteError(1);
        return(7);
    }   
    _asm
    TBLRDPOSTDEC                            
    _endasm
    if(TABLAT != statusCount.byte.UB)
    {
        SetPageWriteError(1);
        return(7);
    }
    _asm
    TBLRDPOSTDEC                            
    _endasm
    if(TABLAT != statusCount.byte.HB)
    {
        SetPageWriteError(1);
        return(7);
    }   
    _asm
    TBLRD                            
    _endasm
    if(TABLAT != statusCount.byte.LB)
    {
        SetPageWriteError(1);
        return(7);
    }

    ErasePage(bank, currentPage);
    TBLPTR = savedTBLPTR;
    return(GetPageExpiredPage());          
}

/************************************************************************
DataEEInit

This routine finds an unexpired page to become an active page. It then 
counts the number of active pages. If no active pages are found, the
first unexpired page is initialized for emulation. If one active page is 
found, it is assumes a reset occurred and the function does nothing. If 
two active pages are found, it is assumes a reset occurred during a pack.
The second page is erased and a pack is called. If three, an error code 
is returned as the allocated memory is assumed to be corrupted. This
function must be called prior to any other operation.

Parameters:		None
Return:			Status value (0 for pass)
Side Effects:	Data EE flags may be updated.
************************************************************************/
unsigned char DataEEInit(void)
{
    unsigned char pageCnt;
    unsigned char erasePage;
    unsigned int savedTBLPTR;        //Context save of TBLPAG value. Current and packed page are on same page.
    unsigned int currentPage;
    unsigned int packedPage;         //Array row (PM page) of packed page
    unsigned char bank;
   
    savedTBLPTR = TBLPTR;
    // Point the table page pointer to the emulation pages
    TBLPTR = (short long)&Emulation_Page [0];

    for(bank = 0; bank < DATA_EE_BANKS; bank++)
    {
        pageCnt = 0;
        erasePage = 0;
        packedPage = 0;

        // Find unexpired page
        for (currentPage = 0;
            (currentPage < NUM_DATA_EE_PAGES) && 
            (GetPageStatus(bank, currentPage, STATUS_EXPIRED) == PAGE_EXPIRED);
            currentPage++) {}        
        
        if (currentPage == NUM_DATA_EE_PAGES)                
        {
            TBLPTR = savedTBLPTR;
            SetPageExpiredPage(1);
            return(1);     // Error - All pages expired
        }

        // Count active page(s).
        for (currentPage = 0;currentPage < NUM_DATA_EE_PAGES;currentPage++) 
        {
            if(GetPageStatus(bank, currentPage, STATUS_CURRENT) == PAGE_CURRENT)
            {
                pageCnt++;
            }   
        }        

        //If no active pages found, initialize page 0
        if(pageCnt == 0)
        {
            ErasePage(bank, 0);
            TBLPTR = DEE_TBL_PTR(bank, 0);
            TABLAT = 0xFC;                            //New page: unavailable, active
            _asm
            TBLWTPOSTINC
            _endasm
            TABLAT = 0xFF;                   
            _asm
            TBLWT
            _endasm
            EECON1 = PROGRAM_WORD;
            UnlockWrite();                            
            TBLPTR++;
            TABLAT = 0x00;                            //Reset count 
            _asm
            TBLWTPOSTINC
            _endasm
            TABLAT = 0x00;                   
            _asm
            TBLWT
            _endasm
            EECON1 = PROGRAM_WORD;
            UnlockWrite();  
            TBLPTR = savedTBLPTR;
            continue;
        }
        //If one active page, do nothing
        else if(pageCnt == 1)
        {
            TBLPTR = savedTBLPTR;
            continue;
        }
        //If two active pages, erase second and repack first
        else if(pageCnt == 2)
        {
            if((GetPageStatus(bank, NUM_DATA_EE_PAGES - 1, STATUS_CURRENT) == PAGE_CURRENT) &&
                (GetPageStatus(bank, 0, STATUS_CURRENT) == PAGE_CURRENT))
            {
                currentPage = NUM_DATA_EE_PAGES - 1;
                erasePage = 0;
            }
            else
            {
                currentPage = 0;
                while((GetPageStatus(bank, currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT) &&
                    (currentPage < NUM_DATA_EE_PAGES))
                {
                    currentPage++;
                }
                erasePage = currentPage + 1;
                if (erasePage == NUM_DATA_EE_PAGES)
                {
                    erasePage = 0;
                }
            }
            ErasePage(bank, erasePage);

            if(!GetNextAvailCount(bank))
            {
                PackEE(bank);
            }
            TBLPTR = savedTBLPTR;
            continue;
        }
        else
        {
            TBLPTR = savedTBLPTR;
            SetPageCorruptStatus(1);
            return(6);
        }
    }
    return(0);
}

/************************************************************************
DataEERead

This routine verifies the address is valid. If not, the Illegal Address
flag is set and 0xFF is returned. It then finds the active page. If an 
active page can not be found, the Page Corrupt status bit is set and 
0xFF is returned. A reverse search of the active page attempts to find 
the matching address in the program memory MSB (odd address). If a match
is found, the corresponding data EEPROM data (even address) is returned, 
otherwise 0xFFFF is returned. This function can be called by the user.

Parameters:		Data EE address
Return:			Data EE data or 0xFFFF if address not found
Side Effects:	Data EE flags may be updated.
************************************************************************/
    
unsigned int DataEERead(unsigned int addr)
{
    unsigned int            i;
    unsigned int            currentPage;
    unsigned char           latchData;
    unsigned char           latchAddr;
    unsigned short long     savedTBLPTR;
    unsigned char           bank;

    savedTBLPTR = TBLPTR;                                  // Save previous value of the TBLPTR

    if((addr >= DATA_EE_TOTAL_SIZE))
    {
        SetPageIllegalAddress(1);
        return(0xFF);
    }

    bank = addr / DATA_EE_SIZE;

    // Find the active page.
    for (currentPage = 0;
         (currentPage < NUM_DATA_EE_PAGES) && 
         (GetPageStatus(bank, currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT);
         currentPage++) {}        

    if (currentPage == NUM_DATA_EE_PAGES)
    {
        TBLPTR = savedTBLPTR;
        SetPageCorruptStatus(1);
        return(0xFF);     // Error - no active page
    }

    TBLPTR = (DEE_TBL_PTR(bank, currentPage + 1)) - 2;

    i = NUMBER_OF_INSTRUCTIONS_IN_PAGE-1;    // -1 for the status & EW count

    do
    {
        _asm
        TBLRD
        _endasm
        latchAddr = TABLAT;
        TBLPTR -= 2;

        i--;
    }
    while((i> 0) && (latchAddr != (addr % DATA_EE_SIZE)));

    if(!i)
    {
        SetAddrNotFound(1);
        TBLPTR = savedTBLPTR;
        return(0xFF);
    }

    TBLPTR += 3;              // Address matched inc TBLPTR to the data value  
    _asm 
    TBLRDPOSTDEC
    _endasm
    latchData = TABLAT;       // Read and return data value

    TBLPTR = savedTBLPTR;
    return(latchData);   

                             // Returns FFh if no valid EEAddr was found or no data in active page
}

/************************************************************************
DataEEWrite

This routine verifies the address is valid. If not, the Illegal Address
flag is set and an error code is returned. It then finds the active page. 
If an active page can not be found, the Page Corrupt status bit is set 
and an error code is returned. A read is performed, if the data was not
changed, the function exits. If the last location is programmed, the Pack
Skipped error flag is set (one location should always be available). The
data EE information (MSB = data, LSB = addresss) is programmed and 
verified. If the verify fails, the Write Error flag is set. If the write
went into the last location of the page, pack is called. This function
can be called by the user.

Parameters:		Data EE data and address
Return:			Pass or fail status (0 = Pass)
Side Effects:	Data EE flags may be updated. CPU stall occurs for flash
                programming. Pack may be generated.
************************************************************************/
unsigned char DataEEWrite(unsigned char data, unsigned int addr)
{
    unsigned short long     savedTBLPTR;        //Context save of TBLPAG value. Current and packed page are on same page.
    unsigned short long     currentPage;
    unsigned int            nextLoc;
    unsigned char           latch;
    unsigned char           dataEEFlags_sh;
    unsigned char           bank;

    savedTBLPTR = TBLPTR;                                  // Save previous value of the TBLPTR

    if((addr >= DATA_EE_TOTAL_SIZE))
    {
        SetPageIllegalAddress(1);
        return(5);
    }

    bank = addr / DATA_EE_SIZE;

    // Find the active page.
    for (currentPage = 0;
         (currentPage < NUM_DATA_EE_PAGES) && 
         (GetPageStatus(bank, currentPage, STATUS_CURRENT) == PAGE_NOT_CURRENT);
         currentPage++) {}        

    if (currentPage == NUM_DATA_EE_PAGES)
    {
        TBLPTR = savedTBLPTR;
        SetPageCorruptStatus(1);
        return(6);      // Error - no active page
    }

    TBLPTR = DEE_TBL_PTR(bank, currentPage);

    dataEEFlags_sh = dataEEFlags.val;

    //Do not write data if it did not change
    if(DataEERead(addr) == data)
    {
        if(GetAddrNotFound() == 0) // Check if the read was successful
        {
            TBLPTR = savedTBLPTR;
            dataEEFlags.val = dataEEFlags_sh;
            return(0);
        }
    }

    dataEEFlags.val = dataEEFlags_sh;               //Restore status flags
    nextLoc = GetNextAvailCount(bank);

    if(!nextLoc)                                    
    {
        TBLPTR = savedTBLPTR;
        SetPagePackSkipped(1);
        return(4);                 
    }

    TBLPTR += nextLoc;

    TABLAT = (addr % DATA_EE_SIZE);
    _asm
    TBLWTPOSTINC
    _endasm
    nextLoc++;
    TABLAT = data;                  
    _asm
    TBLWT
    _endasm

    EECON1 = PROGRAM_WORD;
    UnlockWrite();
    nextLoc++;

    // READ THE DATA I JUST WROTE AND VERIFY 
    _asm
    TBLRDPOSTDEC
    _endasm 
     if(TABLAT != data )
    {
        SetPageWriteError(1);
        return(7);  //Error - RAM does not match PM
    }

    // READ THE ADDRESS I JUST WROTE AND VERIFY 
    _asm
    TBLRD
    _endasm 
     if(TABLAT != (addr % DATA_EE_SIZE) )
    {
        SetPageWriteError(1);
        return(7);  //Error - RAM does not match PM
    }

    //Pack if page has only one location available
    if (nextLoc == NUMBER_OF_INSTRUCTIONS_IN_PAGE*2)
    {
       Nop();
       PackEE(bank);
    }

    TBLPTR = savedTBLPTR;

    return(0);   
}
