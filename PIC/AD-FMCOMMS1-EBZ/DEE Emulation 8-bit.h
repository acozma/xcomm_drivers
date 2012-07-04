/*************************************************************************
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
* Author        Date        Comment
*************************************************************************
* S. Cowden    2007/05/24  Version 1.0.0 - Initial Release
************************************************************************/


/* The NoFilDEE.inc file contains the following #defines  
EMULATION_PAGES_START_ADDRESS
DATA_EE_BANKS
NUM_DATA_EE_PAGES
NUMBER_OF_INSTRUCTIONS_IN_PAGE 
DATA_EE_SIZE                   
ERASE_WRITE_CYCLE_MAX
NUMBER_OF_INSTRUCTION_IN_ROW
NUMBER_OF_ROWS_IN_PAGE

ERASE           0x14
PROGRAM_ROW     0x04
PROGRAM_WORD    0x24       Values written to the EECON1 register
*/

#include "NoFilDEE.inc"

#define DATA_EE_TOTAL_SIZE              (DATA_EE_BANKS * DATA_EE_SIZE)
#define PAGE_AVAILABLE                  1
#define PAGE_CURRENT                    0
#define PAGE_EXPIRED                    0
#define PAGE_NOT_AVAILABLE              0
#define PAGE_NOT_CURRENT                1
#define PAGE_NOT_EXPIRED                1
#define STATUS_AVAILABLE                0
#define STATUS_CURRENT                  1
#define STATUS_EXPIRED                  2

#define GetAddrNotFound() dataEEFlags.addrNotFound
#define SetAddrNotFound(x) dataEEFlags.addrNotFound = x

#define GetPageExpiredPage() dataEEFlags.expiredPage
#define SetPageExpiredPage(x) dataEEFlags.expiredPage = x

#define GetPagePackB4PageFull() dataEEFlags.packB4PageFull
#define SetPagePackB4PageFull(x) dataEEFlags.packB4PageFull = x

#define GetPagePackB4Init() dataEEFlags.packB4Init
#define SetPagePackB4Init(x) dataEEFlags.packB4Init = x

#define GetPagePackSkipped() dataEEFlags.packSkipped
#define SetPagePackSkipped(x) dataEEFlags.packSkipped = x

#define GetPageIllegalAddress() dataEEFlags.IllegalAddress
#define SetPageIllegalAddress(x) dataEEFlags.IllegalAddress = x

#define GetPageCorruptStatus() dataEEFlags.pageCorrupt
#define SetPageCorruptStatus(x) dataEEFlags.pageCorrupt = x

#define GetPageWriteError() dataEEFlags.writeError
#define SetPageWriteError(x) dataEEFlags.writeError = x


typedef union
{
    unsigned char val;
    struct
    {
        unsigned addrNotFound:1;            // Return 0xFF
        unsigned expiredPage:1;             // Return 0x1
        unsigned packB4PageFull:1;          // Not a return condition
        unsigned packB4Init:1;              // Return 0x3
        unsigned packSkipped:1;             // Return 0x4
        unsigned IllegalAddress:1;          // Return 0x5
        unsigned pageCorrupt:1;             // Return 0x6
        unsigned writeError:1;              // Return 0x7
    };
} DATA_EE_FLAGS;

extern DATA_EE_FLAGS dataEEFlags;



void            UnlockWrite         (void);
int             GetPageStatus       (unsigned char bank, unsigned char page, unsigned char field);
void            ErasePage           (unsigned char bank, unsigned char page);
char            IncEWCount          (unsigned char *index);
unsigned int    GetNextAvailCount   (unsigned char bank);
int             PackEE              (unsigned char bank);
unsigned char   DataEEInit          (void);
unsigned int    DataEERead          (unsigned int addr);
unsigned char   DataEEWrite         (unsigned char data, unsigned int addr);
