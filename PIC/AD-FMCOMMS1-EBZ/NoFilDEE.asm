;*************************************************************************
;* Software License Agreement
;*
;* Copyright © 2007 Microchip Technology Inc.  All rights reserved.
;*
;* Microchip licenses to you the right to use, modify, copy and distribute 
;* Software only when embedded on a Microchip microcontroller or digital 
;* signal controller, which is integrated into your product or third party 
;* product (pursuant to the sublicense terms in the accompanying license 
;* agreement).  
;*
;* You should refer to the license agreement accompanying this Software for 
;* additional information regarding your rights and obligations.
;*
;* SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS” WITHOUT WARRANTY OF ANY 
;* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
;* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
;* PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE 
;* LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
;* CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY 
;* DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY 
;* INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST 
;* PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, 
;* SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO 
;* ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
;*
;* Author        Date        Comment
;*************************************************************************
;* S. Cowden    2007/05/24  Version 1.0.0 - Initial Release
;*************************************************************************


#include "NoFilDEE.inc"


	list r = dec

Emulation_PageSCN  	CODE    EMULATION_PAGES_START_ADDRESS
Emulation_Page  	RES     DATA_EE_BANKS * NUM_DATA_EE_PAGES * (NUMBER_OF_INSTRUCTIONS_IN_PAGE*2)
            		GLOBAL  Emulation_Page

	END
