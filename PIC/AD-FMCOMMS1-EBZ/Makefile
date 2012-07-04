# MPLAB IDE generated this makefile for use with GNU make.
# Project: AD-FMCOMMS1-EBZ.mcp
# Date: Tue Jul 03 10:03:15 2012

AS = MPASMWIN.exe
CC = mcc18.exe
LD = mplink.exe
AR = mplib.exe
RM = rm

AD-FMCOMMS1-EBZ.cof : descriptors.o I2C_Slave.o main.o SPI_Master.o USB.o DEE\ Emulation\ 8-bit.o NoFilDEE.o
	$(LD) /p18F24J50 /l"C:\MCC18\lib" "18f24j50.lkr" "descriptors.o" "I2C_Slave.o" "main.o" "SPI_Master.o" "USB.o" "DEE Emulation 8-bit.o" "NoFilDEE.o" /u_CRUNTIME /z__MPLAB_BUILD=1 /o"AD-FMCOMMS1-EBZ.cof" /M"AD-FMCOMMS1-EBZ.map" /W

descriptors.o : descriptors.c descriptors.h
	$(CC) -p=18F24J50 "descriptors.c" -fo="descriptors.o" -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

I2C_Slave.o : I2C_Slave.c I2C_Slave.h SPI_Master.h DEE\ Emulation\ 8-bit.h I2C_Slave.c C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18cxxx.h C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18f24j50.h globals.h descriptors.h NoFilDEE.inc
	$(CC) -p=18F24J50 "I2C_Slave.c" -fo="I2C_Slave.o" -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

main.o : main.c I2C_Slave.h DEE\ Emulation\ 8-bit.h main.c C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18cxxx.h C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18f24j50.h globals.h USB.h NoFilDEE.inc
	$(CC) -p=18F24J50 "main.c" -fo="main.o" -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

SPI_Master.o : SPI_Master.c SPI_Master.h SPI_Master.c C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18cxxx.h C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18f24j50.h globals.h
	$(CC) -p=18F24J50 "SPI_Master.c" -fo="SPI_Master.o" -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

USB.o : USB.c SPI_Master.h USB.c C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18cxxx.h C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18f24j50.h globals.h USB.h descriptors.h
	$(CC) -p=18F24J50 "USB.c" -fo="USB.o" -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

DEE\ Emulation\ 8-bit.o : DEE\ Emulation\ 8-bit.c DEE\ Emulation\ 8-bit.h DEE\ Emulation\ 8-bit.c C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18cxxx.h C:/Program\ Files/Microchip/mplabc18/v3.42/h/p18f24j50.h GenericTypeDefs.h NoFilDEE.inc
	$(CC) -p=18F24J50 "DEE Emulation 8-bit.c" -fo="DEE Emulation 8-bit.o" -Ou- -Ot- -Ob- -Op- -Or- -Od- -Opa-

NoFilDEE.o : NoFilDEE.asm NoFilDEE.inc
	"C:\Program Files\Microchip\mplabc18\v3.42\mpasm\mpasmwin.exe" /q /p18F24J50 "NoFilDEE.asm" /l"NoFilDEE.lst" /e"NoFilDEE.err" /o"NoFilDEE.o"

clean : 
	$(RM) "descriptors.o" "I2C_Slave.o" "main.o" "SPI_Master.o" "USB.o" "DEE Emulation 8-bit.o" "NoFilDEE.o" "NoFilDEE.err" "NoFilDEE.lst" "AD-FMCOMMS1-EBZ.cof" "AD-FMCOMMS1-EBZ.hex"

