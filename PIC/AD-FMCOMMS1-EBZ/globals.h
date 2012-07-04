
// Enable one or the other
#define RUNNING_ON_EVAL_BOARD
//#define RUNNING_ON_PICDEV

//I2CDET#define LEGACY_I2C

#define bUSBIF 0x10

// OSCTUNE
#define bINTSRC		0x80
#define bPLLEN		0x40

// UCON
#define bSUSPND		0x02
#define bRESUME		0x04
#define bUSBEN		0x08
#define bPKTDIS		0x10
#define bSEO		0x20
#define bPPBRST		0x40

// UCFG
#define bPPB0		0x01
#define bPPB1		0x02
#define bFSEN		0x04
#define bUTRDIS		0x08
#define bUPUEN		0x10
#define bUOEMON		0x40
#define bUTEYE		0x80

// UEPn
#define bEPSTALL	0x01
#define bEPINEN		0x02
#define bEPOUTEN	0x04
#define bEPCONDIS	0x08
#define bEPHSHK		0x10

// UIR
#define bURSTIF 	0x01
#define bUERRIF 	0x02
#define bACTVIF 	0x04
#define bTRNIF		0x08
#define bIDLEIF		0x10
#define bSTALLIF	0x20
#define bSOFIF		0x40

// USTAT
#define bPPBI		0x02
#define bDIR		0x04
#define bENDP		0x78

// BDnSTAT (Write)
#define bBC8		0x01
#define bBC9		0x02
#define bBSTALL		0x04
#define bDTSEN		0x08
#define bDTS		0x40
#define bUOWN		0x80

// BDnSTAT (Read)
#define PID0		0x04
#define PID1		0x08
#define PID2		0x10
#define PID3		0x20

// Buffer Descriptor 0 registers
#define BD0STAT (*((unsigned char*)0x0400))
#define BD0CNT *((unsigned char*)0x0401)
#define BD0ADRL *((unsigned char*)0x0402)
#define BD0ADRH *((unsigned char*)0x0403)

// Buffer Descriptor 1 registers
#define BD1STAT *((unsigned char*)0x0404)
#define BD1CNT *((unsigned char*)0x0405)
#define BD1ADRL *((unsigned char*)0x0406)
#define BD1ADRH *((unsigned char*)0x0407)

// Buffer Descriptor 2 registers
#define BD2STAT *((unsigned char*)0x0408)
#define BD2CNT *((unsigned char*)0x0409)
#define BD2ADRL *((unsigned char*)0x040A)
#define BD2ADRH *((unsigned char*)0x040B)

// Buffer Descriptor 3 registers
#define BD3STAT *((unsigned char*)0x040C)
#define BD3CNT *((unsigned char*)0x040D)
#define BD3ADRL *((unsigned char*)0x040E)
#define BD3ADRH *((unsigned char*)0x040F)

// USB Buffers
#define B0DATA ((unsigned char*)0x0500)
#define B1DATA ((unsigned char*)0x0540)
#define B2DATA ((unsigned char*)0x0580)
#define B3DATA ((unsigned char*)0x05C0)