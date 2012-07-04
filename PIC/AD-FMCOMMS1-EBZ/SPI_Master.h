#ifndef _SPI_Master_h
#define _SPI_Master_h

void SPI_Init(unsigned short settings, unsigned short chip_selects);
void AssertChipSelects(int chip_select_map, int state);
void SPI_StartTxDMA(unsigned short BufferAddress, int length);
void SPI_StartRxDMA(unsigned short BufferAddress);

#endif
