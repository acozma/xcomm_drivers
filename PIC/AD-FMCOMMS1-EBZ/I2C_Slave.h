#ifndef _I2C_Slave_h
#define _I2C_Slave_h

void I2C_Slave_Init(unsigned char slave_address);
void I2C_Slave_ISR(void);

#endif