/*
 * dri_dm9000a.h
 *
 *  Created on: 2015-3-30
 *      Author: Administrator
 */

#ifndef DRI_DM9000A_H_
#define DRI_DM9000A_H_


#include <cdefbf609.h>
#include <ccblkfn.h>
#include <services/gpio/adi_gpio.h>

/*
 * DM9000A some REGs
 */
#define NCR     0X00 //Network Control Register
#define NSR 	0x01 //Network Status Register

#define TCR		0x02 //TX Control Register
#define TSR1	0x03 //TX Status Register I
#define TSR2	0x04 //TX Status Register II

#define RCR		0x05 //RX Control Register
#define RSR		0x06 //RX Status Register
#define ROCR    0X07 //Receive Overflow Counter Register

#define BPTR    0X08 //Back Pressure Threshold Register
#define FCTR    0X3A //Flow Control Threshold Register
#define SMCR    0X2F //Special Mode Control Register

#define EPCR    0X0B //EEPROM & PHY Control Register
#define EPAR    0X0C //EEPROM & PHY Control Register
#define EPDRL   0X0D //EEPROM & PHY Control Register
#define EPDRH   0X0E //EEPROM & PHY Control Register
#define WUCR    0x0F //wake up control register.
#define PAR_BASE 0X10 //MAC address base ADDR
#define MAR_BASE 0X16 //Multi-cast address base ADDR

#define GPCR	0x1E //General Purpose Control Register
#define GPR		0x1f //General Purpose Register

#define ETCSR   0x30 //Early Transmit Control/Status Register
#define MRCMDX	0xf0 //Memory Data Pre-Fetch Read Command Without Address Increment Register
#define MRCMDX1	0xf1 //Memory Data Read Command With Address Increment Register
#define MRCMD	0xf2 //Memory Data Read Command With Address Increment Register

#define MRRL    0xf4 //Memory Data Read_ address Register Low Byte
#define MRRH    0xf5 //Memory Data Read_ address Register High Byte

#define MWCMDX	0xf6 //Memory Data Write Command Without Address Increment Register
#define MWCMD	0xf8 //Memory Data Write Command With Address Increment Register

#define MWRL    0xfa //Memory Data Write_ address Register Low Byte
#define MWRH    0xfb //Memory Data Write_ address Register High Byte

#define TXPLL	0xfc //TX Packet Length Low Byte Register
#define TXPLH	0xfd //TX Packet Length High Byte Register

#define ISR		0xfe //Interrupt Status Register
#define IMR		0xff //Interrupt Mask Register




void Enable_MAC_INT_Interrupt( bool enable );

//
void DM9000A_Init(uint8_t* srcMac );

int ExEthSend(void*buffer, int len);

void* ExEthRecv(void);

#endif /* DRI_DM9000A_H_ */
