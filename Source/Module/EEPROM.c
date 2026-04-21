/******************** (C) COPYRIGHT 2021 SONiX *******************************
* COMPANY:	SONiX
* DATE:			2023/11
* AUTHOR:		SA1
* IC:				SN32F400
*****************************************************************************/

/*_____ I N C L U D E S ____________________________________________________*/
#include "EEPROM.h"
#include "..\Driver\I2C.h"

/*_____ D E C L A R A T I O N S ____________________________________________*/

/*_____ D E F I N I T I O N S ______________________________________________*/

/*_____ M A C R O S ________________________________________________________*/

/*_____ F U N C T I O N S __________________________________________________*/

/*****************************************************************************
* Function		: eeprom_write
* Description	: write data to eeprom
* Input			: dev_addr: device address (7-bit, e.g., 0x50 for 24Cxx)
*                 mem_addr: memory address inside EEPROM
*                 *dat: data to write
*                 length: number of bytes
* Note			: dev_addr should be 0xA0 for 24C02 on SN32F407 board
*****************************************************************************/
void eeprom_write(uint8_t dev_addr, uint8_t mem_addr, uint8_t *dat, uint16_t length)
{
    /* dev_addr dã bao g?m bit Write (0) ? cu?i, ví d? 0xA0 = 10100000 */
    I2C0_Start();
    
    if(I2C_write_byte(dev_addr) == I2C_NACK_FALG)  /* G?i device address + Write */
    {
        I2C0_Stop();
        return;
    }
    
    if(I2C_write_byte(mem_addr) == I2C_NACK_FALG)  /* G?i memory address */
    {
        I2C0_Stop();
        return;
    }
    
    while(length--)
    {
        if(I2C_write_byte(*dat++) == I2C_NACK_FALG)  /* G?i data */
        {
            I2C0_Stop();
            return;
        }
    }
    I2C0_Stop();
}

/*****************************************************************************
* Function		: eeprom_read
* Description	: read data from eeprom
* Input			: dev_addr: device address (7-bit, e.g., 0x50 for 24Cxx)
*                 mem_addr: memory address inside EEPROM
*                 *dat: buffer to store read data
*                 length: number of bytes to read
* Note			: dev_addr should be 0xA1 for read operation
*****************************************************************************/
void eeprom_read(uint8_t dev_addr, uint8_t mem_addr, uint8_t *dat, uint16_t length)
{
    /* ===== CÁCH ÐÚNG: ===== */
    /* Bu?c 1: G?i device address + Write bit (0) d? set memory pointer */
    I2C0_Start();
    
    if(I2C_write_byte(dev_addr & 0xFE) == I2C_NACK_FALG)  /* Force Write bit = 0 */
    {
        I2C0_Stop();
        return;
    }
    
    if(I2C_write_byte(mem_addr) == I2C_NACK_FALG)  /* G?i memory address */
    {
        I2C0_Stop();
        return;
    }
    
    /* Bu?c 2: Repeat start + g?i device address + Read bit (1) */
    I2C0_Start();
    
    if(I2C_write_byte(dev_addr | 0x01) == I2C_NACK_FALG)  /* Set Read bit = 1 */
    {
        I2C0_Stop();
        return;
    }
    
    /* Bu?c 3: Ð?c data */
    while(length > 1)
    {
        *dat++ = I2C_read_byte(I2C_ACK_FALG);   /* ACK cho byte cu?i */
        length--;
    }
    
    *dat = I2C_read_byte(I2C_NACK_FALG);        /* NACK cho byte cu?i cùng */
    I2C0_Stop();
}