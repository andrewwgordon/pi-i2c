/** 
*   @file i2ddev.h
*   @author Andrew Gordon
*   @date 1 Jan 2021 
*
*   @brief I2C user mode interface.
*
*/
#ifndef _I2CDEV_H
#define _I2CDEV_H

#include <stdint.h>

#define OK      (1)
#define ERROR   (-1)

/**
 *  @brief i2c bus, either 0 or 1.
 *
 */
typedef enum i2c_bus_enum
{
    I2C_BUS_0,              ///< equates to linux /dev/i2c-0
    I2C_BUS_1               ///< equates to linux /dev/i2c-1
} i2c_bus_t;

/**
*   Initialise the I2C bus and get a handle to the device.
*   @param[in] i2c_bus The I2C bus identifer I2C_BUS_0 or I2C_BUS_1.
*   @param[in] dev_addr The address of the I2C device, normally expressed as hex (e.g. 0x68)
*   @return The handle to I2C device or -1 for error.
*/
int16_t
i2cdev_init(i2c_bus_t i2c_bus, uint8_t dev_addr);

/**
*   Read single bit from an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] bit_num Bit position to read (0 to 7).
*   @param[out] data Container for byte value read from device.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_readbit(int16_t i2cdev_h, uint8_t reg_addr, uint8_t bit_num, uint8_t *data);

/**
*   Read multiple bits from an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] bit_start Bit position to start read (0 to 7).
*   @param[in] length Number of bits to read.
*   @param[out] data Container for byte value read from device.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_readbits(int16_t i2cdev_h, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t *data);

/**
*   Read single byte from an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[out] data Container for byte value read from device.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_readbyte(int16_t i2cdev_h, uint8_t reg_addr, uint8_t *data);

/**
*   Read multiple bytes from an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] length Number of bytes to read.
*   @param[out] data Container for byte value read from device.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_readbytes(int16_t i2cdev_h, uint8_t reg_addr, uint8_t length, uint8_t *data);

/**
*   Write single bit to an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] bit_num Bit position to write (0 to 7).
*   @param[in] data Byte value.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_writebit(int16_t i2cdev_h, uint8_t reg_addr, uint8_t bit_num, uint8_t data);

/**
*   Write multiple bits to an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] bit_start Bit position to write (0 to 7).
*   @param[in] length Number of bits to write (not more than 8).
*   @param[in] data Byte value.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_writebits(int16_t i2cdev_h, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);

/**
*   Write single byte to an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] data Byte value.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_writebyte(int16_t i2cdev_h, uint8_t reg_addr, uint8_t data);

/**
*   Write multiple bytes to an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] length Mumner of bytes to write.
*   @param[in] data Byte value.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_writebytes(int16_t i2cdev_h, uint8_t reg_addr, uint8_t length, uint8_t *data);

/**
*   Write a word to an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] data 16-bit word.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_writeword(int16_t i2cdev_h, uint8_t reg_addr,uint16_t data);
#endif