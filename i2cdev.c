/** 
*   @file i2ddev.c
*   @author Andrew Gordon
*   @date 1 Jan 2021 
*
*   @brief I2C user mode interface.
*
*/
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>

#include "i2cdev.h"

#define I2C_BUS_PREFIX  "/dev/i2c-%d"    ///< Prefix to Linux device for I2C.

/**
*   Initialise the I2C bus and get a handle to the device.
*   @param[in] i2c_bus The I2C bus identifer I2C_BUS_0 or I2C_BUS_1.
*   @param[in] dev_addr The address of the I2C device, normally expressed as hex (e.g. 0x68)
*   @return The handle to I2C device or -1 for error.
*/
int16_t
i2cdev_init(i2c_bus_t i2c_bus, uint8_t dev_addr)
{
    int16_t i2cdev_h;                   ///< Handle to I2C device, -1 for error.
    char i2cbus_blk[11];                ///< Handle to I2C bus.
    int16_t i2cdev_ioctl;               ///< Handle to I2C device initialisaion.

    // Concatenate I2C device prefix and bus no.
    //
    sprintf(i2cbus_blk,I2C_BUS_PREFIX,i2c_bus);
    // Attempt to open a connection to the I2C bus.
    //
    if ((i2cdev_h = open(i2cbus_blk,O_RDWR)) < 0)
    {
        // Error
        //
        return i2cdev_h;
    }
    // Attempt to connect to device.
    //
    if ((i2cdev_ioctl = ioctl(i2cdev_h,I2C_SLAVE,dev_addr)) < 0)
    {
        // Error
        //
        return i2cdev_ioctl; 
    }

    return i2cdev_h;
}

/**
*   Read single bit from an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] bit_num Bit position to read (0 to 7).
*   @param[out] data Container for byte value read from device.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_readbit(int16_t i2cdev_h, uint8_t reg_addr, uint8_t bit_num, uint8_t *data)
{
    uint8_t buffer[1];

    // Read the byte at the register
    //
    if (i2cdev_readbyte(i2cdev_h, reg_addr, buffer) == ERROR)
    {
        // Error
        //
        return ERROR;
    }
    // Mask the bit positon
    //
    *data = buffer[0] & (1 << bit_num);
    return OK;
}

/**
*   Read multiple bits from an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] bit_start Bit position to start read (0 to 7).
*   @param[in] length Number of bits to read (max of 8)
*   @param[out] data Container for byte value read from device.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_readbits(int16_t i2cdev_h, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t *data)
{
    uint8_t buffer[1];
    uint8_t read_byte;
    uint8_t mask;

    // Check bit start - length is greater than 0
    //
    if ((bit_start - length) > 0)
    {
        return ERROR;
    }

    // Read the whole byte at the register
    //
    if (i2cdev_readbyte(i2cdev_h, reg_addr, buffer) == ERROR)
    {
        return ERROR;
    }
    read_byte = buffer[0];
    // Extract the bits by mask
    //
    mask = ((1 << length) - 1) << (bit_start - length + 1);
    read_byte &= mask;
    read_byte >>= (bit_start - length + 1);
    *data = read_byte;
    return OK;
}

/**
*   Read single byte from an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[out] data Container for byte value read from device.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_readbyte(int16_t i2cdev_h, uint8_t reg_addr, uint8_t *data)
{
    uint8_t buffer[1];                      ///< Write / read buffer

    buffer[0] = reg_addr;
    // Attempt to set register to read
    //
    if (write(i2cdev_h, buffer, 1) != 1)
    {
        return ERROR;
    }
    // Attempt to read byte at register.
    //
    if (read(i2cdev_h, buffer, 1) != 1)
    {
        return ERROR;
    }
    data[0] = buffer[0];
    return OK;
}

/**
*   Read multiple bytes from an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] length Number of bytes to read.
*   @param[out] data Container for byte value read from device.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_readbytes(int16_t i2cdev_h, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
    uint8_t i;
    uint8_t buffer[32];                      ///< Write / read buffer, assumes 32 bytes max.

    // Check number of bytes to read is no greater than 32
    //
    if (length > 32)
    {
        return ERROR;
    }

    buffer[0] = reg_addr;
    // Attempt to set register to read
    //
    if (write(i2cdev_h, buffer, 1) != 1)
    {
        return ERROR;
    }
    // Attempt to read bytes at register.
    //
    if (read(i2cdev_h, buffer, length) != length)
    {
        return ERROR;
    }
    // Set the return data buffer
    //
    for (i = 0; i < length; i++)
    {
        data[i] = buffer[i];
    }
    return OK;
}

/**
*   Write single bit to an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] bit_num Bit position to write (0 to 7).
*   @param[in] data Byte value.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_writebit(int16_t i2cdev_h, uint8_t reg_addr, uint8_t bit_num, uint8_t data)
{
    uint8_t buffer[1];
    uint8_t read_byte;

    // Attempt to read byte at register.
    //
    if (i2cdev_readbyte(i2cdev_h, reg_addr, buffer) == ERROR)
    {
        return ERROR;
    }
    read_byte = buffer[0];
    // Reset the byte with the write bit mask
    //
    read_byte = (data != 0) ? (read_byte | (1 << bit_num)) : (read_byte & ~(1 << bit_num));
    // Attempt to write the byte back to the register.
    //
    if (i2cdev_writebyte(i2cdev_h, reg_addr, read_byte) == ERROR)
    {
        return ERROR;
    }
    else 
    {
        return OK;
    }
}

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
i2cdev_writebits(int16_t i2cdev_h, uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data)
{
    uint8_t buffer[1];
    uint8_t read_byte;
    uint8_t mask;

    // Check bit start - length is greater than 0
    //
    if ((bit_start - length) > 0)
    {
        return ERROR;
    }

    // Read the whole byte at the register
    //
    if (i2cdev_readbyte(i2cdev_h, reg_addr, buffer) == ERROR)
    {
        return ERROR;
    }
    read_byte = buffer[0];
    mask = ((1 << length) - 1) << (bit_start - length + 1);
    //
    // shift data into correct position
    data <<= (bit_start - length + 1); 
    //
    // zero all non-important bits in data
    data &= mask; 
     //
    // zero all important bits in existing byte
    read_byte &= ~(mask);
    //
    // combine data with existing byte
    read_byte |= data;
    if (i2cdev_writebyte(i2cdev_h, reg_addr, read_byte) == ERROR)
    {
        return ERROR;
    }
    else
    {
        return OK;
    }    
}


/**
*   Write single byte to an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] data Byte value.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_writebyte(int16_t i2cdev_h, uint8_t reg_addr, uint8_t data)
{
    uint8_t buffer[2];

    // Set the register and data values.
    //
    buffer[0] = reg_addr;
    buffer[1] = data;
    
    // Attempt to write to the device.
    //
    if (write(i2cdev_h, buffer, 2) != 2)
    {
        return ERROR; 
    }
    else
    {
        return OK;
    }    
}

/**
*   Write multiple bytes to an 8-bit device register.
*   @param[in] i2cdev_h The handle to the I2C device.
*   @param[in] reg_addr The address of the I2C device register, normally expressed as hex (e.g. 0x68)
*   @param[in] length Mumner of bytes to write.
*   @param[in] data Byte value.
*   @return Status of the operation, -1 for error.
*/
int8_t
i2cdev_writebytes(int16_t i2cdev_h, uint8_t reg_addr, uint8_t length, uint8_t *data)
{
    uint8_t i;
    uint8_t buffer[33];

    // Check number of bytes is no greater than 32.
    //
    if (length > 32)
    {
        return ERROR;
    }
    // Set the register.
    //
    buffer[0] = reg_addr;
    // Set the remaining buffer to the data.
    //
    for (i = 0; i < length; i++)
    {
        buffer[i+1] = data[i];
    }
    // Write to the device.
    //
    if (write(i2cdev_h, buffer, length + 1) != (length + 1))
    {
        return ERROR; 
    }
    else
    {
        return OK;
    } 
 }