# I2C Slave

## Description

* This I2C slave has an address of `0x20`

* WHO_AM_I has an address of `0x10`. When read, gives out `0xEE` 
* OUT_X_L has an address of `0x21`. When read, gives out the x-axis linear acceleration (lower bits)
* OUT_X_H has an address of `0x22`. When read, gives out the x-axis linear acceleration (higher bits)
* OUT_Y_L has an address of `0x23`. When read, gives out the x-axis linear acceleration (lower bits)
* OUT_Y_H has an address of `0x24`. When read, gives out the y-axis linear acceleration (higher bits)
* OUT_Z_L has an address of `0x25`. When read, gives out the y-axis linear acceleration (lower bits)
* OUT_Z_H has an address of `0x26`. When read, gives out the x-axis linear acceleration (higher bits)
* OUT_TEMP_L has an address of `0x25`. When read, gives out temperature reading (lower bits)
* OUT_TEMP_H has an address of `0x26`. When read, gives out temperature readingn (higher bits)

## How to access

The slave and its registers can be conveniently accessed by using HAL. Example of reading WHO_AM_I
```
HAL_I2C_Mem_Read(&hi2c1, (I2C_SLAVE_ADDR << 1) , 0x10, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 100);
```

## How to setup
* This was tested on a Nucleo F302R8 board and a Discovery board as I2C master
* Connect SCL at PB8 and SDA at PB9


