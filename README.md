# I2C Slave

## Description

* This I2C slave has an address of `0x20`

* WHO_AM_I has an address of `0x10`. When read, gives out `0xEE` 
* OUT_X has an address of `0x21`. When read, gives out the x-axis linear acceleration
* OUT_Y has an address of `0x22`. When read, gives out the y-axis linear acceleration
* OUT_Z has an address of `0x23`. When read, gives out the z-axis linear acceleration

## How to access

The slave and its registers can be conveniently accessed by using HAL. Example of reading WHO_AM_I
```
HAL_I2C_Mem_Read(&hi2c1, (I2C_SLAVE_ADDR << 1) , 0x10, I2C_MEMADD_SIZE_8BIT, &whoami, 1, 100);
```

## How to setup
* This was tested on a Nucleo F302R8 board
* Connect SCL at PB8 and SDA at PB9


