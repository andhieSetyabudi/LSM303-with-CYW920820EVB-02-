/*
 * LSM303.c
 *
 *  Created on: 7 Jul 2023
 *  Author: Andri Setyabudi
 */

#include "LSM303.h"

static uint8_t read_data(uint8_t reg, uint8_t *pData, uint16_t len)
{
	if(LSM303.i2c_write != NULL && LSM303.i2c_read != NULL)
	{
		if( LSM303.i2c_write(&reg, 1u,LSM303.address&0x7f) == LSM303_Success )
			return LSM303.i2c_read(pData, len,LSM303.address);
	}
	return LSM303_Failed;
}

static uint8_t write_data(uint8_t *pData, uint16_t len)
{
	if(LSM303.i2c_write != NULL && LSM303.i2c_read != NULL)
		LSM303.i2c_write(pData, len,LSM303.address&0x7f);
	return LSM303_Failed;
}


static uint8_t who_i_am(void)
{
	LSM303.device_id = 0xff;
	if( read_data((uint8_t)WHO_I_AM_REG, &LSM303.device_id, 1) == LSM303_Success )
	{
		if( LSM303.device_id == WHO_ID )
			return LSM303_Success;
	}
	return LSM303_Failed;
}

static uint8_t lsm303_takeAddress(void)
{
	return 0;
}

static uint8_t lsm303_setDefault(void)
{
	uint8_t def_val[7] = {
			CTRL_REG1_DEFAULT,
			CTRL_REG2_DEFAULT,
			CTRL_REG3_DEFAULT,
			CTRL_REG4_DEFAULT,
			CTRL_REG5_DEFAULT,
			CTRL_REG6_DEFAULT,
			CTRL_REG7_DEFAULT,
	};
	uint8_t buff = 0;
	uint8_t result = 1;
	for(uint8_t i = 0; i<2; i++)
	{
		result = read_data((uint8_t)(CTRL_REG1 + i),&buff, 1);
		if( result == LSM303_Success ) {
			if( buff != (uint8_t)def_val[i] )
			{
				uint8_t br[2] = {(uint8_t)(CTRL_REG1 + i),def_val[i]};
				result = write_data(br, 2u);
				if( result != LSM303_Success )
					break;
			}
		} else
			break;
	}
	return result;
}

static uint8_t lsm303_readAccelerometer(uint16_t *accel_x, uint16_t *accel_y, uint16_t *accel_z)
{
	uint8_t b_r[6]={0,};
	if(read_data((uint8_t)OUT_X_L_A|0x80,b_r, 6) == LSM303_Success )
	{
		if(accel_x)
			*accel_x = (uint16_t)(b_r[1]<<8 |b_r[0]);
		if(accel_y)
			*accel_y = (uint16_t)(b_r[3]<<8 |b_r[2]);
		if(accel_z)
			*accel_z = (uint16_t)(b_r[5]<<8 |b_r[4]);
		return LSM303_Success;
	}
	return LSM303_Failed;
}

static uint8_t lsm303_readMagnetometer(uint16_t *accel_x, uint16_t *accel_y, uint16_t *accel_z)
{
	uint8_t b_r[6]={0,};
	if(read_data((uint8_t)OUT_X_H_M|0x80,b_r, 6) == LSM303_Success )
	{
		if(accel_x)
			*accel_x = (uint16_t)(b_r[1]<<8 |b_r[0]);
		if(accel_y)
			*accel_y = (uint16_t)(b_r[3]<<8 |b_r[2]);
		if(accel_z)
			*accel_z = (uint16_t)(b_r[5]<<8 |b_r[4]);
		return LSM303_Success;
	}
	return LSM303_Failed;
}

static uint8_t lsm303_init(uint8_t address, uint8_t (*i2c_read)(uint8_t*,uint16_t,uint8_t),uint8_t (*i2c_write)(uint8_t*,uint16_t,uint8_t))
{
	LSM303.address   = address;
	LSM303.i2c_read  = i2c_read;
	LSM303.i2c_write = i2c_write;
	if (who_i_am() == LSM303_Success )
	{

		return lsm303_setDefault();
	}
	return LSM303_Failed;
}



lsm303_var LSM303 = {
		.address 	= LSM303_DEFAULT_ADDR,
		.device_id	= 0xff,
		.i2c_read 	= NULL,
		.i2c_write 	= NULL,

		.init 			= lsm303_init,
		.checkDevice 	= who_i_am,

		.readAccel = lsm303_readAccelerometer,
		.readMag   = lsm303_readMagnetometer,
};
