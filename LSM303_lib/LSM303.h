/*
 * LSM303.h
 *
 *  Created on: 7 Jul 2023
 *  Author    : Andri Setyabudi
 */

#ifndef LSM303_H_
#define LSM303_H_

#include "stdio.h"
#include "inttypes.h"
#include "stdlib.h"
#include <math.h>

#define LSM303_Success				0x00
#define LSM303_Failed				0x01

#define LSM303_DEFAULT_ADDR			0x1D
#define D_SA0_HIGH_ADDRESS			0x1D
#define D_SA0_LOW_ADDRESS           0x1E

 #define LSM303_ADDRESS_ACCEL          (0x32 >> 1)         // 0011001x
 #define LSM303_ADDRESS_MAG            (0x3C >> 1)         // 0011110x

typedef enum LSM303_default_reg_val{
	WHO_ID				=	0x49,
	CTRL_REG1_DEFAULT	= 	0x57,	//ODR = 100Hz,	enable axis = X, Y, Z
	CTRL_REG2_DEFAULT	= 	0x00,	// 2g , 773Hz anti-aliasing, self-test disable, I2C mode
	CTRL_REG3_DEFAULT	=  	0x00,	// all interrupt disable
	CTRL_REG4_DEFAULT	= 	0x00,	// all interrupt disable
	CTRL_REG5_DEFAULT	= 	0x64,	// Magnetic Hi-resolution, ODR = 6.25
	CTRL_REG6_DEFAULT	= 	0x20,	// MFS = 01 (+/- 4 gauss full scale)
	CTRL_REG7_DEFAULT	= 	0x00,	// Continuous Conversion
}LSM303_default_val;


typedef enum LSM303_Reg_t{
	WHO_I_AM_REG	=	0x0f,

	CTRL_REG1		=   0x20,
	CTRL_REG2		= 	0x21,
	CTRL_REG3 		=	0x22,
	CTRL_REG4		=	0x23,
	CTRL_REG5 		= 	0x24,
	CTRL_REG6		= 	0x25,
	CTRL_REG7		=	0x26,

	STATUS_REG_A 	= 	0x27,
	OUT_X_L_A 		= 	0x28,
	OUT_X_H_A 		=	0x29,
	OUT_Y_L_A 		=	0x2A,
	OUT_Y_H_A 		=	0x2B,
	OUT_Z_L_A 		=	0x2C,
	OUT_Z_H_A 		=	0x2D,

	INT1_CFG_A 		= 	0x30,
	INT1_SOURCE_A 	=	0x31,
	INT1_THS_A 		=	0x32,
	INT1_DURATION_A =	0x33,
	INT2_CFG_A 		=	0x34,
	INT2_SOURCE_A 	=	0x35,
	INT2_THS_A 		=	0x36,
	INT2_DURATION_A =	0x37,

	OUT_X_L_M 		= 	0x08,
	OUT_X_H_M 		= 	0x09,
	OUT_Y_L_M 		= 	0x0A,
	OUT_Y_H_M 		= 	0x0B,
	OUT_Z_L_M 		= 	0x0C,
	OUT_Z_H_M 		= 	0x0D,
}LSM303_REG;

typedef struct lsm303_var_t{
		uint8_t address;
		uint8_t address_accel, address_mag;
		uint8_t device_id;
		uint8_t (*i2c_read) (uint8_t* data, uint16_t length, uint8_t slave_address);
		uint8_t (*i2c_write)(uint8_t* data, uint16_t length, uint8_t slave_address);

		uint8_t (*init)(uint8_t address, uint8_t (*i2c_read)(uint8_t*,uint16_t,uint8_t),uint8_t (*i2c_write)(uint8_t*,uint16_t,uint8_t));
		uint8_t (*checkDevice)(void);

		uint8_t (*readAccel)(uint16_t *accel_x, uint16_t *accel_y, uint16_t *accel_z);
		uint8_t (*readMag) (uint16_t *axis_x, uint16_t *axis_y, uint16_t *accel_z);

}lsm303_var;

extern lsm303_var LSM303;

#endif /* LSM303_H_ */
