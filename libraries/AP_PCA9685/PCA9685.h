/*  PCA9685 library for ArduCopter
    Copyright (C) 2015 Sgw32   

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef PCA9685_H
#define PCA9685_H

#include <AP_Common.h>
#include <AP_HAL.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <GCS_MAVLink.h>
#include <AP_Notify.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>


#define PCA9685_I2C_ADDRESS 0x67
#define PCA9685_I2C_ADDRESS2 0x38
#define PCA9685_I2C_BASE_ADDRESS 0x40

#define PCA9685_RESTART 0x80 // 0b10000000
#define PCA9685_EXTCLK 0x40 // 0b01000000
#define PCA9685_AI 0x20 // 0b00100000
#define PCA9685_SLEEP 0x10 // 0b00010000
#define PCA9685_SUB1 0x8 // 0b00001000
#define PCA9685_SUB2 0x4 // 0b00000100
#define PCA9685_SUB3 0x2 // 0b00000010
#define PCA9685_ALLCALL 0x1 // 0b00000001

//MODE 2
#define PCA9685_INVRT 0x10 // 0b00010000
#define PCA9685_OCH 0x8 // 0b00001000
#define PCA9685_OUTDRV 0x4 // 0b00000100
#define PCA9685_OUTNE1 0x2 // 0b00000010
#define PCA9685_OUTNE2 0x1 // 0b00000001

#define PCA9685_PRESCALER 0xFE
#define PCA9685_MODE1 0x00
#define PCA9685_MODE2 0x01

#define CALC_FREQ(x) 25000000/4096/x-1

#define LED_ON_L(x) 0x6+4*x
#define LED_ON_H(x) 0x7+4*x
#define LED_OFF_L(x) 0x8+4*x
#define LED_OFF_H(x) 0x9+4*x

#define LOW_PART(x) x&0xFF
#define HIGH_PART(x) x>>8

extern const AP_HAL::HAL& hal;

class AeroxoTiltrotorPCA9685
{
public:
	AeroxoTiltrotorPCA9685(){}
	~AeroxoTiltrotorPCA9685(){}

	uint8_t writeRegg0(uint8_t reg,uint8_t data)
	{
	  if (hal.i2c->writeRegister(PCA9685_I2C_ADDRESS , reg,data)!=0)
	  {
	   // hal.console->println("PCA9685 I2C connection error.");
	  }
	  return 0;
	}

	void initPCA9685()
	{
	  writeRegg0(PCA9685_MODE1,0x10);
	  //uint32_t fr = 500000/4096-1;
	  writeRegg0(PCA9685_PRESCALER,133);
	  writeRegg0(PCA9685_MODE1,0x1);
	  writeRegg0(PCA9685_MODE2,0x14&(~PCA9685_INVRT));

	  for (uint8_t cn=0;cn!=16;cn++)
	  {
	  writeRegg0(LED_ON_L(cn),0);
	  writeRegg0(LED_ON_H(cn),0);
	  }

	}

	void setPWM(uint8_t num, uint16_t on,uint16_t off)
	{
	  writeRegg0(LED_ON_L(num),LOW_PART(on));
	  writeRegg0(LED_ON_H(num),HIGH_PART(on));
	  writeRegg0(LED_OFF_L(num),LOW_PART(off));
	  writeRegg0(LED_OFF_H(num),HIGH_PART(off));
	}

	void setServo(uint8_t num, uint16_t pwm) //angle from 1000 to 2000
	{
	  // 4096 - 20 msecs
	  // 4096/20 - 1 msec
	  // 4096/10 - 2 msec
	  uint32_t pwm2d = 4096*((uint32_t)pwm);
	  pwm2d/=20000;
	  //optimize
	  //hal.console->printf("PWM2: %lu",pwm2d);
	  //UNCOMMENT PLS
	  writeRegg0(LED_OFF_L(num),LOW_PART(pwm2d));
	  writeRegg0(LED_OFF_H(num),HIGH_PART(pwm2d));
	}

};

/*uint8_t writeRegg0(uint8_t reg,uint8_t data)
{
  if (hal.i2c->writeRegister(PCA9685_I2C_BASE_ADDRESS , reg,data)!=0)
  {
   // hal.console->println("PCA9685 I2C connection error.");
  }
  return 0;
}

void initPCA9685()
{
  writeRegg0(PCA9685_MODE1,0x10);
  //uint32_t fr = 500000/4096-1;
  writeRegg0(PCA9685_PRESCALER,133);
  writeRegg0(PCA9685_MODE1,0x1);
  writeRegg0(PCA9685_MODE2,0x14&(~PCA9685_INVRT));

  for (uint8_t cn=0;cn!=16;cn++)
  {
  writeRegg0(LED_ON_L(cn),0);
  writeRegg0(LED_ON_H(cn),0);
  }

}

void setPWM(uint8_t num, uint16_t on,uint16_t off)
{
  writeRegg0(LED_ON_L(num),LOW_PART(on));
  writeRegg0(LED_ON_H(num),HIGH_PART(on));
  writeRegg0(LED_OFF_L(num),LOW_PART(off));
  writeRegg0(LED_OFF_H(num),HIGH_PART(off));
}

void setServo(uint8_t num, uint16_t pwm) //angle from 1000 to 2000
{
  // 4096 - 20 msecs
  // 4096/20 - 1 msec
  // 4096/10 - 2 msec
  uint32_t pwm2d = 4096*((uint32_t)pwm);
  pwm2d/=20000;
  //optimize
  //hal.console->printf("PWM2: %lu",pwm2d);
 // writeRegg2(LED_ON_L(num),LOW_PART(0));
  //writeRegg2(LED_ON_H(num),HIGH_PART(0));

  //UNCOMMENT PLS
  //writeRegg2(LED_OFF_L(num),LOW_PART(pwm2d));
  //writeRegg2(LED_OFF_H(num),HIGH_PART(pwm2d));
}*/

#endif 
