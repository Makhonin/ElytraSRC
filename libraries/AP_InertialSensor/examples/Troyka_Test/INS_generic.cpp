// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

//
// Simple test for the AP_InertialSensor driver.
//

#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include "LPS331.h"
#include "l3g4200d.h"
#include "lis3mdl.h"
#include "lis331dlh.h"


//const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

uint32_t timer;

#define PCA9685_I2C_ADRESS 0x38
#define PCA9685_I2C_BASE_ADDRESS 0x40
#define PCA9685_MODE1 0x00 // location for Mode1 register address
#define PCA9685_MODE2 0x01 // location for Mode2 reigster address
#define PCA9685_LED0 0x06 // location for start of LED0 registers

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;
LPS331 barometer;
L3G4200D_TWI gyro;
LIS3MDL_TWI compass;
LIS331DLH_TWI accel;

void setup(void)
{
     //uint8_t i2cres=0;
     
    // barometer.setHAL(hal);
    hal.console->println("Elytra test");
    hal.console->println("Initialising sensors...");
	
    hal.scheduler->delay(100);
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();
	
    // take i2c bus sempahore
   if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
       return;
            
    //int i2cadress =  0x5c;  //baro
	
	bool isOnline;
	 
	/*if (hal.i2c->readRegister(i2cadress, 0x0F,&i2cres)!=0)
	{
		hal.console->println("I2C error.");
	}*/
	

	/*if (i2cres==0xbb)	
	{
		isOnline = true;
	} else {
		isOnline = false;
	}*/
	
	if (barometer.autoDetectAddress())
	{
		hal.console->println("Ok!");
		isOnline=true;
	}
	else
	{
		hal.console->println("I2C error.");
		isOnline=false;
	}
	barometer.begin();
	compass.begin();
	compass.setRange(RANGE_4);
	accel.begin();
	//accel.setRange(LIS331RANGE_2);
	gyro.begin();
	gyro.setRange(RANGE_250);
	//i2c_sem->give();
	if (isOnline)
		hal.console->println("Initialisation complete.");
    hal.scheduler->delay(1000);
    timer = hal.scheduler->micros();
    hal.console->println("Started.");
}

void loop(void)
{
    if((hal.scheduler->micros()- timer) > 1000000L)
    {
		compass.readXYZ_Calib();
		float pressure = barometer.readPressureMillibars();
		float temperature = barometer.readTemperatureC();
		
		// printf("%f ",gyro.readX_DegPerSec());
		/*printf("%f ",gyro.readY_DegPerSec());
		printf("%f ",gyro.readZ_DegPerSec());
		*/
		
		hal.console->printf_P("%f %f %f yaw: %f %f %f %f %f %f\n",accel.readX_G(),accel.readY_G(),accel.readZ_G(),compass.read_Yaw(),gyro.readX_DegPerSec(),gyro.readY_DegPerSec(),gyro.readZ_DegPerSec(),pressure,temperature);
        timer = hal.scheduler->micros();
        hal.console->println("Ok.");
    } else {
	    hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();
