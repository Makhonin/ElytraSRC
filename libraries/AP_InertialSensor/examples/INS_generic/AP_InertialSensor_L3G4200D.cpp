/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
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
/*
  This is an INS driver for the combination L3G4200D gyro and ADXL345 accelerometer.
  This combination is available as a cheap 10DOF sensor on ebay

  This sensor driver is an example only - it should not be used in any
  serious autopilot as the latencies on I2C prevent good timing at
  high sample rates. It is useful when doing an initial port of
  ardupilot to a board where only I2C is available, and a cheap sensor
  can be used.

Datasheets:
  ADXL345 Accelerometer http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf
  L3G4200D gyro http://www.st.com/st-web-ui/static/active/en/resource/technical/document/datasheet/CD00265057.pdf
*/

#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include <AP_Math.h>
#include "AP_InertialSensor_L3G4200D.h"
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <sys/time.h>
#include <sched.h>
#include <linux/rtc.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <sched.h>
#include <linux/rtc.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <sys/types.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <sys/mman.h>

const extern AP_HAL::HAL& hal;

///////
/// Accelerometer ADXL345 register definitions
/*#define ADXL345_ACCELEROMETER_ADDRESS                  0x53
#define ADXL345_ACCELEROMETER_XL345_DEVID              0xe5
#define ADXL345_ACCELEROMETER_ADXLREG_BW_RATE          0x2c
#define ADXL345_ACCELEROMETER_ADXLREG_POWER_CTL        0x2d
#define ADXL345_ACCELEROMETER_ADXLREG_DATA_FORMAT      0x31
#define ADXL345_ACCELEROMETER_ADXLREG_DEVID            0x00
#define ADXL345_ACCELEROMETER_ADXLREG_DATAX0           0x32
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL         0x38
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_CTL_STREAM     0x9F
#define ADXL345_ACCELEROMETER_ADXLREG_FIFO_STATUS      0x39*/

#define LIS331DLH_I2C_ADDRESS 0b0011000
#define LIS331DLH_I2C_WHOAMI 0x0F

#define LIS331DLH_I2C_RANGE_2         0
#define LIS331DLH_I2C_RANGE_4         1
#define LIS331DLH_I2C_RANGE_8         2

#define LIS331DLH_I2C_CTRL_REG1       0x20
#define LIS331DLH_I2C_CTRL_REG1_PM    (1 << 5)

#define LIS331DLH_I2C_CTRL_REG4       0x23

#define LIS331DLH_I2C_OUT_X           0x28
#define LIS331DLH_I2C_OUT_Y           0x2A
#define LIS331DLH_I2C_OUT_Z           0x2C

#define LIS331DLH_I2C_ADR_FS_2        0x00
#define LIS331DLH_I2C_ADR_FS_4        0x10
#define LIS331DLH_I2C_ADR_FS_8        0x30

#define LIS331DLH_I2C_G               9.8

#define LIS331DLH_I2C_SENS_FS_2       0.001
#define LIS331DLH_I2C_SENS_FS_4       0.002
#define LIS331DLH_I2C_SENS_FS_8       0.0039


// ADXL345 accelerometer scaling
// Result will be scaled to 1m/s/s
// ADXL345 in Full resolution mode (any g scaling) is 256 counts/g, so scale by 9.81/256 = 0.038320312
//#define ADXL345_ACCELEROMETER_SCALE_M_S    (GRAVITY_MSS / 256.0f)

/// Gyro ITG3205 register definitions
#define L3G4200D_I2C_ADDRESS                       0x68

#define L3G4200D_REG_WHO_AM_I                      0x0f
#define L3G4200D_REG_WHO_AM_I_VALUE                     0xd3

#define L3G4200D_REG_CTRL_REG1                     0x20
#define L3G4200D_REG_CTRL_REG1_DRBW_800_110             0xf0
#define L3G4200D_REG_CTRL_REG1_PD                       0x08
#define L3G4200D_REG_CTRL_REG1_XYZ_ENABLE               0x07

#define L3G4200D_REG_CTRL_REG4                     0x23
#define L3G4200D_REG_CTRL_REG4_FS_2000                  0x20

#define L3G4200D_REG_CTRL_REG5                     0x24
#define L3G4200D_REG_CTRL_REG5_FIFO_EN                  0x40

#define L3G4200D_REG_FIFO_CTL                      0x2e
#define L3G4200D_REG_FIFO_CTL_STREAM                    0x40

#define L3G4200D_REG_FIFO_SRC                      0x2f
#define L3G4200D_REG_FIFO_SRC_ENTRIES_MASK              0x1f
#define L3G4200D_REG_FIFO_SRC_EMPTY                     0x20
#define L3G4200D_REG_FIFO_SRC_OVERRUN                   0x40

#define L3G4200D_REG_XL                            0x28

// this bit is ORd into the register to enable auto-increment mode
#define L3G4200D_REG_AUTO_INCREMENT		           0x80

// L3G4200D Gyroscope scaling
// running at 2000 DPS full range, 16 bit signed data, datasheet 
// specifies 70 mdps per bit
#define L3G4200D_GYRO_SCALE_R_S (DEG_TO_RAD * 70.0f * 0.001f)

// constructor
AP_InertialSensor_L3G4200D::AP_InertialSensor_L3G4200D(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu),
    _data_idx(0),
    _have_gyro_sample(false),
    _have_accel_sample(false),
    _have_sample(false),
    _accel_filter(800, 10),
    _gyro_filter(800, 10)
{
	_have_sample = true;//No
    pthread_spin_init(&_data_lock, PTHREAD_PROCESS_PRIVATE);
}

AP_InertialSensor_L3G4200D::~AP_InertialSensor_L3G4200D()
{
    pthread_spin_destroy(&_data_lock);
}

bool AP_InertialSensor_L3G4200D::_init_sensor(void) 
{
  // if (!_hardware_init())
    //    return false;
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER))
        return false;

    // Init the accelerometer
    uint8_t data = 0;
    
     //IF WHO AM I PRESENTS
    hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, LIS331DLH_I2C_WHOAMI, &data);
    if (data != 0b00110010) {
        hal.scheduler->panic(PSTR("AP_InertialSensor_L3G4200D: could not find LIS331DLH accelerometer sensor"));
    }
    else
    {
		hal.console->print_P(PSTR("OK LIS331DLH"));
	}
    
    hal.i2c->writeRegister(LIS331DLH_I2C_ADDRESS, LIS331DLH_I2C_CTRL_REG4, LIS331DLH_I2C_ADR_FS_8);
    hal.scheduler->delay(5);
    hal.i2c->writeRegister(LIS331DLH_I2C_ADDRESS, LIS331DLH_I2C_CTRL_REG1, 0x7|LIS331DLH_I2C_CTRL_REG1_PM);
    hal.scheduler->delay(5);
    
     hal.i2c->readRegister(L3G4200D_I2C_ADDRESS, L3G4200D_REG_WHO_AM_I, &data);
    if (data != L3G4200D_REG_WHO_AM_I_VALUE)
        hal.scheduler->panic(PSTR("AP_InertialSensor_L3G4200D: could not find L3G4200D gyro sensor"));
     else
		hal.console->print_P(PSTR("OK L3G4200D"));
        
      // setup for 800Hz sampling with 110Hz filter
    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG1, 
                           L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                           L3G4200D_REG_CTRL_REG1_PD |
                           L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG1, 
                           L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                           L3G4200D_REG_CTRL_REG1_PD |
                           L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG1, 
                           L3G4200D_REG_CTRL_REG1_DRBW_800_110 |
                           L3G4200D_REG_CTRL_REG1_PD |
                           L3G4200D_REG_CTRL_REG1_XYZ_ENABLE);
    hal.scheduler->delay(1);

    // setup for 2000 degrees/sec full range
    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG4, 
                           L3G4200D_REG_CTRL_REG4_FS_2000);
    hal.scheduler->delay(1);

    // enable FIFO
    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS, 
                           L3G4200D_REG_CTRL_REG5, 
                           L3G4200D_REG_CTRL_REG5_FIFO_EN);
    hal.scheduler->delay(1);

    // enable FIFO in stream mode. This will allow us to read the gyros much less frequently
    hal.i2c->writeRegister(L3G4200D_I2C_ADDRESS,
                           L3G4200D_REG_FIFO_CTL,
                           L3G4200D_REG_FIFO_CTL_STREAM);
    hal.scheduler->delay(1);
                           

    // Set up the filter desired
    _set_filter_frequency(_accel_filter_cutoff());

    // give back i2c semaphore
    i2c_sem->give();

    // start the timer process to read samples
    hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_L3G4200D::_accumulate, void));   
        
    _gyro_instance = _imu.register_gyro();
    _accel_instance = _imu.register_accel();
    xx=xy=xz=-10;
    for (reg=0;reg!=6;reg++)
		buffer[reg]=0;
	reg = LIS331DLH_I2C_OUT_X;
    _product_id = AP_PRODUCT_ID_L3G4200D;
	hal.console->print_P(PSTR("OK2 L3G4200D"));
    return true;
}

/*
  set the filter frequency
 */
void AP_InertialSensor_L3G4200D::_set_filter_frequency(uint8_t filter_hz)
{
    _accel_filter.set_cutoff_frequency(800, filter_hz);
    _gyro_filter.set_cutoff_frequency(800, filter_hz);
}

/*
  copy filtered data to the frontend
 */
bool AP_InertialSensor_L3G4200D::update(void) 
{
    Vector3f accel, gyro;

    //_have_sample = true;
   // _publish_gyro(_gyro_instance, gyro);

    pthread_spin_lock(&_data_lock);
    unsigned int idx = !_data_idx;
    accel = _data[idx].accel_filtered;
    gyro = _data[idx].gyro_filtered;
    _have_sample = false;
    pthread_spin_unlock(&_data_lock);

    // Adjust for chip scaling to get m/s/s
//accel=accel>>4;
    accel *= LIS331DLH_I2C_SENS_FS_8*LIS331DLH_I2C_G/16.0f;
    _publish_accel(_accel_instance, accel);

    // Adjust for chip scaling to get radians/sec
    gyro *= L3G4200D_GYRO_SCALE_R_S;
    _publish_gyro(_gyro_instance, gyro);

    if (_last_filter_hz != _accel_filter_cutoff()) {
        _set_filter_frequency(_accel_filter_cutoff());
        _last_filter_hz = _accel_filter_cutoff();
    }

    return true;
}

// Accumulate values from accels and gyros
void AP_InertialSensor_L3G4200D::_accumulate(void)
{
    // get pointer to i2c bus semaphore
   
        /*_have_gyro_sample = true;
        _have_accel_sample = true;

        _have_sample = true;
    
        */
        _last_timer=hal.scheduler->micros();
     //AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();
//hal.console->println("Gyro loop!");
    // take i2c bus sempahore
    /*if (!i2c_sem->take_nonblocking())
        return;*/

    /*uint8_t num_samples_available;
    uint8_t fifo_status = 0;

    // Read gyro FIFO status
    fifo_status = 0;
    hal.i2c->readRegister(L3G4200D_I2C_ADDRESS,
                          L3G4200D_REG_FIFO_SRC,
                          &fifo_status);
    if (fifo_status & L3G4200D_REG_FIFO_SRC_OVERRUN) {
        // FIFO is full
        num_samples_available = 32;
    } else if (fifo_status & L3G4200D_REG_FIFO_SRC_EMPTY) {
        // FIFO is empty
        num_samples_available = 0;
    } else {
        // FIFO is partly full
        num_samples_available = fifo_status & L3G4200D_REG_FIFO_SRC_ENTRIES_MASK;
    }

    if (num_samples_available > 0) {
        // read all the entries in one go, using AUTO_INCREMENT. This saves a lot of time on I2C setup
        int16_t buffer[num_samples_available][3];
        if (hal.i2c->readRegisters(L3G4200D_I2C_ADDRESS, L3G4200D_REG_XL | L3G4200D_REG_AUTO_INCREMENT, 
                                   sizeof(buffer), (uint8_t *)&buffer[0][0]) == 0)
        {
            for (uint8_t i=0; i<num_samples_available; i++) {
                _data[_data_idx].gyro_filtered = _gyro_filter.apply(Vector3f(buffer[i][0], -buffer[i][1], -buffer[i][2]));
                _have_gyro_sample = true;
            }
        }
    }

    // Read accelerometer FIFO to find out how many samples are available
   
    num_samples_available = 1;

    // read the samples and apply the filter
    if (num_samples_available > 0) {
        uint8_t buffer[6];
       
		//uint8_t lowByte=0;
		//uint8_t highByte=0;
		uint8_t rerror=0;
		
		uint8_t reg = LIS331DLH_I2C_OUT_X;
		
		if (hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, reg,&buffer[0])!=0)
		{
			rerror=1;
		}
		
		++reg;
		
		if (hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, reg,&buffer[1])!=0)
		{
			rerror=1;
		}          
		
		++reg;
		
		if (hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, reg,&buffer[2])!=0)
		{
			rerror=1;
		}          
		            
		++reg;
		
		if (hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, reg,&buffer[3])!=0)
		{
			rerror=1;
		}          
		            
		++reg;
		
		if (hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, reg,&buffer[4])!=0)
		{
			rerror=1;
		}          
		
		++reg;
		
		if (hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, reg,&buffer[5])!=0)
		{
			rerror=1;
		}          
		
		            
		//if (hal.i2c->readRegistersMultiple(LIS331DLH_I2C_ADDRESS, 
        //                                   LIS331DLH_I2C_OUT_X, 
        //                                   sizeof(buffer[0]), 6,
        //                                   (uint8_t *)&buffer[0]) == 0)  
        if (!rerror)
        {
			
            for (uint8_t i=0; i<num_samples_available; i++) {
				
				 int16_t xx =  (((int16_t)buffer[1]<<8)|buffer[0]);
				 int16_t xy =  (((int16_t)buffer[3]<<8)|buffer[2]);
				 int16_t xz =  (((int16_t)buffer[5]<<8)|buffer[4]);
				 //hal.console->printf_P("accel %f %f %f \n",xx,xy,xz);
                _data[_data_idx].accel_filtered = _accel_filter.apply(Vector3f(xx, -xy, -xz));
                _have_accel_sample = true;
            }
        }
        else
        {
			hal.console->println("Error LIS331DLH.");
			
		}
    }
*/

		
		/*uint8_t rerror=0; //Getting by multi update get.

		if (hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, reg,&buffer[reg-LIS331DLH_I2C_OUT_X])!=0)
		{
			reg=LIS331DLH_I2C_OUT_X;
		}
		
		++reg;
		if (reg==LIS331DLH_I2C_OUT_X+6)
			reg=LIS331DLH_I2C_OUT_X;
		*/
		

_data[_data_idx].gyro_filtered = _gyro_filter.apply(Vector3f(0, 0.01, 0));
                _have_gyro_sample = true;
                
				/* int16_t xx =  (((int16_t)buffer[1]<<8)|buffer[0]);
				 int16_t xy =  (((int16_t)buffer[3]<<8)|buffer[2]);
				 int16_t xz =  (((int16_t)buffer[5]<<8)|buffer[4]);
                _data[_data_idx].accel_filtered = _accel_filter.apply(Vector3f(xx, -xy, -xz));*/
                _data[_data_idx].accel_filtered = _accel_filter.apply(Vector3f(xy, -xx, -xz));
                _have_accel_sample = true;
                    
    // give back i2c semaphore
    //i2c_sem->give();

    if (_have_accel_sample && _have_gyro_sample) {
        _have_gyro_sample = false;
        _have_accel_sample = false;
		//hal.console->println("Have sample of gyro!");
        pthread_spin_lock(&_data_lock);
        _data_idx = !_data_idx;
        _have_sample = true;
        pthread_spin_unlock(&_data_lock);
    }   
    
   // hal.console->printf_P("Delta time: %u \n",hal.scheduler->micros() - _last_timer);    
        
}


// Accumulate values from accels and gyros
void AP_InertialSensor_L3G4200D::async_accumulate(void)
{
	//hal.console->println("Async accumulate.");
    // get pointer to i2c bus semaphore
    _last_timer=hal.scheduler->micros();
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus sempahore
    if (!i2c_sem->take_nonblocking())
        return;

   
    // Read accelerometer FIFO to find out how many samples are available
    
    if (hal.i2c->readRegister(LIS331DLH_I2C_ADDRESS, reg,&buffer[reg-LIS331DLH_I2C_OUT_X])!=0)
	{
			reg=LIS331DLH_I2C_OUT_X;
	}
		
		++reg;
		if (reg==LIS331DLH_I2C_OUT_X+6)
		{
				reg=LIS331DLH_I2C_OUT_X;
		
				 xx =  (((int16_t)buffer[1]<<8)|buffer[0]);
				 xy =  (((int16_t)buffer[3]<<8)|buffer[2]);
				 xz =  (((int16_t)buffer[5]<<8)|buffer[4]);
        }


                    
    // give back i2c semaphore
    i2c_sem->give();
  
}

AP_InertialSensor_Backend *AP_InertialSensor_L3G4200D::detect(AP_InertialSensor &_imu)
{
    AP_InertialSensor_L3G4200D *sensor = new AP_InertialSensor_L3G4200D(_imu);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->_init_sensor()) {
        delete sensor;
        return NULL;
    }

    return sensor;
}

#endif // CONFIG_HAL_BOARD

