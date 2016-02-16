#include <AP_Math.h>
#include <AP_HAL.h>


#include "AP_LIS3MDL.h"

extern const AP_HAL::HAL& hal;

#define COMPASS_ADDRESS      0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

AP_Compass_LIS3MDL::AP_Compass_LIS3MDL(Compass &compass):
    AP_Compass_Backend(compass),
    _retry_time(0),
    _i2c_sem(NULL),
    _mag_x(0),
    _mag_y(0),
    _mag_z(0),
    _mag_x_accum(0),
    _mag_y_accum(0),
    _mag_z_accum(0),
    _accum_count(0),
    _last_accum_time(0),
    _compass_instance(0),
    _product_id(0)
{}

// detect the sensor
AP_Compass_Backend *AP_Compass_LIS3MDL::detect(Compass &compass)
{
    AP_Compass_LIS3MDL *sensor = new AP_Compass_LIS3MDL(compass);
    if (sensor == NULL) {
        return NULL;
    }
    if (!sensor->init()) {
        delete sensor;
        return NULL;
    }
    return sensor;
}

// read_register - read a register value
bool AP_Compass_LIS3MDL::read_register(uint8_t address, uint8_t *value)
{
    if (hal.i2c->readRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }
    return true;
}

// write_register - update a register value
bool AP_Compass_LIS3MDL::write_register(uint8_t address, uint8_t value)
{
    if (hal.i2c->writeRegister((uint8_t)COMPASS_ADDRESS, address, value) != 0) {
        _retry_time = hal.scheduler->millis() + 1000;
        return false;
    }
    return true;
}

// Read Sensor data
bool AP_Compass_LIS3MDL::read_raw()
{

    _mag_x = 0;
    _mag_y = 0;
    _mag_z = 0;

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_LIS3MDL::accumulate(void)
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
}


/*
 *  re-initialise after a IO error
 */
bool AP_Compass_LIS3MDL::re_initialise()
{
    /*if (!write_register(ConfigRegA, _base_config) ||
        !write_register(ConfigRegB, magGain) ||
        !write_register(ModeRegister, ContinuousConversion))
        return false;*/
    return true;
}


// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_LIS3MDL::init()
{
   
    hal.console->println("Using LIS3MDL compass for Elytra.");
	hal.console->println("Compass LIS3MDL setup complete!");
    return true;
}

// Read Sensor data
void AP_Compass_LIS3MDL::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
    
}

