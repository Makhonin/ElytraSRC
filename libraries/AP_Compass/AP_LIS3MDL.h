#ifndef AP_LIS3MDL_H
#define AP_LIS3MDL_H

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"
#include "AP_Compass_Backend.h"

class AP_Compass_LIS3MDL : public AP_Compass_Backend
{
private:
    float               calibration[3];
    bool                _initialised;
    bool                read_raw(void);
    uint8_t             _base_config;
    bool                re_initialise(void);
    bool                read_register(uint8_t address, uint8_t *value);
    bool                write_register(uint8_t address, uint8_t value);
    uint32_t            _retry_time; // when unhealthy the millis() value to retry at
    AP_HAL::Semaphore*  _i2c_sem;

    int16_t			    _mag_x;
    int16_t			    _mag_y;
    int16_t			    _mag_z;
    int16_t             _mag_x_accum;
    int16_t             _mag_y_accum;
    int16_t             _mag_z_accum;
    uint8_t			    _accum_count;
    uint32_t            _last_accum_time;

    uint8_t             _compass_instance;
    uint8_t             _product_id;

public:
    AP_Compass_LIS3MDL(Compass &compass);
    bool        init(void);
    void        read(void);
    void        accumulate(void);

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);
};

#endif
