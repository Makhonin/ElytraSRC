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
  LPS331 barometer driver. Based on original code by Jordi Munoz and
  Jose Julio

  by Sgw32
*/

#include <AP_HAL.h>

#include "AP_Baro.h"

#define LPS331_REF_P_XL       0x08
#define LPS331_REF_P_L        0x09
#define LPS331_REF_P_H        0x0A

#define LPS331_WHO_AM_I       0x0F

#define LPS331_RES_CONF       0x10

#define LPS331_CTRL_REG1      0x20
#define LPS331_CTRL_REG2      0x21
#define LPS331_CTRL_REG3      0x22
#define LPS331_INTERRUPT_CFG  0x23
#define LPS331_INT_SOURCE     0x24
#define LPS331_THS_P_L        0x25
#define LPS331_THS_P_H        0x26
#define LPS331_STATUS_REG     0x27

#define LPS331_PRESS_OUT_XL   0x28
#define LPS331_PRESS_OUT_L    0x29
#define LPS331_PRESS_OUT_H    0x2A

#define LPS331_TEMP_OUT_L     0x2B
#define LPS331_TEMP_OUT_H     0x2C

#define LPS331_AMP_CTRL       0x30

#define LPS331_DELTA_PRESS_XL 0x3C
#define LPS331_DELTA_PRESS_L  0x3D
#define LPS331_DELTA_PRESS_H  0x3E

extern const AP_HAL::HAL& hal;

AP_Baro_LPS331::AP_Baro_LPS331(AP_Baro &baro, AP_SerialBus *serial, bool use_timer) :
    AP_Baro_Backend(baro),
    _serial(serial),
    _updated(false),
    _state(0),
    _last_timer(0),
    _use_timer(use_timer)
{
    _instance = _frontend.register_sensor();
    _serial->init();
    if (!_serial->sem_take_blocking()){
        hal.scheduler->panic(PSTR("PANIC: AP_Baro_LPS331: failed to take serial semaphore for init"));
    }
    
    _serial->write(LPS331_RES_CONF);
    hal.scheduler->delay(4);
    
    _serial->writeReg(LPS331_CTRL_REG1, 0b11100000);
    hal.scheduler->delay(100);
    _serial->write(LPS331_TEMP_OUT_L | (1 << 7));
    _last_timer = hal.scheduler->micros();
    _state = 0;

    _s_D1 = 0;
    _s_D2 = 0;
    _d1_count = 0;
    _d2_count = 0;

    _serial->sem_give();
    
	_last_timer = hal.scheduler->micros();
    if (_use_timer) {
        hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AP_Baro_LPS331::_timer, void));
    }
}

/**
 * MS5611 crc4 method based on PX4Firmware code
 */
bool AP_Baro_LPS331::_check_crc(void)
{
    return true;
}

void AP_Baro_LPS331::_calculate()
{
	// hal.console->println("calculate!");
     float pressure = (float)_D1 / 40.96f;
    float temperature = 42.5 + (float)_D2 / 480;
    _copy_to_frontend(_instance, pressure, temperature);
}

void AP_Baro_LPS331::accumulate()
{
    if (!_use_timer) {
        // the timer isn't being called as a timer, so we need to call
        // it in accumulate()
        _timer();
    }
}

/*
  Read the sensor. This is a state machine
  We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
  temperature does not change so quickly...
*/
void AP_Baro_LPS331::_timer(void)
{
	//hal.console->println("lps timer");
    // Throttle read rate to 100hz maximum.
    if (hal.scheduler->micros() - _last_timer < 10000) {
        return;
    }
   // hal.console->println("lps2");
    if (!_serial->sem_take_nonblocking()) {
        return;
    }

    if (_state == 0) {
        int16_t d2 = _serial->read_16bits_2(LPS331_TEMP_OUT_L | (1 << 7));
        if (d2 != 0) {
            _s_D2 += d2;
            _d2_count++;
            if (_d2_count == 32) {
                // we have summed 32 values. This only happens
                // when we stop reading the barometer for a long time
                // (more than 1.2 seconds)
                _s_D2 >>= 1;
                _d2_count = 16;
            }
        }
        _state++;
        //_serial->write(LPS331_PRESS_OUT_XL | (1 << 7));      // Command to read pressure
    } else {
        uint32_t d1 = _serial->read_24bits_2(LPS331_PRESS_OUT_XL | (1 << 7));;
        if (d1 != 0) {
            // occasional zero values have been seen on the PXF
            // board. These may be SPI errors, but safest to ignore
            _s_D1 += d1;
            _d1_count++;
            if (_d1_count == 128) {
                // we have summed 128 values. This only happens
                // when we stop reading the barometer for a long time
                // (more than 1.2 seconds)
                _s_D1 >>= 1;
                _d1_count = 64;
            }
            // Now a new reading exists
            _updated = true;
        }
        else
        {
			hal.console->println("Given zero pressure...");
		}
        _state++;
        if (_state == 5) {
            //_serial->write(LPS331_TEMP_OUT_L | (1 << 7)); // Command to read temperature
            _state = 0;
        } else {
            //_serial->write(LPS331_PRESS_OUT_XL | (1 << 7)); // Command to read pressure
        }
    }
    
    //_updated = true;
    _last_timer = hal.scheduler->micros();
	_serial->sem_give();
    //hal.console->println("lps3");
}

void AP_Baro_LPS331::update()
{
    if (!_use_timer) {
        // if we're not using the timer then accumulate one more time
        // to cope with the calibration loop and minimise lag
        accumulate();
    }

    if (!_updated) {
        return;
    }
    uint32_t sD1;
    int32_t  sD2;
    uint8_t d1count, d2count;

    // Suspend timer procs because these variables are written to
    // in "_update".
   
     hal.scheduler->suspend_timer_procs();
    sD1 = _s_D1; _s_D1 = 0;
    sD2 = _s_D2; _s_D2 = 0;
    d1count = _d1_count; _d1_count = 0;
    d2count = _d2_count; _d2_count = 0;
    _updated = false;
    hal.scheduler->resume_timer_procs();
    
    if (d1count != 0) {
        _D1 = ((float)sD1) / d1count;
    }
    if (d2count != 0) {
        _D2 = ((float)sD2) / d2count;
    }
    
    _calculate();
}
