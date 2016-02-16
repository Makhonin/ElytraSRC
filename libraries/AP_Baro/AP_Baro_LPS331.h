/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef __AP_Baro_LPS331_H__
#define __AP_Baro_LPS331_H__

#include <AP_HAL.h>
#include "AP_Baro.h"
#include "AP_Baro_MS5611.h"

class AP_Baro_LPS331 : public AP_Baro_Backend
{
public:
    AP_Baro_LPS331(AP_Baro &baro, AP_SerialBus *serial, bool use_timer);
    void update();
    void accumulate();

private:
    void _calculate();
    AP_SerialBus *_serial;

    bool _check_crc();

    void _timer();

    /* Asynchronous state: */
    volatile bool            _updated;
    volatile uint8_t         _d1_count;
    volatile uint8_t         _d2_count;
    volatile uint32_t        _s_D1;
    volatile int32_t _s_D2;
    uint8_t                  _state;
    uint32_t                 _last_timer;

    bool _use_timer;

protected:
    // Internal calibration registers
    uint16_t                 _C1,_C2,_C3,_C4,_C5,_C6;
    float                    _D1,_D2;
    uint8_t _instance;
};


#endif // __AP_Baro_LPS331_H__
