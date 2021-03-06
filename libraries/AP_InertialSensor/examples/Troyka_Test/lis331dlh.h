#include <stdarg.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifndef LIS331DLH_H
#define LIS331DLH_H

#define LIS331DLH_TWI_ADDRESS 0b0011000

#define LIS331RANGE_2         0
#define LIS331RANGE_4         1
#define LIS331RANGE_8         2


class LIS331DLH_TWI
{
    public:
        LIS331DLH_TWI(uint8_t addr = LIS331DLH_TWI_ADDRESS);

        void begin();
        void sleep(bool enable);

        int16_t readX();
        int16_t readY();
        int16_t readZ();
        float readX_G();
        float readY_G();
        float readZ_G();
        void setRange(uint8_t range);

    private:
        uint8_t _addr;
        uint8_t _ctrlReg1;
        uint8_t _ctrlReg4;
        float _mult;
		int i2cd;
    protected:
        void writeCtrlReg1();
        void writeCtrlReg4();
        int16_t readAxis(uint8_t reg);
        uint8_t readByte(uint8_t reg);
};

#endif
