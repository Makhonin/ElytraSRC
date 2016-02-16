#include "lis331dlh.h"

extern const AP_HAL::HAL& hal;

#define CTRL_REG1       0x20
#define CTRL_REG1_PM    (1 << 5)

#define CTRL_REG4       0x23

#define OUT_X           0x28
#define OUT_Y           0x2A
#define OUT_Z           0x2C

#define ADR_FS_2        0x00
#define ADR_FS_4        0x10
#define ADR_FS_8        0x30

#define G               9.8

#define SENS_FS_2       0.001
#define SENS_FS_4       0.002
#define SENS_FS_8       0.0039

LIS331DLH_TWI::LIS331DLH_TWI(uint8_t addr)
{
    _addr = addr;

    _ctrlReg1 = 0x7; // default according to datasheet
}

void LIS331DLH_TWI::begin()
{
   // Wire.begin();
    setRange(LIS331RANGE_2);
    LIS331DLH_TWI::sleep(false);
}

void LIS331DLH_TWI::setRange(uint8_t range)
{
    switch (range) {
        case LIS331RANGE_2: {
            _ctrlReg4 = ADR_FS_2;
            _mult = SENS_FS_2;
            break;
        }
        case LIS331RANGE_4: {
            _ctrlReg4 = ADR_FS_4;
            _mult = SENS_FS_4;
            break;
        }
        case LIS331RANGE_8: {
            _ctrlReg4 = ADR_FS_8;
            _mult = SENS_FS_8;
            break;
        }
        default: {
        _mult = SENS_FS_8;    
        }
        break;
    }
    writeCtrlReg4();
}
void LIS331DLH_TWI::sleep(bool enable)
{
    if (enable)
        _ctrlReg1 &= ~CTRL_REG1_PM;
    else
        _ctrlReg1 |= CTRL_REG1_PM;

    writeCtrlReg1();
}


int16_t LIS331DLH_TWI::readX()
{
	hal.console->printf_P("X ");
    return readAxis(OUT_X);
}

int16_t LIS331DLH_TWI::readY()
{
	hal.console->printf_P("Y ");
    return readAxis(OUT_Y);
}

int16_t LIS331DLH_TWI::readZ()
{
	hal.console->printf_P("Z ");
    return readAxis(OUT_Z);
}

float LIS331DLH_TWI::readX_G()
{
    return readX()*_mult*G;
}

float LIS331DLH_TWI::readY_G()
{
    return readY()*_mult*G;
}

float LIS331DLH_TWI::readZ_G()
{
    return readZ()*_mult*G;
}

int16_t LIS331DLH_TWI::readAxis(uint8_t reg)
{
   /* Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(_addr, 1);
    while (Wire.available() < 1)
        ;
    uint8_t lowByte = Wire.read();

    ++reg;

    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(_addr, 1);
    while (Wire.available() < 1)
        ;
    uint8_t highByte = Wire.read();

    return (((int16_t)highByte << 8) | lowByte) >> 4;*/
    
   
    uint8_t lowByte=0;
    uint8_t highByte=0;
    
    if (hal.i2c->readRegister(_addr, reg,&lowByte)!=0)
	{
		hal.console->println("I2C error.");
	}
	
	++reg;
	
	if (hal.i2c->readRegister(_addr, reg,&highByte)!=0)
	{
		hal.console->println("I2C error.");
	}
	hal.console->printf_P("H%d L%d ",highByte,lowByte);
	
    
    int16_t res =  (((int16_t)highByte<<8)|lowByte);
    res=res/16;
    return res;
}

uint8_t LIS331DLH_TWI::readByte(uint8_t reg)
{
   /* Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(_addr, 1u);
    while (!Wire.available())
        ;*/
    uint8_t i2cres=0;
    
    if (hal.i2c->readRegister(_addr, reg,&i2cres)!=0)
	{
		hal.console->println("I2C error.");
	}
	
    return i2cres;
}

void LIS331DLH_TWI::writeCtrlReg1()
{
    /*Wire.beginTransmission(_addr);
    Wire.write(CTRL_REG1);
    Wire.write(_ctrlReg1);
    Wire.endTransmission();*/
    //i2cd = wiringPiI2CSetup(_addr);
    //int res = wiringPiI2CWriteReg8(i2cd,CTRL_REG1,_ctrlReg1);
    
    if (hal.i2c->writeRegister(_addr, CTRL_REG1,_ctrlReg1)!=0)
	{
		hal.console->println("I2C error.");
	}
}

void LIS331DLH_TWI::writeCtrlReg4()
{
    /*Wire.beginTransmission(_addr);
    Wire.write(CTRL_REG4);
    Wire.write(_ctrlReg4);
    Wire.endTransmission();*/
    //i2cd = wiringPiI2CSetup(_addr);
    //int res = wiringPiI2CWriteReg8(i2cd,CTRL_REG4,_ctrlReg4);
    
    if (hal.i2c->writeRegister(_addr, CTRL_REG4,_ctrlReg4)!=0)
	{
		hal.console->println("I2C error.");
	}
    
}
