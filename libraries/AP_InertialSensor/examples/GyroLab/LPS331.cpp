#include "LPS331.h"

// Defines ///////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LPS331AP_ADDRESS_SA0_LOW  0x5c //5c
#define LPS331AP_ADDRESS_SA0_HIGH 0b1011101

extern const AP_HAL::HAL& hal;

// Constructors //////////////////////////////////////////////////////

LPS331::LPS331(void)
{
  // Pololu board pulls SA0 high, so default assumption is that it is
  // high
  address = LPS331AP_ADDRESS_SA0_LOW;
}

// Public Methods ////////////////////////////////////////////////////

// sets or detects slave address; returns bool indicating success
void LPS331::begin()
{
	
  /*Wire.begin();*/
  writeReg(LPS331_CTRL_REG1, 0b11100000);
  hal.scheduler->delay(100);
}

// writes register
void LPS331::writeReg(uint8_t reg, uint8_t value)
{
 
    
    
    if (hal.i2c->writeRegister(address, reg,value)!=0)
	{
		hal.console->println("I2C error.");
	}
	    
    /*char buf[1];
    char regaddr[2];
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(address);
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500);
    bcm2835_i2c_set_baudrate(100000);
    
    regaddr[0] = reg;
    regaddr[1] = value;
    
    bcm2835_i2c_write(regaddr, 1);
    bcm2835_i2c_read(buf, 1);
    //printf("%d",buf[0]);
    bcm2835_i2c_end(); */
    
	
}

// reads register
uint8_t LPS331::readReg(uint8_t reg)
{
  /*byte value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission(false); // restart
  Wire.requestFrom(address, (byte)1);
  value = Wire.read();
  Wire.endTransmission();*/
 /* i2cd = wiringPiI2CSetup(address);
    return wiringPiI2CReadReg8(i2cd,reg);*/
    /*char buf[1];
    char regaddr[2];
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(address);
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500);
    bcm2835_i2c_set_baudrate(100000);
    
    regaddr[0] = reg;
    regaddr[1] = 0;
    
    bcm2835_i2c_write(regaddr, 1);
    bcm2835_i2c_read(buf, 1);
    //printf("%d",buf[0]);
    bcm2835_i2c_end(); 
	return buf[0];*/
	  
	 
    uint8_t i2cres=0;
    
    if (hal.i2c->readRegister(address, reg,&i2cres)!=0)
	{
		hal.console->println("I2C error.");
	}
	
    return i2cres;
}

// reads pressure in millibars (mbar)/hectopascals (hPa)
float LPS331::readPressureMillibars(void)
{
  return (float)readPressureRaw() / 4096;
}

// reads pressure in inches of mercury (inHg)
float LPS331::readPressureInchesHg(void)
{
  return (float)readPressureRaw() / 138706.5;
}

// reads pressure and returns raw 24-bit sensor output
uint32_t LPS331::readPressureRaw(void)
{
  /*Wire.beginTransmission(address);
  // assert MSB to enable register address auto-increment
  Wire.write(LPS331_PRESS_OUT_XL | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)3);

  while (Wire.available() < 3);

  uint8_t pxl = Wire.read();
  uint8_t pl = Wire.read();
  uint8_t ph = Wire.read();

  // combine bytes*/
  
	/*char buf[3];
    char regaddr[2];
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(address);
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500);
    bcm2835_i2c_set_baudrate(100000);
    
    regaddr[0] = LPS331_PRESS_OUT_XL | (1 << 7);
    regaddr[1] = 0;
    
    bcm2835_i2c_write(regaddr, 1);
    bcm2835_i2c_read(buf, 3);
    printf("%d\n",buf[1]);
    printf("%d\n",buf[2]);
    printf("%d\n",buf[3]);
    bcm2835_i2c_end(); 
  
  
  
  return (uint32_t)(uint8_t)buf[1] << 16 | (uint16_t)buf[2] << 8 | buf[3];*/
  
    uint8_t buf[3];
	
  
    //uint8_t i2cres=0;
    
    /*if (hal.i2c->readRegister(address, reg,&i2cres)!=0)
	{
		hal.console->println("I2C error 2.");
	}*/
	
	 if (hal.i2c->readRegisters(address, LPS331_PRESS_OUT_XL | (1 << 7), 3, buf) != 0) {
        hal.console->println("I2C error 3.");
        return 0;
    }
	// hal.console->printf_P("%d\n",buf[0]);
    // hal.console->printf_P("%d\n",buf[1]);
     //hal.console->printf_P("%d\n",buf[2]);
    return (uint32_t)(uint8_t)buf[2] << 16 | (uint16_t)buf[1] << 8 | buf[0];
}

// reads temperature in degrees C
float LPS331::readTemperatureC(void)
{
  return 42.5 + (float)readTemperatureRaw() / 480;
}

// reads temperature in degrees F
float LPS331::readTemperatureF(void)
{
  return 108.5 + (float)readTemperatureRaw() / 480 * 1.8;
}

// reads temperature and returns raw 16-bit sensor output
int16_t LPS331::readTemperatureRaw(void)
{
  /*Wire.beginTransmission(address);
  // assert MSB to enable register address auto-increment
  Wire.write(LPS331_TEMP_OUT_L | (1 << 7));
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)2);

  while (Wire.available() < 2);

  uint8_t tl = Wire.read();
  uint8_t th = Wire.read();

  // combine bytes
  return (int16_t)(th << 8 | tl);*/
  
  /*char buf[2];
    char regaddr[2];
    bcm2835_i2c_begin();
    bcm2835_i2c_setSlaveAddress(address);
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500);
    bcm2835_i2c_set_baudrate(100000);
    
    regaddr[0] = LPS331_TEMP_OUT_L | (1 << 7);
    regaddr[1] = 0;
    
    bcm2835_i2c_write(regaddr, 1);
    bcm2835_i2c_read(buf, 2);
    //printf("%d",buf[0]);
    bcm2835_i2c_end(); 
  
  return (int16_t)(buf[1] << 8 | buf[0]);*/
  
  
   uint8_t buf[2];
	
	
	 if (hal.i2c->readRegisters(address, LPS331_TEMP_OUT_L | (1 << 7), 2, buf) != 0) {
        hal.console->println("I2C error 2.");
        return 0;
    }
	 hal.console->printf_P("%d\n",buf[0]);
     hal.console->printf_P("%d\n",buf[1]);
     
    return (int16_t)(buf[1] << 8 | buf[0]);
  
}

// converts pressure in mbar to altitude in meters, using 1976 US
// Standard Atmosphere model (note that this formula only applies to a
// height of 11 km, or about 36000 ft)
//  If altimeter setting (QNH, barometric pressure adjusted to sea
//  level) is given, this function returns an indicated altitude
//  compensated for actual regional pressure; otherwise, it returns
//  the pressure altitude above the standard pressure level of 1013.25
//  mbar or 29.9213 inHg
float LPS331::pressureToAltitudeMeters(float pressure_mbar, float altimeter_setting_mbar)
{
  return (1 - pow(pressure_mbar / altimeter_setting_mbar, 0.190263)) * 44330.8;
}

// converts pressure in inHg to altitude in feet; see notes above
float LPS331::pressureToAltitudeFeet(float pressure_inHg, float altimeter_setting_inHg)
{
  return (1 - pow(pressure_inHg / altimeter_setting_inHg, 0.190263)) * 145442;
}

// Private Methods ///////////////////////////////////////////////////

bool LPS331::autoDetectAddress(void)
{
  // try each possible address and stop if reading WHO_AM_I returns the expected response
  address = LPS331AP_ADDRESS_SA0_LOW;
  if (testWhoAmI()) return true;
  

  return false;
}

bool LPS331::testWhoAmI(void)
{
  return (readReg(LPS331_WHO_AM_I) == 0xBB);
}
