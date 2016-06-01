/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <GCS_MAVLink.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_NavEKF.h>
#include <AP_BattMonitor.h>
#include <AP_RangeFinder.h>

#include <PCA9685.h>
#include <ElytraConfigurator.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

RC_Channel rc1(0), rc2(1), rc3(2), rc4(3);

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(rc1, rc2, rc3, rc4, 400);
AP_MotorsQuad   motors(400, 490);
//AP_MotorsHexa	motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsY6	motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsOcta	motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsOctaQuad	motors(rc1, rc2, rc3, rc4, 400);
//AP_MotorsHeli	motors(rc1, rc2, rc3, rc4, 400);
AeroxoTiltrotorPCA9685 pcaTilt;
ElytraConfigurator elCfg;

void motor_order_test();

// setup
void setup()
{
    hal.console->println("AP_Motors_Time test");
	pcaTilt.initPCA9685();
	elCfg.scanBenchFile();
	pcaTilt.setServo(4,2000-elCfg.getC1());
    pcaTilt.setServo(5,1000+elCfg.getC2());
    pcaTilt.setServo(6,1000+elCfg.getC3());
    pcaTilt.setServo(7,2000-elCfg.getC4());
    // motor initialisation
    motors.set_update_rate(490);
    motors.set_frame_orientation(AP_MOTORS_X_FRAME);
   // motors.set_frame_orientation(AP_MOTORS_PLUS_FRAME);
    //motors.set_min_throttle(130);
    motors.set_hover_throttle(500);
    motors.Init();      // initialise motors
	motors.setup_motors();
    
    motors.enable();
    motors.output_min();

    hal.scheduler->delay(4000);
}

// loop
void loop()
{

    motor_order_test();
}

// stability_test
void motor_order_test()
{

    motors.armed(true);
    
	hal.console->printf_P(PSTR("Motor 1\n"));
    motors.output_test(1, elCfg.getP1());
	hal.console->printf_P(PSTR("Motor 2\n"));
    motors.output_test(3, elCfg.getP2());
	hal.console->printf_P(PSTR("Motor 3\n"));
    motors.output_test(4, elCfg.getP3());
	hal.console->printf_P(PSTR("Motor 4\n"));
    motors.output_test(2, elCfg.getP4());

	hal.scheduler->delay(10000);
	for (int8_t i=1; i <= 4; i++)
	motors.output_test(i, 1100);
    motors.armed(false);
   hal.scheduler->delay(4000);
   hal.console->printf("\n\n");
}

AP_HAL_MAIN();
