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


uint8_t data_ready=0;
uint32_t timer;
uint8_t info[4];
uint8_t fl[4];
float nav_data[10];
int32_t latitude;
int32_t longitude;
const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

/*


USAGE:

Plug in RS232-compatible adapter(i.e. FT232) to Raspberry USB port.
Run 
sudo GyroLab.elf -B /dev/ttyS1

Fix ttyS1 if needed.

Code by Sgw32 in 2016.

*/

/* wait for <SYNC> */

void sync()
{
	int a = hal.uartB->available();
	while (!a)
		a = hal.uartB->available();
	int r = hal.uartB->read();
	while (r!=0xAA)
	{
		while (!a)
			a = hal.uartB->available();
		r = hal.uartB->read();
	}
}

/* Wait for num bytes */

void wait_for_ready(int bytes)
{
	int a = a = hal.uartB->available();
	while (a!=bytes)
		a = hal.uartB->available();
}

void processDataBlock(int len,int id)
{
	//Navigation data packet. See manual page 7 paragraph 3.2
	if ((len==55)&&(id==0x70))
	{
		wait_for_ready(54);
		

		//Status data. 
		//Not used for now.
		for (uint8_t i=0;i!=4;i++)
			info[i]=hal.uartB->read();
		//Read 9x floats from buffer.
		for (uint8_t i=0;i!=9;i++)
		{
			for (uint8_t j=0;j!=4;j++)
				fl[j]=hal.uartB->read();
			nav_data[i] = *(float *)&fl;
		}

		//Read GPS data (not used?)
		for (uint8_t j=0;j!=4;j++)
				fl[j]=hal.uartB->read();

		latitude = (fl[0] << 24) | (fl[1] << 16) | (fl[2] << 8) | fl[3];

		for (uint8_t j=0;j!=4;j++)
				fl[j]=hal.uartB->read();

		longitude = (fl[0] << 24) | (fl[1] << 16) | (fl[2] << 8) | fl[3];

		// Last is GPS height
		for (uint8_t j=0;j!=4;j++)
				fl[j]=hal.uartB->read();
			nav_data[9] = *(float *)&fl;

		//Dump ok there. Ready for output.
		data_ready=1;
	}
	else
	{
		
	}
}

void process()
{
	if (data_ready)
		return;
	/* Wait for <SYNC> */
	sync();

	/* <SYNC><SYNC><LEN><ID> = 4 bytes (4-1 = 3) */

    if (a > 2) 
	{
		int r = hal.uartB->read();

		// <SYNC> <SYNC>
		if (r==0xAA)
		{
			int len = hal.uartB->read();
			int id = hal.uartB->read();
			processDataBlock(len,id);
		}

	}
}

void setup(void)
{
	// As said in manual. Baud rate = 460800
    hal.uartB->begin(460800, 128, 256);

    hal.console->println("Elytra test");
    hal.console->println("Initialising GyroLab MEMS...");
	
    hal.scheduler->delay(100);
   
    hal.console->println("Started.");

	
}



void loop(void)
{
	process();
    if((hal.scheduler->micros()- timer) > 1000000L)
    {
		//compass.readXYZ_Calib();
		//float pressure = barometer.readPressureMillibars();
		//float temperature = barometer.readTemperatureC();
		
		// printf("%f ",gyro.readX_DegPerSec());
		/*printf("%f ",gyro.readY_DegPerSec());
		printf("%f ",gyro.readZ_DegPerSec());
		*/
		
		//hal.console->printf_P("%f %f %f yaw: %f %f %f %f %f %f\n",accel.readX_G(),accel.readY_G(),accel.readZ_G(),compass.read_Yaw(),gyro.readX_DegPerSec(),gyro.readY_DegPerSec(),gyro.readZ_DegPerSec(),pressure,temperature);
        
		if (data_ready)
		{
			hal.console->println("Ok.");
			hal.uartB->flush();
			data_ready=0;
		}
		else
		{
			hal.console->println("No data or data error.");
			hal.uartB->flush();
		}
		timer = hal.scheduler->micros();
    } else {
	    hal.scheduler->delay(1);
    }
}

AP_HAL_MAIN();
