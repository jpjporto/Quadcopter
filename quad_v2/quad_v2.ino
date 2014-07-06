#include <Wire.h>
#include "I2Cdev.h"
#include "MPU9150Lib.h"
#include "CalLib.h"
#include "DueFlash.h"
#include <dmpKey.h>
#include <dmpmap.h>
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <PID_v1.h>
#include <Servo.h>

//Constants
#define PWM_ZERO 1500
#define SENSOR_VOLTAGE 3.3
#define PWM_MAX 2000
#define RADIO_MAX 100
#define RADIO_MIN 20
#define MAX_ANGLE 60

double error_alt, radio_alt, desired_alt = 50, alt_pwm;
double error_x, error_y, error_z, x_axis, y_axis, z_axis, desired_x = 0, desired_y = 0, desired_z = 0;
double x_pwm, y_pwm, z_pwm;
int K = 5, pwm[4] = {0,0,0,0};
int flight_st;

enum FLIGHT_STATUS{
	FS_FLIGHT,
	FS_ERROR

};

Servo esc[4];
const int analogInPin = A0;
PID alt_PID(&radio_alt, &alt_pwm, &desired_alt, 5, 1, .5, DIRECT);
PID x_PID(&x_axis, &x_pwm, &desired_x, 5, 5, 1, DIRECT);
PID y_PID(&y_axis, &y_pwm, &desired_y, 5, 5, 1, DIRECT);
PID z_PID(&z_axis, &z_pwm, &desired_z, 5, 5, 1, DIRECT);


//  DEVICE_TO_USE selects whether the IMU at address 0x68 (default) or 0x69 is used
//    0 = use the device at 0x68
//    1 = use the device at ox69

#define  DEVICE_TO_USE    0

MPU9150Lib dueMPU;                                              // the MPU object

//  MPU_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the sensor data and DMP output

#define MPU_UPDATE_RATE  (20)

//  MAG_UPDATE_RATE defines the rate (in Hz) at which the MPU updates the magnetometer data
//  MAG_UPDATE_RATE should be less than or equal to the MPU_UPDATE_RATE

#define MAG_UPDATE_RATE  (10)

//  MPU_MAG_MIX defines the influence that the magnetometer has on the yaw output.
//  The magnetometer itself is quite noisy so some mixing with the gyro yaw can help
//  significantly. Some example values are defined below:

#define  MPU_MAG_MIX_GYRO_ONLY          0                   // just use gyro yaw
#define  MPU_MAG_MIX_MAG_ONLY           1                   // just use magnetometer and no gyro yaw
#define  MPU_MAG_MIX_GYRO_AND_MAG       10                  // a good mix value 
#define  MPU_MAG_MIX_GYRO_AND_SOME_MAG  50                  // mainly gyros with a bit of mag correction 

//  MPU_LPF_RATE is the low pas filter rate and can be between 5 and 188Hz

#define MPU_LPF_RATE   100

//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port

#define  SERIAL_PORT_SPEED  115200

int loopState;                                              // what code to run in the loop

#define  LOOPSTATE_NORMAL  0                                // normal execution
#define  LOOPSTATE_MAGCAL  1                                // mag calibration
#define  LOOPSTATE_ACCELCAL  2                              // accel calibration

static CALLIB_DATA calData;                                 // the calibration data

long lastPollTime;                                          // last time the MPU-9150 was checked
long pollInterval;                                          // gap between polls to avoid thrashing the I2C bus

void magCalStart(void);
void magCalLoop(void);
void accelCalStart(void);
void accelCalLoop(void);

void mpuInit()
{
  dueMPU.selectDevice(DEVICE_TO_USE);                        // only really necessary if using device 1
  dueMPU.init(MPU_UPDATE_RATE, MPU_MAG_MIX_GYRO_AND_MAG, MAG_UPDATE_RATE, MPU_LPF_RATE);   // start the MPU
}

boolean duePoll()
{
  if ((millis() - lastPollTime) < pollInterval)
    return false;                                            // not time yet
  if (dueMPU.read()) {
    lastPollTime = millis();
    return true;
  } 
  return false;                                              // try again next time round
}

void PID_setup()
{
	int sample_time = (1000 / MPU_UPDATE_RATE) - 1;
	alt_PID.SetSampleTime(sample_time);
	alt_PID.SetOutputLimits(0, PWM_MAX - PWM_ZERO);
	alt_PID.SetMode(AUTOMATIC);
	
	x_PID.SetSampleTime(sample_time);
	x_PID.SetOutputLimits(0, PWM_MAX - PWM_ZERO);
	x_PID.SetMode(AUTOMATIC);
	
	y_PID.SetSampleTime(sample_time);
	y_PID.SetOutputLimits(0, PWM_MAX - PWM_ZERO);
	y_PID.SetMode(AUTOMATIC);
	
	z_PID.SetSampleTime(sample_time);
	z_PID.SetOutputLimits(0, PWM_MAX - PWM_ZERO);
	z_PID.SetMode(AUTOMATIC);
}

/*
	Get radio altitude
	inputs: none
	outputs: altitude in cm
*/
inline double radio_altitude(void)
{
  unsigned int sensorValue = analogRead(analogInPin);
  static unsigned int processed_val = processed_val * 0.75 + sensorValue * 0.25;
  double Voltage = (double)sensorValue*SENSOR_VOLTAGE/4096;
  double dist = 41.543*pow((Voltage + 0.30221),-1.5281);
  return constrain(dist, RADIO_MIN, RADIO_MAX);
}

void esc_setup()
{
  esc[0].attach(9);
  esc[1].attach(10);
  esc[2].attach(11);
  esc[3].attach(12);
  
  for (int i = 0; i < 4; i++)
    esc[i].writeMicroseconds(PWM_ZERO);
  
}

void update_speeds(int pwm[])
{
	for (int i = 0; i < 4; i++) {
		pwm[i] = constrain(pwm[i], PWM_ZERO, PWM_MAX);
		esc[i].writeMicroseconds(pwm[i]);
	}
}

void setup()
{
	analogReadResolution(12);

	esc_setup();
//AP - accel stuff
	Serial.begin(SERIAL_PORT_SPEED);
	Serial.print("Arduino9150 starting using device ");Serial.println(DEVICE_TO_USE);
	Wire.begin();
	magCalSetup();
	accelCalSetup();
	mpuInit();
	pollInterval = (1000 / MPU_UPDATE_RATE) - 1; // a bit less than the minimum interval
	lastPollTime = millis();	
	
	//AP - last thing, loiter for a while
	delay(30000); 
	PID_setup();
	flight_st = FS_FLIGHT;

}

void loop()
{  
	
	
	if ((duePoll())&&(flight_st == FS_FLIGHT))    // get the latest data if ready yet
	{
		radio_alt = radio_altitude();
		//dueMPU.printAngles(dueMPU.m_fusedEulerPose);          // print the output of the data fusion


		x_axis = dueMPU.m_fusedEulerPose[VEC3_X]*RAD_TO_DEGREE;
		y_axis = dueMPU.m_fusedEulerPose[VEC3_Y]*RAD_TO_DEGREE;
		z_axis = dueMPU.m_fusedEulerPose[VEC3_Z]*RAD_TO_DEGREE;
		
		// Compute PIDs
		alt_PID.Compute();
		x_PID.Compute();
		y_PID.Compute();
		z_PID.Compute();
	
		
 //AP - motor control

		//for (int i = 0; i < 4; i++)
			//pwm[i] = PWM_ZERO + alt_pwm;
		pwm[0] = PWM_ZERO + alt_pwm + x_pwm;
		pwm[1] = PWM_ZERO + alt_pwm + y_pwm;
		pwm[2] = PWM_ZERO + alt_pwm - x_pwm;
		pwm[3] = PWM_ZERO + alt_pwm - y_pwm;
		
		//AP - if angle is too large, shut down
		if ((abs(x_axis) > MAX_ANGLE)||(abs(y_axis) > MAX_ANGLE)){
			pwm[0] = PWM_ZERO;
			pwm[1] = PWM_ZERO;
			pwm[2] = PWM_ZERO;
			pwm[3] = PWM_ZERO;
			flight_st = FS_ERROR;
		}
		update_speeds(pwm);
		
		//Serial.print(pwm[0]);
		//Serial.print("; ");
		//Serial.println(x_axis);
		
	} //end duePoll() loop

}


void magCalSetup(void)
{
  calLibRead(DEVICE_TO_USE, &calData);
  calData.magValid = true;
  calData.magMinX = -142;                                // init mag cal data
  calData.magMaxX = 42;
  calData.magMinY = 26;                              
  calData.magMaxY = 199;
  calData.magMinZ = -304;                             
  calData.magMaxZ = -108; 
  calLibWrite(DEVICE_TO_USE, &calData);
}

void accelCalSetup(void)
{
  calLibRead(DEVICE_TO_USE, &calData);
  calData.accelValid = true;
  calData.accelMinX = -12666;                                // init mag cal data
  calData.accelMaxX = 13536;
  calData.accelMinY = -12522;                              
  calData.accelMaxY = 14488;
  calData.accelMinZ = -16558;                             
  calData.accelMaxZ = -17888; 
  calLibWrite(DEVICE_TO_USE, &calData);
}
