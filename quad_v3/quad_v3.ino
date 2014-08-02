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
#include <SFE_BMP180.h>


//Constants
#define PWM_ZERO 1500
#define SENSOR_VOLTAGE 3.3
#define PWM_MAX 2000
#define RADIO_MAX 100
#define RADIO_MIN 20
#define MAX_ANGLE 30

//Serial port, use this if using bluetooth board on serial1
//#define Serial Serial1

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


// Barometer variables
SFE_BMP180 pressure;
double baseline; // baseline pressure

float altitude_error_i, acc_scale, altitude_error, inst_error;
float inst_acceleration, delta, estimated_velocity, estimated_altitude;
float last_orig_altitude, last_estimated_altitude;
float FACTOR, KP1, KP2, KI, dt;

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
	alt_init();
	
	if (!pressure.begin())
		Serial.println("Barometer not working");
	
	//AP - last thing, loiter for a while
	delay(10000); 
	baseline = getPressure();
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
		
		altitude_update();
		
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
		Serial1.println(x_axis);
		
	} //end duePoll() loop

}

void takeoffMode()
{
	alt_PID.SetTunings(5,5,0.1);
	x_PID.SetTunings(1,5,0.5);
	y_PID.SetTunings(1,5,0.5);
	z_PID.SetTunings(1,5,0.5);
	desired_alt = 50;
}

void alt_init()
{
	FACTOR = 1;
	KP1 = 0.55*FACTOR; // PI observer velocity gain
	KP2 = 1.0*FACTOR; // PI observer position gain
	KI = 0.001/FACTOR; // PI observer integral gain (bias cancellation)

	altitude_error_i = 0;
	acc_scale = 0.0;
	altitude_error = 0;
	inst_acceleration = 0;
	delta = 0;
	estimated_velocity = 0;
	estimated_altitude = 0;

	last_orig_altitude = 0;
	last_estimated_altitude = 0;
	dt = 0.05;
}

void altitude_update()
{
	MPUQuaternion compensated_acc_q, compensated_acc_q_earth, q;
	q[0] = dueMPU.m_fusedQuaternion[0];
	q[1] = dueMPU.m_fusedQuaternion[1];
	q[2] = dueMPU.m_fusedQuaternion[2];
	q[3] = dueMPU.m_fusedQuaternion[3];
	MPUVector3 acc;
	acc[0] = (float) dueMPU.m_calAccel[0];
	acc[1] = (float) dueMPU.m_calAccel[0];
	acc[2] = (float) dueMPU.m_calAccel[0];
	
	double P = getPressure();
	float alt = (float) pressure.altitude(P,baseline)/100.0;
	
	Serial.print("Pressure: ")
	Serial.print(P);
	Serial.print("Baro alt: ");
	Serial.print(alt);

	compute_compensated_acc(q, acc, compensated_acc_q);
	compute_dynamic_acceleration_vector(q, compensated_acc_q, compensated_acc_q_earth);

	last_orig_altitude = alt;
	last_estimated_altitude = compute_altitude(compensated_acc_q_earth[3], alt);
	Serial.print("Est Alt: ");
	Serial.println(last_estimated_altitude);
}

// Remove gravity from accelerometer measurements
void compute_compensated_acc(MPUQuaternion q, MPUVector3 a, MPUQuaternion comp_a)
{
	MPUVector3 g;
	g[0] = 2*(q[1]*q[3] - q[0]*q[2]);
	g[1] = 2*(q[0]*q[1] + q[2]*q[3]);
	g[2] = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];

	comp_a[0] = 0.0;
	comp_a[1] = a[0]/1000.0 - g[0];
	comp_a[2] = a[1]/1000.0 - g[1];
	comp_a[3] = a[2]/1000.0 - g[2];
}

// Rotate dynamic acceleration vector from sensor frame to earth frame
void compute_dynamic_acceleration_vector(const MPUQuaternion q, const MPUQuaternion compensated_acc_q, MPUQuaternion compensated_acc_q_earth)
{
	MPUQuaternion q_conj, tmp;
	MPUQuaternionConjugate(q, q_conj);

	MPUQuaternionMultiply(q, compensated_acc_q, tmp);
	MPUQuaternionMultiply(tmp, q_conj, compensated_acc_q_earth);
	/*
	float tmp[4];
	tmp[0] = q[3]*acc_q[3] - q[0]*acc_q[0] - q[1]*acc_q[1] - q[2]*acc_q[2];
	tmp[1] = q[3]*acc_q[0] + q[0]*acc_q[3] + q[1]*acc_q[2] - q[2]*acc_q[1];
	tmp[2] = 
        def q_mult(q1, q2):
            x1, y1, z1, w1 = q1
            x2, y2, z2, w2 = q2
            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
            z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

            return x, y, z, w
	
        tmp = q_mult(q, compensated_acc_q)
        return q_mult(tmp, q_conj(q))
	*/
}

// Computes estimation of altitude based on barometer and accelerometer measurements
// Code is based on blog post from Fabio Varesano: http://www.varesano.net/blog/fabio/complementary-filtering-high-res-barometer-and-accelerometer-reliable-altitude-estimation
// He seems to have got the idea from the MultiWii project
float compute_altitude(float compensated_acceleration, float altitude)
{
/*
	# Initialization
	if not self.initialized:
		self.initialized = True
		self.estimated_altitude = altitude
		self.estimated_velocity = 0
		self.altitude_error_i = 0
*/
	// Estimation Error
	altitude_error = altitude - estimated_altitude;
	altitude_error_i = altitude_error_i + altitude_error;
	altitude_error_i = constrain(altitude_error_i, -2500.0, 2500.0);

	inst_acceleration = compensated_acceleration * 9.80665 + altitude_error_i * KI;
	
	// Integrators
	delta = inst_acceleration * dt + (KP1 * dt) * altitude_error;
	estimated_altitude += (estimated_velocity/5.0 + delta) * (dt / 2) + (KP2 * dt) * altitude_error;
	estimated_velocity += delta*10.0;
	
	return estimated_altitude;
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

double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
