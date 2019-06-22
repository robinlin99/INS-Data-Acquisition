/* Include necessary libraries */
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

/* Define time_differential (dt) and sample_epoch (sample period) variables */
// 100 milliseconds
int time_differential = 100;
// Initial sample_epoch is set to 0
int sample_epoch = 0;

int yaw;
int pitch;
int roll;

// Define events
sensors_event_t accel_event;
sensors_event_t gyro_event;
sensors_event_t mag_event;
sensors_event_t bmp_event;
sensors_vec_t   orientation;

/* Define function displaying accelerometer details */
void displaySensorDetails_Accel(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
}

/* Define function displaying gyroscope details */
void displaySensorDetails_Gyro(void)
{
  sensor_t sensor;
  gyro.getSensor(&sensor);
}

float a = 1.0;

/* Define sensor initiation function */
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
}

// Time index t = k
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(9600);
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  /* Initialise all sensors */
  initSensors();
  /*Display details*/
  displaySensorDetails_Accel();
  displaySensorDetails_Gyro();
  
  /* Receive initial conditions for euler's angle */
  /* Other initial conditions defined in MATLAB script */
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    roll = orientation.roll;
    pitch = orientation.pitch;
  }
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    yaw = orientation.heading;
  }

  // Send one decimal integer to be read by MATLAB script, initiating update cycle algorithm
  Serial.print(a);

}

// Time index t = k + 1 and beyond
void loop(void) 
{
  /* Update state vector algorithm */
  // put your main code here, to run repeatedly:
  /* Receive acceleration data */
  accel.getEvent(&accel_event);
  int A_x = accel_event.acceleration.x;
  int A_y = accel_event.acceleration.y;
  int A_z = accel_event.acceleration.z;

  /* Receive euler's angles */
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    roll = orientation.roll;
    pitch = orientation.pitch;
  }

  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    yaw = orientation.heading;
  }

  /* Receive altitude data */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    int altitude = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
  }

  /* Receive gyroscope data */
  gyro.getEvent(&gyro_event);
  int W_x = gyro_event.gyro.x;
  int W_y = gyro_event.gyro.y;
  int W_z = gyro_event.gyro.z;

  /* Delay */
  delay(time_differential);

  /* Increment sample_epoch */
  sample_epoch++;
  
}
