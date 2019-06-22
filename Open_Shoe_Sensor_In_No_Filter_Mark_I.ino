#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_10DOF.h>

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(20);

int time_differential = 10;
int sample_epoch = 0;
float yaw;
float pitch;
float roll;
float A_x;
float A_y;
float A_z;
float W_x;
float W_y;
float W_z;

sensors_event_t accel_event;
sensors_event_t gyro_event;
sensors_event_t mag_event;
sensors_vec_t   orientation;

void displaySensorDetails_Accel(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
}

void displaySensorDetails_Gyro(void)
{
  sensor_t sensor;
  gyro.getSensor(&sensor);
}

void initSensors()
{
  if(!accel.begin())
  {
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!gyro.begin())
  {
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
}

void setup() 
{
  Serial.begin(115200);
  gyro.enableAutoRange(true);
  initSensors();
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
  Serial.println(yaw); 
  Serial.println(pitch); 
  Serial.println(roll);
}

void loop(void) 
{
  accel.getEvent(&accel_event);
  A_x = accel_event.acceleration.x;
  A_y = accel_event.acceleration.y;
  A_z = accel_event.acceleration.z;
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
  gyro.getEvent(&gyro_event);
  W_x = gyro_event.gyro.x;
  W_y = gyro_event.gyro.y;
  W_z = gyro_event.gyro.z;

  Serial.println(A_x);
  Serial.println(A_y);
  Serial.println(A_z);
  Serial.println(yaw);
  Serial.println(pitch);
  Serial.println(roll);
  Serial.println(W_x);
  Serial.println(W_y);
  Serial.println(W_z);

  delay(time_differential);
  sample_epoch++; 
}
