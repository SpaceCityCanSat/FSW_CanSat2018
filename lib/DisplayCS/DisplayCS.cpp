#include "DisplayCS.h"



DisplayCS::DisplayCS()
{

}


DisplayCS::DisplayCS(FSWCDH_CS *_cdh, Adafruit_INA219 *_ina219, Adafruit_BMP280 *_bmp, Adafruit_BNO055 *_bno)
{
  cdh = _cdh;
  ina219 = _ina219;
  bmp = _bmp;
  bno = _bno;
}

//void accelPlot()
//{
//  /* Display the floating point data */
//  Serial.print(accel.x()); Serial.print(" ");
//  Serial.print(accel.y()); Serial.print(" ");
//  Serial.print(accel.z()); Serial.println(" ");
//}
//
//void orientPlot()
//{
//  //Display the floating point data
//  Serial.print(event.orientation.x, 4); Serial.print(" ");
//  Serial.print(event.orientation.y, 4); Serial.print(" ");
//  Serial.print(event.orientation.z, 4); Serial.println(" ");
//}
//
//void bmpPlot()
//{
//  //Serial.print(bmp->readTemperature()); Serial.print(" ");
//  Serial.print(bmp->readPressure()); Serial.println(" ");
//  //Serial.println(bmp->readAltitude(localPress));
//}
//
//void inaPlot()
//{
//
//  Serial.print(shuntvoltage); Serial.print(" ");
//  Serial.print(busvoltage); Serial.print(" ");
//  Serial.print(current_mA); Serial.print(" ");
//  Serial.println(power_mW);
//}

/************************Display Functions**************************/

void DisplayCS::bmp()
{
  Serial.print("Temperature = ");
  Serial.print(bmp->readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp->readPressure());
  Serial.println(" Pa");

  Serial.print("Approx altitude = ");
  Serial.print(bmp->readAltitude());
  //Serial.print(bmp->readAltitude(localPress)); // this should be adjusted to your local forcast
  Serial.println(" m");

  Serial.println();
}

void DisplayCS::ina()
{
	float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219->getShuntVoltage_mV();
  busvoltage = ina219->getBusVoltage_V();
  current_mA = ina219->getCurrent_mA();
  power_mW = ina219->getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);
  
  Serial.print("Load Voltage:  "); Serial.print(loadvoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(power_mW); Serial.println(" mW");
  Serial.println("");
}


void DisplayCS::bno()
{
	//Get a new sensor event
  sensors_event_t event;
  bno->getEvent(&event);

  /*
    The key raw data functions are:

    getVector (adafruit_vector_type_t vector_type)
    getQuat (void)
    getTemp (void)
    .getVector ( adafruit_vector_type_t vector_type )
    The .getVector function accepts a single parameter (vector_type), which indicates what type of 3-axis vector data to return.

    The vector_type field can be one of the following values:

    VECTOR_MAGNETOMETER (values in uT, micro Teslas)
    VECTOR_GYROSCOPE (values in rps, radians per second)
    VECTOR_EULER (values in Euler angles or 'degrees', from 0..359)
    VECTOR_ACCELEROMETER (values in m/s^2)
    VECTOR_LINEARACCEL (values in m/s^2)
    VECTOR_GRAVITY (values in m/s^2)
  */
  imu::Vector<3> accel = bno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	
  //Display the floating point data
  Serial.println("IMU Orientation:");
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  /* New line for the next sample */
  Serial.println("");

  /* Display the floating point data */
  Serial.println("IMU Acceleration:");
  Serial.print("X: ");
  Serial.print(accel.x());
  Serial.print(" Y: ");
  Serial.print(accel.y());
  Serial.print(" Z: ");
  Serial.print(accel.z());
  Serial.println("");
  Serial.println("");

  /* Optional: Display calibration status */
  //displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();



}

void DisplayCS::met()
{
	
	Serial.print("\nCurrent Mission Elapsed Time in seconds: ");
	Serial.println(cdh->MET);
}

//BNO Stuff:

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno->getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno->getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno->getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}