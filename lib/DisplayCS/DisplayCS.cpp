#include "DisplayCS.h"



DisplayCS::DisplayCS()
{

}


DisplayCS::DisplayCS(FSWCDH_CS *CDH)
{
  cdh = CDH;
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
//  //Serial.print(bmp.readTemperature()); Serial.print(" ");
//  Serial.print(bmp.readPressure()); Serial.println(" ");
//  //Serial.println(bmp.readAltitude(localPress));
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

void DisplayCS::bmpDisplay()
{
  Serial.print("Temperature = ");
  Serial.print(cdh->lmTemp);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(cdh->bmpPressure);
  Serial.println(" Pa");

  Serial.print("Approx altitude = ");
  Serial.print(cdh->bmpAltitude);
  //Serial.print(bmp.readAltitude(localPress)); // this should be adjusted to your local forcast
  Serial.println(" m");

  Serial.println();
}

void DisplayCS::inaDisplay()
{
  Serial.print("Load Voltage:  "); Serial.print(cdh->inaVoltage); Serial.println(" V");
  Serial.print("Current:       "); Serial.print(cdh->inaCurrent); Serial.println(" mA");
  Serial.print("Power:         "); Serial.print(cdh->inaPower); Serial.println(" mW");
  Serial.println("");
}


void DisplayCS::bnoDisplay()
{
  //Display the floating point data
  Serial.println("IMU Orientation:");
  Serial.print("X: ");
  //Serial.print(event.orientation.x, 4);
  Serial.print(cdh->bnoTiltX, 4);
  Serial.print("\tY: ");
  //Serial.print(event.orientation.y, 4);
  Serial.print(cdh->bnoTiltY, 4);
  Serial.print("\tZ: ");
  //Serial.print(event.orientation.z, 4);
  Serial.print(cdh->bnoTiltZ, 4);
  /* New line for the next sample */
  Serial.println("");

  /* Display the floating point data */
  Serial.println("IMU Acceleration:");
  Serial.print("X: ");
  Serial.print(cdh->bnoAccelX);
  Serial.print(" Y: ");
  Serial.print(cdh->bnoAccelY);
  Serial.print(" Z: ");
  Serial.print(cdh->bnoAccelZ);
  Serial.println("");
  Serial.println("");

  /* Optional: Display calibration status */
  //displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();



}

////BNO Stuff:
//
//void displaySensorStatus(void)
//{
//  /* Get the system status values (mostly for debugging purposes) */
//  uint8_t system_status, self_test_results, system_error;
//  system_status = self_test_results = system_error = 0;
//  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
//
//  /* Display the results in the Serial Monitor */
//  Serial.println("");
//  Serial.print("System Status: 0x");
//  Serial.println(system_status, HEX);
//  Serial.print("Self Test:     0x");
//  Serial.println(self_test_results, HEX);
//  Serial.print("System Error:  0x");
//  Serial.println(system_error, HEX);
//  Serial.println("");
//  delay(500);
//}
//
//void displaySensorDetails(void)
//{
//  sensor_t sensor;
//  bno.getSensor(&sensor);
//  Serial.println("------------------------------------");
//  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
//  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
//  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
//  Serial.println("------------------------------------");
//  Serial.println("");
//  delay(500);
//}
//
//void displayCalStatus(void)
//{
//  /* Get the four calibration values (0..3) */
//  /* Any sensor data reporting 0 should be ignored, */
//  /* 3 means 'fully calibrated" */
//  uint8_t system, gyro, accel, mag;
//  system = gyro = accel = mag = 0;
//  bno.getCalibration(&system, &gyro, &accel, &mag);
//
//  /* The data should be ignored until the system calibration is > 0 */
//  Serial.print("\t");
//  if (!system)
//  {
//    Serial.print("! ");
//  }
//
//  /* Display the individual values */
//  Serial.print("Sys:");
//  Serial.print(system, DEC);
//  Serial.print(" G:");
//  Serial.print(gyro, DEC);
//  Serial.print(" A:");
//  Serial.print(accel, DEC);
//  Serial.print(" M:");
//  Serial.print(mag, DEC);
//}
