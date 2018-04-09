/* CanSat Flight Software (FSW) Alpha (Testing Version) 0.3
   Author: Michael Greer, 2018

   Alpha 0.3: Implements GPS, INA219, BNO055, BMP280 with the Teensy 3.5. Adds Data Handling
              capability using new CDH class.
   - Includes basic data reading and display functions
   - Basic state switching (Non-flight states)
   - Implemented display and CDH classes
      - CDH: Basic Data Handling; Transmission and Logging
      - Display: Basic sensor display 
*/

#include <Wire.h> //I2C Library
#include <Adafruit_INA219.h> //INA219 lib
#include <Adafruit_Sensor.h> //Unified sensor lib
#include <Adafruit_BMP280.h> //BMP280 lib
#include <Adafruit_BNO055.h> //BNO055 lib
#include <utility/imumaths.h> //maths for imu?
#include <TinyGPS.h> //Teensy compatible GPS lib
#include <FSWCDH_CS.h> //Custom CDH lib
#include <DisplayCS.h> //Custom display lib

TinyGPS gps;
FSWCDH_CS cdh;
DisplayCS disp(&cdh);

#define DISTIME 0
#define PLOTRATE 10 //time
#define GPS_timeout 100
#define gpsPort Serial1
// turn on GPRMC and GGA
#define PMTK_SET_NMEA_OUTPUT_RMCGGA "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28" 

enum FSWState { //The k's make the IDE happier than all caps apparently
  kDISPLAY,
  kINAPLOT,
  kBMPPLOT,
  kACCELPLOT,
  kORIENTPLOT
};

const int chipSelect = -1; //To turn off data logging (debugging)
//const int chipSelect = BUILTIN_SDCARD; //SD card chipselect variable
const char* fileName = "TestFile.txt"; //SD card file name

enum FSWState swState = kDISPLAY;
int sTime = DISTIME; //Set initial display rate
bool newData; //GPS new data check
unsigned long fix_age; //GPS fix age check
unsigned long GPSDate;

double localPress = 1024.04; //Local air pressure in hectapascals

Adafruit_INA219 ina219; //initialize ina219
Adafruit_BMP280 bmp; //initialize bmp280
Adafruit_BNO055 bno = Adafruit_BNO055(55);


/****************************************************************/
/*                        Setup & Loop                          */
/****************************************************************/


void setup(void)
{
  Serial.begin(115200);
  while (!Serial) {
    // will pause Zero, Leonardo, etc until serial console opens
    delay(1);
  }
  if (!cdh.initCDH(fileName, chipSelect)){ //initialize CDH
    Serial.println("Problem initializing Communication & Data Handling System");
    while(1);
  }
  gpsPort.begin(9600);
  gpsPort.println(PMTK_SET_NMEA_OUTPUT_RMCGGA); //Send GPS setup command

  uint32_t currentFrequency;

  Serial.println("Starting up INA219...");
  ina219.begin();
  // Initialize the INA219.
  // By default the initialization will use the largest range (32V, 2A).  However
  // you can call a setCalibration function to change this range (see comments).
  Serial.println("Starting Power measurements...");
  // To use a slightly lower 32V, 1A range (higher precision on amps):
  //ina219.setCalibration_32V_1A();
  // Or to use a lower 16V, 400mA range (higher precision on volts and amps):
  //ina219.setCalibration_16V_400mA();
  Serial.println("Measuring voltage and current with INA219 ...");


  Serial.println("Starting up BMP280...");
  if (!bmp.begin()) {
    //Initialize bmp280
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  }
  else {
    Serial.println("Starting Pressure/Temperature measurements...");
  }

  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  else
  {
    /* Display some basic information on this sensor */
    displaySensorDetails();

    /* Optional: Display current status */
    displaySensorStatus();

    bno.setExtCrystalUse(true);
  }

  //swState = DISPLAY; //set initial software state
  syncGPS();
}

void loop(void)
{
  //State check:
  stateCheck();

  switch (swState) {
    case kDISPLAY:
      syncGPS();
      readGPS();
      readBNO();
      readBMP();
      readINA();
      disp.bnoDisplay();
      disp.bmpDisplay();
      disp.inaDisplay();
      break;
    case kINAPLOT:
      inaPlot();
      break;
    case kBMPPLOT:
      bmpPlot();
      break;
    case kACCELPLOT:
      accelPlot();
      break;
    case kORIENTPLOT:
      orientPlot();
      break;
  }

  delay(sTime);
}


/****************************************************************/
/*                         Functions                            */
/****************************************************************/

void readBNO()
{
  //Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);

  cdh.bnoTiltX = event.orientation.x;
  cdh.bnoTiltY = event.orientation.y;
  cdh.bnoTiltZ = event.orientation.z;

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
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  cdh.bnoAccelX = accel.x();
  cdh.bnoAccelY = accel.y();
  cdh.bnoAccelZ = accel.z();
}

void readBMP()
{
  cdh.lmTemp = bmp.readTemperature();
  cdh.bmpPressure = bmp.readPressure();
  cdh.bmpAltitude = bmp.readAltitude(localPress);
}

void readINA()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  cdh.inaVoltage = loadvoltage;
  cdh.inaCurrent = current_mA;
  cdh.inaPower = power_mW;
}

void syncGPS()
{
  while (!gpsPort.available()) {}
}

void readGPS()
{
  Serial.println("Raw GPS Data: ");
  for (unsigned long start = millis(); millis() - start < GPS_timeout;) {
    while (gpsPort.available()) {
      char c = gpsPort.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
      {
        newData = true;
        gps.f_get_position(&cdh.GPSLat, &cdh.GPSLong, &fix_age);
        cdh.GPSAlt = gps.f_altitude();
        cdh.GPSSats = gps.satellites();
        gps.get_datetime(&GPSDate, &cdh.MET, &fix_age);
      }
    }
  }
  Serial.println();
}

void stateCheck()
{
  if (Serial.available() > 0) {
    String input = Serial.readString();
    Serial.print("String received: "); Serial.println(input);
    if (input == "DISPLAY")
    {
      Serial.println("\nDisplay Mode On\n");
      delay(1000);
      swState = kDISPLAY;
      sTime = DISTIME;
    }
    if (input == "INAPLOT")
    {
      Serial.println("\nPower Plot Mode On\n");
      delay(1000);
      swState = kINAPLOT;
      sTime = PLOTRATE;
    }
    if (input == "BMPPLOT")
    {
      Serial.println("\nBMP Plot Mode On\n");
      delay(1000);
      swState = kBMPPLOT;
      sTime = PLOTRATE;
    }
    if (input == "ACCELPLOT")
    {
      Serial.println("\nIMU Acceleration Plot Mode On\n");
      delay(1000);
      swState = kACCELPLOT;
      sTime = PLOTRATE;
    }
    if (input == "ORIENTPLOT")
    {
      Serial.println("\nIMU Orientation Plot Mode On\n");
      delay(1000);
      swState = kORIENTPLOT;
      sTime = PLOTRATE;
    }
    if (input == "help" | input == "HELP")
    {
      Serial.println("Type in any of the following commands to change the software state:");
      Serial.println(" ");
      Serial.println("DISPLAY - Displays all sensor data at a low sample rate");
      Serial.println("INAPLOT - For power plotting using INA219 data");
      Serial.println("BMPPLOT - For pressure plotting using BMP280 data");
      Serial.println("ACCELPLOT - For acceleration plotting using IMU (BNO055) data");
      Serial.println("ORIENTPLOT - For orientation plotting using IMU (BNO055) data");
      Serial.println("\nReturning to program in 10 seconds...\n");
      delay(10000);
    }
    //    else
    //    {
    //      Serial.println("\nInvalid Entry. Please input a State name\n");
    //      delay(2000);
    //    }
  }
}

/*************************Plot Functions****************************/

void accelPlot()
{
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  /* Display the floating point data */
  Serial.print(accel.x()); Serial.print(" ");
  Serial.print(accel.y()); Serial.print(" ");
  Serial.print(accel.z()); Serial.println(" ");
}

void orientPlot()
{
  //Get a new sensor event
  sensors_event_t event;
  bno.getEvent(&event);

  //Display the floating point data
  Serial.print(event.orientation.x, 4); Serial.print(" ");
  Serial.print(event.orientation.y, 4); Serial.print(" ");
  Serial.print(event.orientation.z, 4); Serial.println(" ");
}

void bmpPlot()
{
  //Serial.print(bmp.readTemperature()); Serial.print(" ");
  Serial.print(bmp.readPressure()); Serial.println(" ");
  //Serial.println(bmp.readAltitude(localPress));
}

void inaPlot()
{
  float shuntvoltage = 0;
  float busvoltage = 0;
  float current_mA = 0;
  float loadvoltage = 0;
  float power_mW = 0;

  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();
  loadvoltage = busvoltage + (shuntvoltage / 1000);

  Serial.print(shuntvoltage); Serial.print(" ");
  Serial.print(busvoltage); Serial.print(" ");
  Serial.print(current_mA); Serial.print(" ");
  Serial.println(power_mW);
}

//BNO Stuff:

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

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
  bno.getSensor(&sensor);
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
  bno.getCalibration(&system, &gyro, &accel, &mag);

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
