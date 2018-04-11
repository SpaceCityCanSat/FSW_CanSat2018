#ifndef DisplayCS_h
#define DisplayCS_h

#include "Arduino.h"
#include "FSWCDH_CS.h"
#include <Adafruit_INA219.h> //INA219 lib
#include <Adafruit_Sensor.h> //Unified sensor lib
#include <Adafruit_BMP280.h> //BMP280 lib
#include <Adafruit_BNO055.h> //BNO055 lib
#include <utility/imumaths.h> //maths for imu?


class DisplayCS
{
  public:
    DisplayCS(); //Default ctor
    DisplayCS(FSWCDH_CS *_cdh, Adafruit_INA219 *_ina219, Adafruit_BMP280 *_bmp, Adafruit_BNO055 *_bno); //Ctor for interfacing with the CDH system

    //Display Functions
    void bmp();
    void ina();
    void bno();
	void met();
    //BNO specific
   void displaySensorStatus(void);
   void displaySensorDetails(void);
   void displayCalStatus(void);
    //Plot Functions
//    void accelPlot();
//    void orientPlot();
//    void bmpPlot();
//    void inaPlot();

    FSWCDH_CS *cdh; //Own pointer to a CDH instance
	Adafruit_INA219 *ina219;
	Adafruit_BMP280	*bmp;
	Adafruit_BNO055 *bno;
};

#endif
