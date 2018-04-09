#ifndef DisplayCS_h
#define DisplayCS_h

#include "Arduino.h"
#include "FSWCDH_CS.h"


class DisplayCS
{
  public:
    DisplayCS(); //Default ctor
    DisplayCS(FSWCDH_CS *CDH); //Ctor for interfacing with the CDH system

    //Display Functions
    void bmp();
    void ina();
    void bno();
	void met();
    //BNO specific
//    void displaySensorStatus(void);
//    void displaySensorDetails(void);
//    void displayCalStatus(void);
    //Plot Functions
//    void accelPlot();
//    void orientPlot();
//    void bmpPlot();
//    void inaPlot();

    FSWCDH_CS *cdh; //Own pointer to a CDH instance
};

#endif
