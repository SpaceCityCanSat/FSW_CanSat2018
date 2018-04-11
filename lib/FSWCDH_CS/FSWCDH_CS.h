/*Communication and Data Handling Class - CanSat
 * HEADER
 * 
 * 
 */
#ifndef FSWCDH_CS_h
#define FSWCDH_CS_h


#include <SoftwareSerial.h>
#include <SD.h>
#include <TimeLib.h>


class FSWCDH_CS
{
  public:
    FSWCDH_CS(); //Default ctor
    FSWCDH_CS(SoftwareSerial *ser); //SWS ctor
    FSWCDH_CS(HardwareSerial *ser); //HWS ctor
    bool initCDH(const char* name, const int chipselect); //initializes SD card and sets file name
    void dataTransmit(); //Transmission function (not implemented)
    bool dataLog(); //Logging function
	void updateMET(unsigned long time);
    
    File dataFile; //SD file
    const char* fileName; //SD file name
    SoftwareSerial *SStransceiver; //SW Serial line pointer
    HardwareSerial *HStransceiver; //HW Serial line pointer
	unsigned long METStart; //MET start reference

    struct{//TELEM Data Structure START
    unsigned short TeamID;
    unsigned long MET; //Currently uses GPS time, which is not correct
    unsigned int PacketCount;
    float bmpAltitude;
    float bmpPressure;
    float lmTemp; //Currently uses bmp temp, which is not correct
    float inaVoltage;
    long GPSTime;
    float GPSLat;
    float GPSLong;
    float GPSAlt;
    int GPSSats;
    float bnoTiltX;
    float bnoTiltY;
    float bnoTiltZ;
	char* State;
    } data;//TELEM Data Structure STOP

    struct{//OTHER DATA
    float inaCurrent;
    float inaPower;
    float bnoAccelX;
    float bnoAccelY;
    float bnoAccelZ;
	} data_extra; //could comment out if tight on memory.
	// Or: Implement some sort of flag that determines if it initializes on compile?
    
};

#endif
