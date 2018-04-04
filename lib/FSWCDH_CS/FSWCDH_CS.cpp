/*Communication and Data Handling Class - CanSat
 * SOURCE
 * 
 * 
 */

#include <FSWCDH_CS.h>

FSWCDH_CS::FSWCDH_CS() //Default ctor
{}

FSWCDH_CS::FSWCDH_CS(SoftwareSerial *ser) //ctor for SWSerial
{
  SStransceiver = ser;
}

FSWCDH_CS::FSWCDH_CS(HardwareSerial *ser) //ctor for HWSerial
{
  HStransceiver = ser;
}

bool FSWCDH_CS::initCDH(const char* name, const int chipselect)
{
  fileName = name;
  if (chipselect >= 0){
	return SD.begin(chipselect);
  } else { return true;}
}

void FSWCDH_CS::dataTransmit()
{

}

bool FSWCDH_CS::dataLog() 
{
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile)
  {
    dataFile.println("Placeholder");
    dataFile.close();
    return true;
  } else {
    return false;
  }
}

