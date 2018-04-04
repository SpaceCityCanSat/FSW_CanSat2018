/*Communication and Data Handling Class - CanSat
 * SOURCE
 * 
 * 
 */

#include <FSWCDH.h>

FSWCDH_CS::FSWCDH_CS(SoftwareSerial *ser) //ctor for SWSerial
{
  SSTransceiver = ser;
}

FSWCDH_CS::FSWCDH_CS(SoftwareSerial *ser) //ctor for HWSerial
{
  HSTransceiver = ser;
}

bool FSWCDH_CS::initCDH(String name, const int chipselect)
{
  fileName = name;
  return SD.begin(chipselect);
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

