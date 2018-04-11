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
	if(SSTransceiver){
		SSTransceiver->print(TeamID); SSTransceiver->print(",");
		SSTransceiver->print(MET); SSTransceiver->print(",");
		SSTransceiver->print(PackCount); SSTransceiver->print(",");
		SSTransceiver->print(bmpAltitude); SSTransceiver->print(",");
		SSTransceiver->print(bmpPressure); SSTransceiver->print(",");
		SSTransceiver->print(lmTemp); SSTransceiver->print(",");
		SSTransceiver->print(inaVoltage); SSTransceiver->print(",");
		SSTransceiver->print(GPSTime); SSTransceiver->print(",");
		SSTransceiver->print(GPSLat); SSTransceiver->print(",");
		SSTransceiver->print(GPSLong); SSTransceiver->print(",");
		SSTransceiver->print(GPSAlt); SSTransceiver->print(",");
		SSTransceiver->print(GPSSats); SSTransceiver->print(",");
		SSTransceiver->print(bnoTiltX); SSTransceiver->print(",");
		SSTransceiver->print(bnoTiltY); SSTransceiver->print(",");
		SSTransceiver->print(bnoTiltZ); SSTransceiver->print(",");
		SSTransceiver->println(State);
	}
	else {
		HSTransceiver->print(TeamID); HSTransceiver->print(",");
		HSTransceiver->print(MET); HSTransceiver->print(",");
		HSTransceiver->print(PackCount); HSTransceiver->print(",");
		HSTransceiver->print(bmpAltitude); HSTransceiver->print(",");
		HSTransceiver->print(bmpPressure); HSTransceiver->print(",");
		HSTransceiver->print(lmTemp); HSTransceiver->print(",");
		HSTransceiver->print(inaVoltage); HSTransceiver->print(",");
		HSTransceiver->print(GPSTime); HSTransceiver->print(",");
		HSTransceiver->print(GPSLat); HSTransceiver->print(",");
		HSTransceiver->print(GPSLong); HSTransceiver->print(",");
		HSTransceiver->print(GPSAlt); HSTransceiver->print(",");
		HSTransceiver->print(GPSSats); HSTransceiver->print(",");
		HSTransceiver->print(bnoTiltX); HSTransceiver->print(",");
		HSTransceiver->print(bnoTiltY); HSTransceiver->print(",");
		HSTransceiver->print(bnoTiltZ); HSTransceiver->print(",");
		HSTransceiver->println(State);
	}
}

bool FSWCDH_CS::dataLog() 
{
  dataFile = SD.open(fileName, FILE_WRITE);
  if (dataFile)
  {
    dataFile.print(TeamID); dataFile.print(",");
	dataFile.print(MET); dataFile.print(",");
	dataFile.print(PackCount); dataFile.print(",");
	dataFile.print(bmpAltitude); dataFile.print(",");
	dataFile.print(bmpPressure); dataFile.print(",");
	dataFile.print(lmTemp); dataFile.print(",");
	dataFile.print(inaVoltage); dataFile.print(",");
	dataFile.print(GPSTime); dataFile.print(",");
	dataFile.print(GPSLat); dataFile.print(",");
	dataFile.print(GPSLong); dataFile.print(",");
	dataFile.print(GPSAlt); dataFile.print(",");
	dataFile.print(GPSSats); dataFile.print(",");
	dataFile.print(bnoTiltX); dataFile.print(",");
	dataFile.print(bnoTiltY); dataFile.print(",");
	dataFile.print(bnoTiltZ); dataFile.print(",");
	dataFile.println(State);
    dataFile.close();
    return true;
  } else {
    return false;
  }
}

void FSWCDH_CS::updateMET(unsigned long time)
{
	MET = time - METStart;
}

