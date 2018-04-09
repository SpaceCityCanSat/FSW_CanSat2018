# FSW_CanSat2018 A0.3 (Alpha - Testing Version)
Flight Software for Space City CanSat's 2018 Project
## Alpha Version
The Alpha version dedicated to the unit and integration testing of the various CanSat modules.

### Change Notes
#### Alpha 0.3
* Implemented data structure to store various sensor values
* Added dedicated sensor read functions
* Refactored CDH and Display code into dedicated libraries to decrease clutter
* Implemented prototype functions for data logging and transmission

# Function Breakdown
## CanSat_FSW_Alpha
Main File
### Arduino Main Functions
* void setup(void)
* void loop(void)
### Sensor Read Functions
* void readBNO()
* void readBMP()
* void readINA()
* void syncGPS()
* void readGPS()
### State Functions
* void stateCheck()
### BNO Specific Display Functions
* void displaySensorStatus(void)
* void displaySensorDetails(void)
* void displayCalStatus(void)
### Plot Functions (For Arduino IDE Plotter)
* void accelPlot()
* void orientPlot()
* void bmpPlot()
* void inaPlot()

## DisplayCS
FSW Display Library
* void bmpDisplay()
* void inaDisplay()
* void bnoDisplay()

## FSWCDH_CS
Commmunication & Data Handling (CDH) Library
* bool initCDH(const char* name, const int chipselect)
* void dataTransmit()
* bool dataLog()
