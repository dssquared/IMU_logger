#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SPI.h>
#include <SD.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)
#define cardSelect   4                      // feather board has i/o pin 4 on SD card detect, may be different on other boards
#define statusLED   13                      // using on-board LED for status light, error/feedback codes for SD card functions
#define sdActivity   8                      // feather board has green LED on i/o pin 8, light when writing to card

File logFile;
Adafruit_BNO055 bno = Adafruit_BNO055();

//char filename[15];                         // "string" to save filename to, or hard code it, name set in setup()
String filename = "Logger.txt";


/*** error() fuction to blink out error code on on-board ***/
/***  LED for SD card functions while not using serial   ***/
// 2 blinks - sd card failed or not present
// 3 blinks - sd card could not create or write to log file
void error(uint8_t errno) {
	while(1) {
		uint8_t i;
		for (i=0; i<errno; i++) {
			digitalWrite(statusLED, HIGH);
			delay(100);
			digitalWrite(statusLED, LOW);
			delay(100);
		}
		for (i=errno; i<10; i++) {
			delay(200);
		}
	}
}  // end error()


void setup(){
	
	//strcpy(filename, "ANALOG00.TXT");        // edit length of filename[] if necessary when using different name
	pinMode(statusLED, OUTPUT);
	pinMode(sdActivity, OUTPUT);
	Serial.begin(9600);
	
	/*** see if SD card is present and can be initialized: ***/
	Serial.print("Initializing SD card...");
	if (!SD.begin(cardSelect)) {
		Serial.println("Card failed, or not present");
		error(2);
		// don't do anything more:
		return;
	}
	
	/*
	for (uint8_t i = 0; i < 100; i++) {
		filename[6] = '0' + i/10;
		filename[7] = '0' + i%10;
		// create if does not exist, do not open existing, write, sync after write
		if (! SD.exists(filename)) {
			break;
		}
	}
	*/
	
	// make sure file is created and we can write to it
	logFile = SD.open(filename, FILE_WRITE);
	if( ! logFile ) {
		Serial.print("Couldnt create ");
		Serial.println(filename);
		error(3);
	}
	// if file created and able to write to close for now
	logFile.close();      // close file for now, file needs to be closed before trying to write or open in loop
	                      // will be opening and closing file in loop, **** may use more power this way ****
						  // if leaving file open it will use a 512char buffer and will only write to file when buffer full
						  // you can force a write by closing the file or logFile.flush() but can use upto 30% more power if not using buffer
	Serial.println("card initialized.");
	

	/*** Initialize IMU ***/
	if(!bno.begin())
	{
		/* There was a problem detecting the BNO055 ... check your connections */
		Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
		while(1);
	}

	delay(1000);

	/*** Display the current temperature ***/
	int8_t temp = bno.getTemp();
	Serial.print("Current Temperature: ");
	Serial.print(temp);
	Serial.println(" C");
	Serial.println("");

	bno.setExtCrystalUse(true);
    //Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
	
	Serial.println("Logger ready!");
	
}  // end setup()

void loop(){
	
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  /*
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  // Display the floating point data
  Serial.print("X: ");
  Serial.print(euler.x());
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  */
  
  // accelerometer data test
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Display the floating point data
  Serial.print("X: ");
  Serial.print(acc.x());
  Serial.print(" Y: ");
  Serial.print(acc.y());
  Serial.print(" Z: ");
  Serial.print(acc.z());
  Serial.print("\t\t");
  Serial.println();



  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);

  delay(BNO055_SAMPLERATE_DELAY_MS);
  
  /*** IMU logging ***/
  /*
  logFile = SD.open(filename, FILE_WRITE);         // *** not sure if doing it this way will overwrite data ***
  // if file available write to it
   if (logFile) {
	   // get imu values of interest and write to sd card
	   digitalWrite(sdActivity, HIGH);             // turn on LED to signal sd card activity
	   
	   //logFile.println();
	   logFile.close();
	   digitalWrite(sdActivity, LOW);              // turn of LED
	   
	   // print to the serial port for debug only, remove all print statements when running headless
	   //Serial.println();
   }
   // if the file isn't open, pop up an error:
   else {
	   Serial.print("error opening file ");
	   Serial.println(filename);
	   error(3):
   }
   
   delay(BNO055_SAMPLERATE_DELAY_MS);
   */
  
} // end loop()
