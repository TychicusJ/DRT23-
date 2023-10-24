#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <math.h>


/*
   This sample sketch demonstrates the normal use of a TinyGPS++(TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 51(rx) and 49(tx) and a QMC5883 Magnetic Compass
   connected to the SCL/SDA pins.
*/

static const int RXPin = 51 , TXPin = 49; // RX and TX pins are reverse on the board RX goto 49 and TX goto 51 
static const uint32_t GPSBaud = 9600;

// Assign a Unique ID to the QMC5883 Compass Sensor
QMC5883LCompass compass;


// The TinyGPSPlus object
TinyGPSPlus gps; // The GT-U7 gps 

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);


void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  compass.init();

  Serial.println(F("Simple Test with TinyGPS++ and attached GT-U7 GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();
  
}

void loop()
{
  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayGpsInfo();
}

void displayGpsInfo()
{
  //Prints the location if lat-lng information was recieved
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  //Prints invalid if no information was recieved in regards to location.
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  //Prints the recieved GPS module date if it was decoded in a vaild response.
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    //Prints invalid oterwise.
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  // prints the recieved GPS Module time if it was decoded in a valid response.
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    //Print invalid oterwise.
    Serial.print(F("INVALID"));

  }

  Serial.println();
  {
    displayCompassInfo();
  }
}

void displayCompassInfo()
{
  // all function for compass
  	int x, y, z, a, b;
	char myArray[3];
	
	compass.read();
  
	x = compass.getX();
	y = compass.getY();
	z = compass.getZ();
	
	a = compass.getAzimuth();
	
	b = compass.getBearing(a);

	compass.getDirection(myArray, a);
  
  
	Serial.print("X: ");
	Serial.print(x);

	Serial.print(" Y: ");
	Serial.print(y);

	Serial.print(" Z: ");
	Serial.print(z);

	Serial.print(" Azimuth: ");
	Serial.print(a);

	Serial.print(" Bearing: ");
	Serial.print(b);

	Serial.print(" Direction: ");
	Serial.print(myArray[0]);
	Serial.print(myArray[1]);
	Serial.print(myArray[2]);

	Serial.println();

	delay(3000);
}

