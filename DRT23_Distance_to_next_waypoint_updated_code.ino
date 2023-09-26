#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <QMC5883LCompass.h>
#include <math.h>

double prevLat = 0.0;
double prevLng = 0.0;
double userLat = 0.0;
double userLng = 0.0;


/*
   This sample sketch demonstrates the normal use of a TinyGPS++(TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device hooked up on pins 36(rx) and 39(tx) and a QMC5883 Magnetic Compass
   connected to the SCL/SDA pins.
*/

static const int RXPin = 15, TXPin = 2; // RX and TX pins are reverse on the board RX goto 39 and TX goto 36 
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
  Serial.println(F("Enter the next waypoint coordinates (latitude and longitude) in decimal degrees."));
  Serial.println(F("Use the format: LATITUDE, LONGITUDE"));
}

void loop()
{
  // Check for user input
  if (Serial.available() > 0)
  {
    String userInput = Serial.readStringUntil('\n');
    parseUserInput(userInput);
  }

  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
    if (gps.encode(ss.read()))
      displayGpsInfo();
}

void parseUserInput(String userInput)
{
  int commaIndex = userInput.indexOf(',');
  if (commaIndex != -1)
  {
    String latStr = userInput.substring(0, commaIndex);
    String lngStr = userInput.substring(commaIndex + 1);

    userLat = latStr.toDouble();
    userLng = lngStr.toDouble();

    Serial.println(F("New waypoint coordinates set."));
    Serial.print(F("Latitude: "));
    Serial.print(userLat, 6);
    Serial.print(F(", Longitude: "));
    Serial.println(userLng, 6);
  }
  else
  {
    Serial.println(F("Invalid input format. Use the format: LATITUDE, LONGITUDE"));
  }
}

void displayGpsInfo()
{
  // Print the location if lat-lng information was received
  Serial.print(F("Location: "));
  if (gps.location.isValid())
  {
    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();

    Serial.print(currentLat, 6);
    Serial.print(F(","));
    Serial.print(currentLng, 6);

    if (prevLat != 0.0 && prevLng != 0.0)
    {
      // Calculate distance using Haversine formula
      double distance = calculateDistance(prevLat, prevLng, currentLat, currentLng);
      Serial.print(F("  Distance (meters): "));
      Serial.print(distance);
    }

    if (userLat != 0.0 && userLng != 0.0)
    {
      double userDistance = calculateDistance(currentLat, currentLng, userLat, userLng);
      Serial.print(F("  User Waypoint Distance (meters): "));
      Serial.print(userDistance);
    }

    // Update previous waypoint
    prevLat = currentLat;
    prevLng = currentLng;
  }
  // Prints invalid if no information was received in regards to location.
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
  displayCompassInfo();
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

	delay(1000);
}

double calculateDistance(double lat1, double lon1, double lat2, double lon2)
{
  // Convert latitude and longitude from degrees to radians
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  // Radius of the Earth in meters
  double earthRadius = 6371000.0; // Approximate value for Earth's radius

  // Haversine formula
  double dlon = lon2 - lon1;
  double dlat = lat2 - lat1;
  double a = pow(sin(dlat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(dlon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double distance = earthRadius * c;

  return distance;
}

