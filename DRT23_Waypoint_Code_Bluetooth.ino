#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <AFMotor.h>                                              // the Adafruit Motor Shield Library ver. 1 https://learn.adafruit.com/adafruit-motor-shield/library-install
#include <QMC5883LCompass.h>
#include <math.h>

double prevLat = 0.0;
double prevLng = 0.0;
double userLat = 0.0;
double userLng = 0.0;
bool waypointSet = false; // Flag to indicate if a waypoint has been set

/*
   This sample sketch demonstrates the normal use of a TinyGPS++(TinyGPSPlus) object.
   It requires the use of Softwarebluetooth, and assumes that you have a
   9600-baud bluetooth GPS device hooked up on pins 51(rx) and 49(tx) and a QMC5883 Magnetic Compass
   connected to the SCL ()/ SDA () pins.
*/

static const int RXPin = 51, TXPin = 49; // RX and TX pins are reverse on the board RX goto 49 and TX goto 51 
static const uint32_t GPSBaud = 9600;
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device
TinyGPSPlus gps; // The TinyGPSPlus object 

// Assign a Unique ID to the QMC5883 Compass Sensor
QMC5883LCompass compass;

static const int Bluetooth_RX = 39, Bluetooth_TX = 37; // RX and TX pins are reverse on the board RX goto 37 and TX goto 39 
static const uint32_t BluetoothBaud = 9600;
SoftwareSerial bluetooth(Bluetooth_RX, Bluetooth_TX);

// Setup Drive Motors using the Adafruit Motor Controller version 1.0 Library

AF_DCMotor motor1(1, MOTOR12_64KHZ);                               // create motor #1, 64KHz pwm
AF_DCMotor motor2(2, MOTOR12_64KHZ);                               // create motor #2, 64KHz pwm
AF_DCMotor motor3(3, MOTOR12_64KHZ);                               // create motor #3, 64KHz pwm
AF_DCMotor motor4(4, MOTOR12_64KHZ);                               // create motor #4, 64KHz pwm

int turn_Speed = 200;                                              // motor speed when using the compass to turn left and right
int mtr_Spd = 255;                                                 // motor speed when moving forward and reverse

const int maxUpdates = 5; // The number of updates to average
double latitudes[maxUpdates] = {0.0};
double longitudes[maxUpdates] = {0.0};
int updateCount = 0;

// Bluetooth Variables & Setup

String str;                                                        // raw string received from android to arduino
int blueToothVal;                                                  // stores the last value sent over via bluetooth
int bt_Pin = 34;                                                   // Pin 34 of the Aruino Mega used to test the Bluetooth connection status - Not Used


void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  compass.init();

  Serial1.begin(9600);                                             // Serial 1 is for Bluetooth communication - DO NOT MODIFY - JY-MCU HC-06 v1.40

  Serial1.println(F("Simple Test with TinyGPS++ and attached GT-U7 GPS module"));
  Serial1.print(F("Testing TinyGPSPlus library v. ")); Serial1.println(TinyGPSPlus::libraryVersion());
  Serial1.println();
  Serial1.println(F("Enter the next waypoint coordinates (latitude and longitude) in decimal degrees."));
  Serial1.println(F("Use the format: LATITUDE, LONGITUDE"));

}

void loop()
{
  // Check for user input
  if (Serial1.available() > 0)
  {
    String userInput = Serial1.readStringUntil('\n');
    parseUserInput(userInput);
  }

  // Check if a waypoint has been set before processing GPS data
  if (waypointSet)
  {
    // This sketch displays information every time a new sentence is correctly encoded.
    while (ss.available() > 0)
    {
      if (gps.encode(ss.read()))
      {
        collectGpsUpdates();
        displayGpsInfo();
        navigateToWaypoint();
      }
    }
  }
}

void collectGpsUpdates()
{
  // Collect latitude and longitude values
  latitudes[updateCount] = gps.location.lat();
  longitudes[updateCount] = gps.location.lng();

  updateCount++;

  if (updateCount == maxUpdates)
  {
    // Calculate the average latitude and longitude
    double averageLatitude = 0.0;
    double averageLongitude = 0.0;

    for (int i = 0; i < maxUpdates; i++)
    {
      averageLatitude += latitudes[i];
      averageLongitude += longitudes[i];
    }

    averageLatitude /= maxUpdates;
    averageLongitude /= maxUpdates;

    // print the average values
    Serial1.print("Average Latitude: ");
    Serial1.println(averageLatitude, 6);
    Serial1.print("Average Longitude: ");
    Serial1.println(averageLongitude, 6);

    // Reset the arrays and update count
    for (int i = 0; i < maxUpdates; i++)
    {
      latitudes[i] = 0.0;
      longitudes[i] = 0.0;
    }

    updateCount = 0;
  }
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
    waypointSet = true; // Set the waypoint flag

    Serial1.println(F("New waypoint coordinates set."));
    Serial1.print(F("Latitude: "));
    Serial1.print(userLat, 6);
    Serial1.print(F(", Longitude: "));
    Serial1.println(userLng, 6);
  }
  else
  {
    Serial1.println(F("Invalid input format. Use the format: LATITUDE, LONGITUDE"));
  }
}

void displayGpsInfo()
{
  
  // Send data to the Bluetooth module
  Serial1.print(F("Location: "));
  if (gps.location.isValid())
  {
    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();

    Serial1.print(currentLat, 6);
    Serial1.print(F(","));
    Serial1.print(currentLng, 6);

     if (userLat != 0.0 && userLng != 0.0)
    {
      // Calculate distance using Haversine formula
      double userDistance = calculateDistance(currentLat, currentLng, userLat, userLng);
      // Send the distance information to Bluetooth
      Serial1.print(F("  User Waypoint Distance (meters): "));
      Serial1.print(userDistance);

      // Calculate desired heading
      double desiredHeading = calculateHeading(currentLat, currentLng, userLat, userLng);

      // Adjust the motors based on the desired heading and current azimuth
       navigateToWaypoint();
    }

    // Update previous waypoint
    prevLat = currentLat;
    prevLng = currentLng;
  }
  // prints invalid if no information was received in regards to location.
  else
  {
    Serial1.print(F("INVALID"));
  }

  Serial1.println(); // Send a newline character to separate data
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
  
  
	Serial1.print("X: ");
	Serial1.print(x);

	Serial1.print(" Y: ");
	Serial1.print(y);

	Serial1.print(" Z: ");
	Serial1.print(z);

	Serial1.print(" Azimuth: ");
	Serial1.print(a);

	Serial1.print(" Bearing: ");
	Serial1.print(b);

	Serial1.print(" Direction: ");
	Serial1.print(myArray[0]);
	Serial1.print(myArray[1]);
	Serial1.print(myArray[2]);

	Serial1.println();

	
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

// Motor control functions for different directions
// **********************************************************************************************************************************************************************
void Forward()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);     
  
  motor1.run(FORWARD);                                                        // go forward all wheels for specified time
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
  delay(500);
}

// **********************************************************************************************************************************************************************
void StopCar()
{
  motor1.run(RELEASE);                                                         
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

}
// **********************************************************************************************************************************************************************
void RightTurn()
{
  motor1.setSpeed(turn_Speed);                                                   
  motor2.setSpeed(turn_Speed);     
  motor3.setSpeed(turn_Speed);     
  motor4.setSpeed(turn_Speed);   

  motor1.run(RELEASE);                                              
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(RELEASE);
  delay(100);                                                           //delay for 100ms more responsive turn per push on bluetooth                   
}

// **********************************************************************************************************************************************************************
void LeftTurn()
{
  motor1.setSpeed(turn_Speed);                                                   
  motor2.setSpeed(turn_Speed);     
  motor3.setSpeed(turn_Speed);     
  motor4.setSpeed(turn_Speed);    
  
  motor1.run(FORWARD);                                                        // Turn left
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(FORWARD);
  delay(100);    
}
// **********************************************************************************************************************************************************************


double calculateHeading(double lat1, double lon1, double lat2, double lon2)
{
  // Convert latitude and longitude from degrees to radians
  lat1 = radians(lat1);
  lon1 = radians(lon1);
  lat2 = radians(lat2);
  lon2 = radians(lon2);

  // Calculate the heading angle using the arctangent function
  double y = sin(lon2 - lon1) * cos(lat2);
  double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(lon2 - lon1);
  double heading = atan2(y, x);

  // Convert the heading angle from radians to degrees
  heading = degrees(heading);

  // Normalize the heading angle to be between 0 and 360 degrees
  if (heading < 0)
  {
    heading += 360.0;
  }

  return heading;
}

void navigateToWaypoint() {
  // Calculate the distance to the target waypoint
  double distanceToWaypoint = calculateDistance(gps.location.lat(), gps.location.lng(), userLat, userLng);

  // Set a threshold distance (adjust as needed)
  double waypointThreshold = 10.0; // meters

  // Continue adjusting motors until the waypoint is reached
  while (distanceToWaypoint > waypointThreshold) {
    // Calculate the desired heading to the waypoint
    double desiredHeading = calculateHeading(gps.location.lat(), gps.location.lng(), userLat, userLng);

    // Adjust the motors based on the desired heading and current azimuth
    double currentAzimuth = compass.getAzimuth();
    double headingError = desiredHeading - currentAzimuth;

    // Normalize the heading error to be between -180 and 180 degrees
    while (headingError > 180.0) headingError -= 360.0;
    while (headingError <= -180.0) headingError += 360.0;

    if (headingError < -10.0) {
      LeftTurn(); // Turn left when the heading error is significant
    } else if (headingError > 10.0) {
      RightTurn(); // Turn right when the heading error is significant
    } else {
      Forward(); // Go forward when the heading error is small
    }

    // Update the distance to the waypoint
    distanceToWaypoint = calculateDistance(gps.location.lat(), gps.location.lng(), userLat, userLng);

    // You can add a delay here to control the loop frequency
    // delay(100); // Adjust the delay as needed
  }

  // When the waypoint is reached, stop the motors
  StopCar();
  Serial1.println(F("Waypoint reached!"));
}
