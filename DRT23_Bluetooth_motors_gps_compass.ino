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
SoftwareSerial ss(RXPin, TXPin); // The bluetooth connection to the GPS device
// The TinyGPSPlus object
TinyGPSPlus gps; // The GT-U7 gps 

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


void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);
  compass.init();

  // Initialize Bluetooth bluetooth connection
  bluetooth.begin(BluetoothBaud); // HC-06 module baud rate

  bluetooth.println(F("Simple Test with TinyGPS++ and attached GT-U7 GPS module"));
  bluetooth.print(F("Testing TinyGPSPlus library v. ")); bluetooth.println(TinyGPSPlus::libraryVersion());
  bluetooth.println();
  bluetooth.println(F("Enter the next waypoint coordinates (latitude and longitude) in decimal degrees."));
  bluetooth.println(F("Use the format: LATITUDE, LONGITUDE"));

}

void loop()
{
  // Check for user input
  if (Serial.available() > 0)
  {
    String userInput = Serial.readStringUntil('\n');
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
        displayGpsInfo();
        navigateToWaypoint();
      }
    }
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

    bluetooth.println(F("New waypoint coordinates set."));
    bluetooth.print(F("Latitude: "));
    bluetooth.print(userLat, 6);
    bluetooth.print(F(", Longitude: "));
    bluetooth.println(userLng, 6);
  }
  else
  {
    bluetooth.println(F("Invalid input format. Use the format: LATITUDE, LONGITUDE"));
  }
}

void displayGpsInfo()
{
  // Send data to the Bluetooth module
  bluetooth.print(F("Location: "));
  if (gps.location.isValid())
  {
    double currentLat = gps.location.lat();
    double currentLng = gps.location.lng();

    bluetooth.print(currentLat, 6);
    bluetooth.print(F(","));
    bluetooth.print(currentLng, 6);

    if (prevLat != 0.0 && prevLng != 0.0)
    {
      // Calculate distance using Haversine formula
      double distance = calculateDistance(prevLat, prevLng, currentLat, currentLng);
      // Send the distance information to Bluetooth
      bluetooth.print(F("  Distance (meters): "));
      bluetooth.print(distance);

      // Calculate desired heading
      double desiredHeading = calculateHeading(currentLat, currentLng, userLat, userLng);

      // Adjust the motors based on the desired heading and current azimuth
      adjustMotors(desiredHeading, compass.getAzimuth());
    }

    // Update previous waypoint
    prevLat = currentLat;
    prevLng = currentLng;
  }
  // Prints invalid if no information was received in regards to location.
  else
  {
    bluetooth.print(F("INVALID"));
  }

  bluetooth.println(); // Send a newline character to separate data
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
  
  
	bluetooth.print("X: ");
	bluetooth.print(x);

	bluetooth.print(" Y: ");
	bluetooth.print(y);

	bluetooth.print(" Z: ");
	bluetooth.print(z);

	bluetooth.print(" Azimuth: ");
	bluetooth.print(a);

	bluetooth.print(" Bearing: ");
	bluetooth.print(b);

	bluetooth.print(" Direction: ");
	bluetooth.print(myArray[0]);
	bluetooth.print(myArray[1]);
	bluetooth.print(myArray[2]);

	bluetooth.println();

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

void adjustMotors(double desiredHeading, double currentAzimuth)
{
  // Define a tolerance for heading error (adjust as needed)
  double headingTolerance = 10.0; // Degrees

  // Calculate the heading error
  double headingError = desiredHeading - currentAzimuth;

  // Normalize the heading error to be between -180 and 180 degrees
  while (headingError > 180.0) headingError -= 360.0;
  while (headingError <= -180.0) headingError += 360.0;


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
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);   

  motor1.run(FORWARD);                                              
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  delay(100);                                                           //delay for 100ms more responsive turn per push on bluetooth                   
}

// **********************************************************************************************************************************************************************
void LeftTurn()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);    
  
  motor1.run(BACKWARD);                                                        // Turn left
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  delay(100);    
}
// **********************************************************************************************************************************************************************
void Reverse()
{
  motor1.setSpeed(mtr_Spd);                                                   
  motor2.setSpeed(mtr_Spd);     
  motor3.setSpeed(mtr_Spd);     
  motor4.setSpeed(mtr_Spd);    
  
  motor1.run(BACKWARD);                                                        // Reverse all wheels
  motor2.run(BACKWARD);
  motor3.run(BACKWARD);
  motor4.run(BACKWARD);
}

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

  // If the distance is less than the threshold, consider it reached
  if (distanceToWaypoint <= waypointThreshold) {
    bluetooth.println(F("Waypoint reached!"));
    // Stop the motors or perform any desired actions
    StopCar(); // Stop the motors when the waypoint is reached
  } else {
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
  }
}
