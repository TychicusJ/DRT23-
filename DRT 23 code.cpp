
#include <SoftwareSerial.h> //include software serial library
#include <Wire.h> //include wire library
#include <QMC5883L.h> //include QMC5883L sensor library
#include <SoftwareSerial.h> //include software serial library
#include <tinyGPSPlus.h> //include Tiny GPS library

SoftwareSerial BTserial(2, 3); // RX, TX - setup Bluetooth

Servo motor; // setup servo motor object

// define variables for QMC5883L sensor
QMC5883L compass;


// define variables for GPS module
SoftwareSerial gpsSerial(4, 5); // RX, TX - setup GPS serial connection
String rawGPS;
String NMEA[14];
float latFloat;
float lonFloat;
int index = 0;
char incomingByte;

// define variables for navigation
float heading = 0; // current heading
float bearing; // direction of next waypoint
float dLat; // change in latitude
float dLon; // change in longitude
float dist; // distance to next waypoint
float nextLatFloat; // latitude of next waypoint
float nextLonFloat; // longitude of next waypoint
int direction; // direction to move motor
int speed; // speed of motor

// define pins for motor
#define dirPin 9
#define pwmPin 10

void setup()
{
 BTserial.begin(9600); // initialize Bluetooth
 Serial.begin(9600); // initialize serial
 gpsSerial.begin(4800); // initialize GPS serial
 memset(NMEA, 0, sizeof(NMEA));
 motor.attach(pwmPin); // attach motor servo
 pinMode(dirPin, OUTPUT); // setup motor pins as outputs
 pinMode(pwmPin, OUTPUT);
 
 // initialize QMC5883L sensor
 if (!compass.begin())
 
 {
 Serial.println("Could not find a valid QMC5883L sensor, check wiring!");
 while (1) {}
 }
 
 while (gpsSerial.available() > 0) { // clear buffer
 char c = gpsSerial.read();
 }
}

void loop()
{
 delay(1000); // delay for stability
 
 // read GPS data
 while (gpsSerial.available() > 0)
 
 {
 incomingByte = gpsSerial.read();
 if (incomingByte == '$')
 
 {
 index = 0;
 
 rawGPS = "";
 
 }
 rawGPS = rawGPS + incomingByte;
 
 if (incomingByte == '\r')
 {
 
 NMEA[index] = strtok(&rawGPS[0], ",");
 
 index++;
 
 while (index <= 13)
 {
 NMEA[index] = strtok(NULL, ",");
 index++;
 }
 latFloat = NMEA[2].toFloat() / 100;
 
 lonFloat = NMEA[4].toFloat() / 100;
 
 if (NMEA[3] == "S")
 
 {
 
 latFloat = -latFloat;
 
   
 }
 
 if (NMEA[5] == "W")
 
 {
 
 lonFloat = -lonFloat;
 
 }
 
 Serial.print("Lat: ");
 
 Serial.println(latFloat);
 
 Serial.print("Lon: ");
 
 Serial.println(lonFloat);
  }
 }
 
 // read QMC5883L sensor data
 int x, ,y , z;
 String cardinal;

 compass.read(&x, &y, &z);

 float headingRadians = atan2(y, x);
  float headingDegrees = headingRadians * 180 / PI;
  float declinationAngle = 11.41666666666667;

  headingDegrees += declinationAngle;

  if (headingDegrees < 0) {
    headingDegrees += 360;
  }

  if (headingDegrees > 348.75 || headingDegrees < 11.25) {
    cardinal = " N";
  }
  else if (headingDegrees > 11.25 && headingDegrees < 33.75) {
    cardinal = " NNE";
  }
  else if (headingDegrees > 33.75 && headingDegrees < 56.25) {
    cardinal = " NE";
  }
  else if (headingDegrees > 56.25 && headingDegrees < 78.75) {
    cardinal = " ENE";
  }
  else if (headingDegrees > 78.75 && headingDegrees < 101.25) {
    cardinal = " E";
  }
  else if (headingDegrees > 101.25 && headingDegrees < 123.75) {
    cardinal = " ESE";
  }
  else if (headingDegrees > 123.75 && headingDegrees < 146.25) {
    cardinal = " SE";
  }
  else if (headingDegrees > 146.25 && headingDegrees < 168.75) {
    cardinal = " SSE";
  }
  else if (headingDegrees > 168.75 && headingDegrees < 191.25) {
    cardinal = " S";
  }
  else if (headingDegrees > 191.25 && headingDegrees < 213.75) {
    cardinal = " SSW";
  }
  else if (headingDegrees > 213.75 && headingDegrees < 236.25) {
    cardinal = " SW";
  }
  else if (headingDegrees > 236.25 && headingDegrees < 258.75) {
    cardinal = " WSW";
  }
  else if (headingDegrees > 258.75 && headingDegrees < 281.25) {
    cardinal = " W";
  }
  else if (headingDegrees > 281.25 && headingDegrees < 303.75) {
    cardinal = " WNW";
  }
  else if (headingDegrees > 303.75 && headingDegrees < 326.25) {
    cardinal = " NW";
  }
  else if (headingDegrees > 326.25 && headingDegrees < 348.75) {
    cardinal = " NNW";
  }

  Serial.print("Heading: ");
  Serial.print(headingDegrees);
  Serial.println(cardinal);

  delay(250);

 
 
 // calculate next waypoint and distance to it
 calculateBearing(latFloat, lonFloat, nextLatFloat, nextLonFloat, &bearing, &dist, &dLat, &dLon);
 if (BTserial.available() )
 {
 int input = BTserial.read();
 if (input == 'A')
    {
    navigate();
    }
 }
}
// function to calculate bearing, distance, and change in latitude and longitude from current location to next waypoint

void calculateBearing(float lat1, float lon1, float lat2, float lon2, float *bearing, float *distance, float 
*dLat, float *dLon)

{
 // convert lat/lon from degrees to radians
 
 lat1 = lat1 * 0.0174532925;
 
 lon1 = lon1 * 0.0174532925;
 
 lat2 = lat2 * 0.0174532925;
 
 lon2 = lon2 * 0.0174532925;
 
 // calculate bearing and distance
 
 *bearing = atan2(sin(lon2-lon1)*cos(lat2), cos(lat1)*sin(lat2)-sin(lat1)*cos(lat2)*cos(lon2-lon1));
 *bearing = *bearing * 57.2957795;
 
 if (*bearing < 0)
 {
   
 *bearing += 360;
 
 }
 
 float dLatRad = pow((lat2-lat1)/2,2);
 
 float dLonRad = pow((lon2-lon1)/2,2);
 
 float a = sin(dLatRad) + cos(lat1) * cos(lat2) * sin(dLonRad);
 
 float c = 2 * atan2(sqrt(a), sqrt(1-a));
 
 *distance = 6371 * c;
 
 // calculate change in latitude and longitude
 *dLat = lat2 - lat1;
 
 *dLon = lon2 - lon1;
}

// function to navigate robot to next waypoint 

void navigate()

{
// set direction and speed based on heading and bearing to next waypoint

 heading = compass.read(&x, &y, &z);
 
 direction = 0;
 
 speed = 0;
 
 if (dist > 0.002)
 
 {
   
  if (abs(bearing - heading) > 5)
  {
    direction = (int) (bearing - heading);
    
    speed = 45;
  } 
  
 else 
 
 {
 speed = dist * 10000;
 }
 }
 // move motor
 if(speed == 0) {
 digitalWrite(dirPin, HIGH);
 } else if(direction > 0) {
 digitalWrite(dirPin, HIGH);
 } else if(direction < 0) {
 digitalWrite(dirPin, LOW);
 }
 motor.write(abs(direction));
 Serial.print("Heading: ");
 Serial.println(heading);
 Serial.print("Bearing: ");
 Serial.println(bearing);
 Serial.print("Direction: ");
 Serial.println(direction);
 Serial.print("Speed: ");
 Serial.println(speed);
}
