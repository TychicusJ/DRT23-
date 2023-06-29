#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <QMC5883LCompass.h>
#include <SPI.h>
#include <SD.h>
#include <L298NX2.h>
#include "BluetoothSerial.h"

/*
   This sketch demonstrates the normal use of a TinyGPS++(TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and using the 
   9600-baud serial GPS device hooked up on pins 36(rx) and 39(tx) and a QMC5883 Magnetic Compass
   connected to the SCL/SDA pins.
*/

const int chipSelect = 5;
static const int RXPin = 36, TXPin = 39; // RX and TX pins are reverse on the board RX goto 39 and TX goto 36 
static const uint32_t GPSBaud = 9600;

// Assign a Unique ID to the QMC5883 Compass Sensor
QMC5883LCompass compass;
TinyGPSPlus gps; // The TinyGPSPlus object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

// Initializing waypoint constants and variables
#define MAX_WAYPOINTS 10 // maximum waypoints to be stored in the SD Card

int waypoint_counter = 0; // counts the number of waypoints stored
int current_waypoint = 0; // index of the current waypoint 
bool reached_waypoint = false; // flag for reaching the waypoint
float target_heading, current_heading; // target and current headings

//right side motor control
int L298N_IN1 = 27;
int L298N_IN2 = 16;

//left side motor control
int L298N_IN3 = 17;
int L298N_IN4 = 25;

BluetoothSerial SerialBT;
String buffer;

void setup()
{
  Serial.begin(115200);
  SerialBT.begin("DRT23"); //Bluetooth device name
  Serial.println("\nThe device started, now you can pair it with bluetooth!");
  ss.begin(GPSBaud);
  compass.init();

  Serial.println(F("Simple Test with TinyGPS++ and attached GT-U7 GPS module"));
  Serial.print(F("Testing TinyGPSPlus library v. "));
  Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println();

  // setup SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card failed or not present.");
  }
  else {
    Serial.println("SD card initialized.");
  }

  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);
  pinMode(L298N_IN3, OUTPUT);
  pinMode(L298N_IN4, OUTPUT);
  

}
  
  int previous_waypoint_counter = 0; // Previous count of Waypoints

  bool newWaypoint()
  {
    // Check if there are new waypoints in the SD card
    // Return true if there are new waypoints, false otherwise
    int current_waypoint_counter = 0; // Get the current count of waypoints

    // Open the file containing the waypoints
    File file = SD.open("waypoints.txt", FILE_READ);

    // If the file is opened successfully
    if (file)
    {
      // Count the number of lines (waypoints) in the file
      while (file.available())
      {
        if (file.read() == '\n')
        {
          current_waypoint_counter++;
        }
      }

      // close the file
      file.close();
    }

    // Compare the current count with the previous count
    bool hasNewWaypoints = (current_waypoint_counter > previous_waypoint_counter);

    // Update the previous count with the current count
    previous_waypoint_counter = current_waypoint_counter;

    return hasNewWaypoints;

  }


// The loop function is where the navigation code goes.
void loop()
{
  // checking to see if there are any new waypoints in the SD card. If there are, then read them from the array:
  if (newWaypoint())
  {
    waypoint_counter++;
    String waypoints[MAX_WAYPOINTS]; // Create an array to store the waypoints
    readWaypoints(waypoints); // Pass the array to the function
    Serial.print("Number of waypoints: "); // print the number of waypoints to serial Monitor
    Serial.println(waypoint_counter);
  }

  // This sketch displays information every time a new sentence is correctly encoded.
  while (ss.available() > 0)
  {
    if (gps.encode(ss.read()))
    {
      displayGpsInfo();
    }
  }

// The directions for the motors 
  startCar();
  delay(1000);
  stopCar();
  delay(1000);
  leftTurn();
  delay(1000);
  rightTurn();
  delay(1000);

  if (SerialBT.available())
  {
    buffer = SerialBT.readStringUntil('\n');
    // Process the received data
    // ...
  }
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

// Place the code related to reading waypoints from SD card and checking for new waypoints here
// Implement the functions: newWaypoint() and readWaypoints() as per your requirements.
// Example function signatures:

void readWaypoints(String waypoints[])
{
  // Open the file containing the waypoints
  File myFile = SD.open("waypoints.txt", FILE_READ);

  // Check if the file is opened successfully
  if (myFile)
  {
    int index = 0; // Index for storing waypoints in the array

    // Read waypoints from the SD card and store them in the waypoints array
    while (myFile.available())
    {
      String buffer = myFile.readStringUntil('\n');
      waypoints[index] = buffer; // Store the waypoint in the array
      index++; // Increment the index for the next waypoint
    }

    myFile.close();
  }
}

// This is where the motors get the inputs for the directions for the motors.
void startCar(){
  digitalWrite(L298N_IN1, HIGH); //27,16,17,25
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, HIGH);
  digitalWrite(L298N_IN4, LOW);
}
void stopCar(){
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, LOW);
}
void leftTurn(){
  digitalWrite(L298N_IN1, LOW);
  digitalWrite(L298N_IN2, HIGH);
  digitalWrite(L298N_IN3, HIGH);
  digitalWrite(L298N_IN4, LOW);
}
void rightTurn(){
  digitalWrite(L298N_IN1, HIGH);
  digitalWrite(L298N_IN2, LOW);
  digitalWrite(L298N_IN3, LOW);
  digitalWrite(L298N_IN4, HIGH);
}
