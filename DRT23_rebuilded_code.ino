#include <Wire.h> // For I2C communication
#include <QMC5883L.h> // Compass library
#include <TinyGPSPlus.h> // GPS library
#include <SD.h> // SD card library
#include <SoftwareSerial.h>

SoftwareSerial GPS_SoftSerial(36,39); // RX and TX pins are reverse on the board RX goto 39 and TX goto 36
TinyGPSPlus gps;
QMC5883L qmc;
File dataFile;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  GPS_SoftSerial.begin(9600);

  // Initialize compass
  compass.init();
  compass.setSamplingRate(QMC5883L_CONTINOUS, 10);
  
  // Initialize SD card
  if (SD.begin(10)) {
    Serial.println("SD card is ready.");
    dataFile = SD.open("waypoints.txt", FILE_READ);
  } else {
    Serial.println("SD card initialization failed.");
  }
}
void readWaypoints() {
  if (dataFile) {
    while (dataFile.available()) {
      String line = dataFile.readStringUntil('\n');
      // Parse the line to extract waypoint data (e.g., latitude and longitude)
      // Store the waypoints in your preferred data structure (e.g., an array or list)
    }
    dataFile.close();
  } else {
    Serial.println("Error opening waypoints.txt");
  }
}
void loop() {
  // Read GPS data
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      if (gps.location.isValid()) {
        double currentLat = gps.location.lat();
        double currentLon = gps.location.lng();

        // Compare currentLat, currentLon with your stored waypoints
        // Calculate the heading using compass data
        // Implement your navigation logic here
      }
    }
  }

  // Read compass data
  int16_t heading = compass.readHeading();
  // Use heading data as needed for navigation

  // Add your navigation logic here
  
  delay(1000); // Adjust the delay as needed
}
