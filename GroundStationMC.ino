#include <Adafruit_GPS.h>
#include "GS_Funcs.h"

// Serial Lines
#define GPSSerial Serial1
#define Xbee Serial2
#define GPS_BAUD_RATE 9600
// GPIOS
#define ACTIVATE 39
#define PAYLOAD 36
#define ACTIVATION_LED 21
#define PAYLOAD_LED 4
#define AIRBRK_LED 26
#define PWR_LED 25
#define LAUNCH_LED 13
#define APAGEY_LED 12
#define LAND_LED 27
#define AVFIX_LED 33
#define GSFIX_LED 15
#define GPS_DEBUG false

Adafruit_GPS GPS(&GPSSerial);

boolean activated = false;
boolean payload = false;
boolean activated_response = false;
boolean payload_response = false;
String payload_string = "0";
long last_low_timer = millis();

// data
String xbee_msg = "";
String gs_gps_long = "";
String gs_gps_lat = "";
int gs_gps_sat;
String gs_gps_fix = "";
long gs_data_timer = millis();
uint32_t timer = millis();

void setup(){
  Xbee.begin(9600, SERIAL_8N1, 14, 32);
  Serial.begin(9600);
  init_pinmodes();
  init_gs_gps();

  if(GPS.fix){
    digitalWrite(GSFIX_LED, HIGH);
    gs_gps_fix = "1";
    Serial.println("GS FIX SUCCESS");
  }
  else{
    digitalWrite(GSFIX_LED, LOW);
    gs_gps_fix = "0";
  }




}
void loop(){

  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    Serial.print(GPS.lastNMEA()); // newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) //newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 4000) {
    timer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      if(!digitalRead(GSFIX_LED)){
        digitalWrite(GSFIX_LED, HIGH);
      }

    }
    if(GPS_DEBUG){gps_debug(c);}
  }
}

void init_pinmodes(){
  pinMode(ACTIVATE, INPUT);
  pinMode(PAYLOAD, INPUT);
  pinMode(ACTIVATION_LED, OUTPUT);
  pinMode(PAYLOAD_LED, OUTPUT);
  pinMode(AIRBRK_LED, OUTPUT);
  pinMode(PWR_LED, OUTPUT);
  pinMode(LAUNCH_LED, OUTPUT);
  pinMode(APAGEY_LED, OUTPUT);
  pinMode(LAND_LED, OUTPUT);
  pinMode(AVFIX_LED, OUTPUT);
  pinMode(GSFIX_LED, OUTPUT);
  digitalWrite(PWR_LED, HIGH);
}
void init_gs_gps(){
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  if(!GPS.begin(GPS_BAUD_RATE)){
    Serial.println("Failed GPS begin");
  }
  else{
    Serial.println("Success GPS begin");
  }

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); //antenna status updates
  delay(1000);
  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);
  Serial.println("Groundstation setup complete");
}
void gps_debug(char c){
  if (c) Serial.print(c);

  if(GPS.fix){
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

  }
}

