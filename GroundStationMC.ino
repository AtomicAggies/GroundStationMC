#include <Adafruit_GPS.h>
#include <cppQueue.h>
#include "GS_Funcs.h"

// Serial Lines
#define GPSSerial Serial1
#define Xbee Serial2
#define GPS_BAUD_RATE 9600
// GPIOS
#define ACTIVATE_PIN 39
#define PAYLOAD_PIN 36
#define ACTIVATION_LED 21
#define PAYLOAD_LED 4
#define AIRBRK_LED 26
#define PWR_LED 25
#define LAUNCH_LED 13
#define APOGEE_LED 12
#define LAND_LED 27
#define AVFIX_LED 33
#define GSFIX_LED 15
#define GPS_DEBUG false
#define DEBUG true
// Queue 
#define	IMPLEMENTATION	FIFO
#define QUEUE_SIZE 25
#define MSG_SIZE 51

Adafruit_GPS GPS(&GPSSerial);

boolean activated = false;
boolean payload = false;
boolean activated_response = false;
boolean payload_response = false;
String payload_string = "0";
long last_low_timer = millis();

// data
struct DataElement {
  char data[MSG_SIZE]; // 50 characters + null terminator
};
DataElement queueStorage[QUEUE_SIZE];
cppQueue data_queue(sizeof(DataElement), QUEUE_SIZE, IMPLEMENTATION,true,(void*)queueStorage,sizeof(queueStorage));
//msg data
String xbee_msg = "";
String gs_gps_long = "";
String gs_gps_lat = "";
int gs_gps_sat;
String gs_gps_fix = "";
//timers
unsigned long activation_data_timer = millis();
unsigned long deac_data_timer = millis();

unsigned long gs_gps_timer = millis();

//State
//ENSURE State values match the transmit values
enum ActivationState { INACTIVE = 11, ACTIVATED = 21, A_AWAIT_RESPONSE = 31 } activation_state;
enum PayloadState { NOT_INITIATED = 12, INITIATED = 22, P_AWAIT_RESPONSE = 32} payload_state;
enum RocketState{LAUNCH_PAD = 0,LAUNCH = 10,APOGEE = 20,LANDING = 30}rocket_state;
enum AirbrakesState{NOT_DEPLOYED = 13,DEPLOYED = 23} airbrakes_state;
enum AvGPSState{AV_NOFIX = 14,AV_FIX = 24} av_gps_state;

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

  activation_state = INACTIVE;
  rocket_state = LAUNCH_PAD;
  av_gps_state = AV_NOFIX;
  if(DEBUG)Serial.println("DEBUG MODE ON");
}
void loop(){

  // read data from the GPS in the 'main loop'
  char c = GPS.read();

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // Serial.print(GPS.lastNMEA()); // newNMEAreceived() flag to false
    GPS.lastNMEA();
    if (!GPS.parse(GPS.lastNMEA())) //newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // approximately every 4 seconds or so, print out the current stats
  if (millis() - gs_gps_timer > 4000) {
    gs_gps_timer = millis(); // reset the timer
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      if(!digitalRead(GSFIX_LED)){
        digitalWrite(GSFIX_LED, HIGH);
      }
    }
    if(GPS_DEBUG){gps_debug(c);}
  }

  if(Xbee.available()>0){ //incoming avionics data
    xbee_msg = Xbee.readStringUntil('\n');
    // DataElement element;
    // strcpy(element.data,xbee_msg,sizeof(element.data)); //FIXME Implement asynchronous or event driven acquisition/sending of data
    //send data to GUI
    if(DEBUG)Serial.println("Xbee available");
    if(DEBUG)Serial.println(xbee_msg);
    //parse case and set led's
    char c_xbee_msg[MSG_SIZE];
    strncpy(c_xbee_msg, xbee_msg.c_str(), sizeof(c_xbee_msg));
    c_xbee_msg[sizeof(c_xbee_msg) - 1] = '\0'; //ensure null termination
    char* xbee_msg_token = strtok(c_xbee_msg,","); //Grabbing first token from message
    if(xbee_msg_token != NULL){ 
      if(DEBUG)Serial.print("xbee msg not null: "); 
      if(DEBUG)Serial.println(xbee_msg_token);
      if(strcmp(xbee_msg_token,"STATE")==0){
        xbee_msg_token = strtok(NULL,","); //grab next token
        if(xbee_msg_token!=NULL){ //string not empty
          int state_code = atoi(xbee_msg_token);
          Serial.println("state code atoi: ");
          Serial.println(state_code);
          switch(state_code){
            case ACTIVATED: 
              activation_state = ACTIVATED; 
              digitalWrite(ACTIVATION_LED,HIGH);
              Serial.println("CONFIRMED ACTIVATION");
              break;
            case INACTIVE:
              activation_state = INACTIVE;
              digitalWrite(ACTIVATION_LED,LOW);
              Serial.println("CONFIRMED DEACTIVATION");
              break;
            case LAUNCH_PAD:
              rocket_state = LAUNCH_PAD;
              digitalWrite(LAUNCH_LED,LOW);
              digitalWrite(APOGEE_LED,LOW);
              digitalWrite(LAND_LED,LOW);
              break;
            case LAUNCH:
              rocket_state = LAUNCH;
              digitalWrite(LAUNCH_LED,HIGH);
              break;
            case APOGEE:
              rocket_state = APOGEE;
              digitalWrite(APOGEE_LED,HIGH);
              digitalWrite(LAUNCH_LED,LOW);
              break;
            case LANDING:
              rocket_state = LANDING;
              digitalWrite(LAND_LED,HIGH);
              digitalWrite(APOGEE_LED,LOW);
              digitalWrite(LAUNCH_LED,LOW);
              break;
            case AV_FIX:
              av_gps_state = AV_FIX;
                if(!digitalRead(AVFIX_LED)){
                  digitalWrite(AVFIX_LED,HIGH);
                }         
              Serial.println("AVIONICS FIX");
              Xbee.println("AVFIX");
              break;
            default:
              String s = "State Code not Recognized:" + state_code;
              Serial.println(s);
              break;
          }

        }
        else{
          Serial.println("GroundStation Error Parsing State");
        }
      }// end if STATE
      else if (strcmp(xbee_msg_token,"BMP")==0){  //Incoming BMP Data: "BMP," + "Temperature," + "Pressure," + "Altitude," + "ENDDATA";
        Serial.println("Incoming BMP: temp,press,altitude");
        Serial.println(xbee_msg);
        Serial.println("_______________________");
      }//end if BMP
      else if (strcmp(xbee_msg_token,"BNO")==0){ //Incoming BNO Data
        Serial.println("Incoming BNO: pitch,roll,yaw");
        Serial.println(xbee_msg);
        Serial.println("_______________________");
      }// end if BNO
      else if (strcmp(xbee_msg_token,"GPS")==0){ //Incoming GPS Data
        Serial.println("Incoming GPS: Hour:Min:Sec,Latitude,Longitude,Speed,Altitude,Geoid Height,ENDDATA");
        Serial.println(xbee_msg);
        Serial.println("_______________________");
      }
      else{
        Serial.print("Unknown Message: ");
        Serial.println(xbee_msg);
        Serial.println("_______________________");
      }
    }
    else{
      Serial.println("Xbee_msg_token is null");
    }
  } // end if xbee messages received

  // Serial.println("Activation state");
  // Serial.println(activation_state);
  
  if(digitalRead(ACTIVATE_PIN)==LOW && activation_state == ACTIVATED){  // physical activation lever is off. This implies we want to deactivate avionics.
    if(millis() - deac_data_timer > 500){ //FIXME implement way to manually stop messaging ??
      Xbee.println("DEACTIVATE"); 
      Serial.println("SENDING DEACTIVATION...");
      deac_data_timer = millis();
    }

  }
  else if(digitalRead(ACTIVATE_PIN)==HIGH && activation_state == INACTIVE ){  //lever is on. We want to activate avionics
    
    if(millis() - activation_data_timer > 250){
      Xbee.println("ACTIVATE");
      Serial.println("SENDING ACTIVATION...");
      activation_data_timer = millis();
    }
  }
  

}

void init_pinmodes(){
  pinMode(ACTIVATE_PIN, INPUT);
  pinMode(PAYLOAD_PIN, INPUT);
  pinMode(ACTIVATION_LED, OUTPUT);
  pinMode(PAYLOAD_LED, OUTPUT);
  pinMode(AIRBRK_LED, OUTPUT);
  pinMode(PWR_LED, OUTPUT);
  pinMode(LAUNCH_LED, OUTPUT);
  pinMode(APOGEE_LED, OUTPUT);
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

