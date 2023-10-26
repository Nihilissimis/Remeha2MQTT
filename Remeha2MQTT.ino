#include <ArduinoOTA.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Remeha2MQTT.h"
#include "ESPTelnet.h"  

// Struct Module (static) data 
struct Module { 
  const char name [11];    // Module/Topic name
  const byte index;        // Index for multiple module strings per topic
  const byte offset;       // Offset values start in topic struct  / const byte
  const bool enable;       // enable this module string
  const bool endian;       // Endianness of data received in boiler Module response (TRUE=big, FALSE=little)
  const bool retained;     // Publish static topic data as retained 
  const byte length;       // length of to be received message
  const byte numVal;       // number of topics in message
  const byte command [10]; // Command 
};

const Module boilerModule[] = {
  {"ID-Serial",  1,  0, 1, 1, 1, 74,  8, {0x02,0xfe,0x00,0x05,0x08,0x01,0x0b,0xd4,0x9c,0x03}}, 
  {"ID-Serial",  2,  8, 1, 1, 1, 26, 11, {0x02,0xfe,0x01,0x05,0x08,0x01,0x0b,0xe9,0x5c,0x03}},
  {"ID-Serial",  3,  19, 1, 1, 1, 26, 11, {0x02,0xfe,0x03,0x05,0x08,0x01,0x0b,0x90,0x9c,0x03}},
  // (not used on Calenta)
  //{"ID-Serial",  4,  28, 0, 1, 1,  0,  0, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x17, 0xd9, 0x05, 0x03}}, 
  //{"ID-Serial",  5,  28, 0, 1, 1,  0,  0, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x18, 0x99, 0x01, 0x03}}, 
  //{"ID-Serial",  6,  28, 0, 1, 1,  0,  0, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x19, 0x58, 0xc1, 0x03}}, 
  //{"ID-Serial",  7,  28, 0, 1, 1,  0,  0, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x1a, 0x18, 0xc0, 0x03}}, 
  //{"ID-Serial",  8,  28, 0, 1, 1,  0,  0, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x1b, 0xd9, 0x00, 0x03}}, 
 
  {"Samples",    0,  30, 1, 0, 0, 74, 51, {0x02, 0xfe, 0x01, 0x05, 0x08, 0x02, 0x01, 0x69, 0xab, 0x03}},
  {"Counters",   1,  81, 1, 1, 0, 26,  8, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x1c, 0x98, 0xc2, 0x03}}, 
  {"Counters",   2,  89, 1, 1, 0, 15,  3, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x1d, 0x59, 0x02, 0x03}},
  // (not used on Calenta)
  //{"Counters",   3,  90, 0, 1, 0, 26,  0, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x1e, 0x19, 0x03, 0x03}}, 
  //{"Counters",   4,  90, 0, 1, 0, 26,  0, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x1f, 0xd8, 0xc3, 0x03}}, 
  {"Parameters", 1,  92, 1, 0, 0, 26,  8, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x14, 0x99, 0x04, 0x03}}, 
  {"Parameters", 2,  100, 1, 0, 0, 26, 15, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x15, 0x58, 0xc4, 0x03}},
  {"Parameters", 3, 115, 1, 0, 0, 24, 15, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x16, 0x18, 0xc5, 0x03}}, 
  {"Parameters", 4, 130, 1, 0, 0, 24,  4, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x17, 0xd9, 0x05, 0x03}},
  {"Parameters", 5, 134, 1, 0, 0, 10,  3, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x18, 0x99, 0x01, 0x03}},
  // (not used on Calenta)
  //{"Parameters", 6, 148, 0, 0, 0, 26, 16, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x19, 0x58, 0xc1, 0x03}}, 
  //{"Parameters", 7, 164, 0, 0, 0, 26, 16, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x1a, 0x18, 0xc0, 0x03}}, 
  //{"Parameters", 8, 178, 0, 0, 0, 26, 14, {0x02, 0xfe, 0x00, 0x05, 0x08, 0x10, 0x1b, 0xd9, 0x00, 0x03}}
};

// struct topics Data
struct Data {
  const char name [32]; // Variable name
  const bool publish; // Publish to MQTT 
  const int8_t mply_div; // multiply / divide (8-bit signed int: +values: multiply, -values: divide. Effective range: 1/127 - 127))
  const byte index; // Location in bytestring from Boiler
  const byte length;  // Number of bytes in bytestring from Boiler
  const byte type; // decode type: 1: Status, 2: Number, 3: Hex as Iint, 4: ASCII, 5: Serial
  const byte bit_prec; // index of bit when bit_encoded or precision for divided values
  const byte sign; // signed integer (1 = signed, 0 unsigned)
  char value [16]; 
  char old_value [16]; 
};

// static data (name, publish, mply_div, index, length, type, bit/precision and sign)
Data topics[] = {
  // ID-Serial 1 - offset 0
  {"Boiler_dF-code", 1, 0, 8, 1, 2, 0, 0}, {"Boiler_dU-code", 1, 0, 9, 1, 3, 1, 0}, {"Boiler_software_version", 1, -10, 12, 1, 3, 1, 0}, {"Boiler_parameter_version", 1, -10, 13, 1, 3, 1, 0},
  {"Boiler_parameter_type", 1, 0, 14, 1, 3, 0, 0},{"Next Service Type",1,0,17,1,2,0,0}, {"Boiler_serial_number", 1, 0, 39, 16, 4, 0, 0}, {"Boiler_name", 1, 0, 55, 16, 4, 0, 0},
  // ID-Serial 2 - offset 7
  {"PCU Device type",1,0,7,1,2,0,0},{"PCU Softw Ver.",1,-10,8,1,3,1,0},
  {"PCU Param Ver.",1,-10,9,1,3,1,0},{"PCU ParamType",1,0,10,1,2,0,0},{"PCU_Bedrijfs uren",1,2,11,2,2,0,0},
  {"PCU connect SU type",1,0,13,1,2,0,0},{"PCU connect PSU type",1,0,14,1,2,0,0},
  {"PCU last block code",1,0,15,1,2,0,0},{"PCU last lock code",1,0,16,1,2,0,0},{"PCU voltage",1,0,17,1,2,0,0},{"PCU SNR",1,0,18,5,5,0,0},
  // ID-Serial 3 - offset 18
  {"SU Device type",1,0,7,1,2,0,0},{"SU Softw Ver.",1,-10,8,1,3,1,0},{"SU Param Ver.",1,-10,9,1,3,1,0},{"SU ParamType",1,0,10,1,2,0,0},
  {"SU_Bedrijfsuren",1,8,11,2,2,0,0},{"SU connect PCU type",1,0,13,1,2,0,0},{"SU connect PSU type",1,0,14,1,2,0,0},
  {"SU last block code",1,0,15,1,2,0,0},{"SU last lock code",1,0,16,1,2,0,0},{"SU last int err.",1,0,17,1,2,0,0},{"SU SNR",1,0,18,5,5,0,0},

  // ID-Serial 4-8 (not used on Calenta)
  // Samples - offset 28
  {"Aanvoer_temperatuur", 1, -100, 7, 2, 2, 2, 1}, {"Retour_temperatuur", 1, -100, 9, 2, 2, 2, 1}, {"Zonneboiler_temperatuur", 1, -100, 11, 2, 2, 2, 1}, {"Buiten_temperatuur", 1, -100, 13, 2, 2, 2, 1}, 
  {"Boiler_temperatuur", 1, -100, 15, 2, 2, 2, 1}, {"Automaat_temperatuur", 1, -100, 19, 2, 2, 2, 1}, {"Ruimte_temperatuur", 1, -100, 21, 2, 2, 2, 1}, {"CV_setpunt", 1, -100, 23, 2, 2, 2, 1}, 
  {"SWW_setpunt", 1, -100, 25, 2, 2, 2, 1}, {"Ruimte_setpunt", 1, -100, 27, 2, 2, 2, 1}, {"Ventilator_setpunt", 1, 0, 29, 2, 2, 0, 1}, {"Ventilator_toeren", 1, 0, 31, 2, 2, 0, 1},
  {"Ionisatie_stroom", 1, -10, 33, 1, 2, 0, 0}, {"Intern_setpunt", 1, -100, 34, 2, 2, 2, 1}, {"Beschikbaar_vermogen", 1, 0, 36, 1, 2, 0, 0}, {"Pomp_snelheid", 1, 0, 37, 1, 2, 0, 0}, 
  {"Gevraagd_vermogen", 1, 0, 39, 1, 2, 0, 0}, {"Geleverd_vermogen", 1, 0, 40, 1, 2, 0, 0},
    // Samples - bitmaps
  {"Modulerende_regelaar", 1, 0, 43, 1, 1, 1, 0}, {"Warmtevraag_modulerend", 1, 0, 43, 1, 1, 2, 0}, {"Warmtevraag_aan-uit", 1, 0, 43, 1, 1, 3, 0}, {"Vorstbeveiliging", 1, 0, 43, 1, 1, 4, 0}, 
  {"SWW_ecostand", 1, 0, 43, 1, 1, 5, 0}, {"SWW_blokkering", 1, 0, 43, 1, 1, 6, 0}, {"Anti_Legionella", 1, 0, 43, 1, 1, 7, 0}, {"SWW_warmtevraag", 1, 0, 43, 1, 1, 8, 0}, 
  {"Ingang_blokkerend", 1, 0, 44, 1, 1, 1, 0}, {"Ingang_vrijgave", 1, 0, 44, 1, 1, 2, 0}, {"Ionisatie", 1, 0, 44, 1, 1, 3, 0}, {"SWW_tapschakelaar", 1, 0, 44, 1, 1, 4, 0},
  {"Gasdruk_minimaal", 1, 0, 44, 1, 1, 6, 0}, {"CV_ingeschakeld", 1, 0, 44, 1, 1, 7, 0}, {"SWW_ingeschakeld", 1, 0, 44, 1, 1, 8, 0}, {"Gasklep_dicht", 1, 0, 45, 1, 1, 1, 0}, 
  {"Onsteking_actief", 1, 0, 45, 1, 1, 1, 0}, {"3-wegklep_positie", 1, 0, 45, 1, 1, 4, 0}, {"Externe_3-wegklep", 1, 0, 45, 1, 1, 5, 0}, {"Externe_gasklep", 1, 0, 45, 1, 1, 7, 0}, 
  {"Pomp_actief", 1, 0, 46, 1, 1, 1, 0}, {"Boilerpomp", 1, 0, 46, 1, 1, 2, 0}, {"Externe_CV_pomp", 1, 0, 46, 1, 1, 3, 0}, {"Status_rapport", 1, 0, 46, 1, 1, 5, 0}, 
  {"Opentherm_SmartPower", 1, 0, 46, 1, 1, 8, 0}, 
    // Samples - end bitmaps
  {"Status", 1, 0, 47, 1, 1, 0, 0}, {"Vergrendeling", 1, 0, 48, 1, 1, 0, 0}, {"Blokkering", 1, 0, 49, 1, 1, 0, 1}, {"Sub-status", 1, 0, 50, 1, 1, 0, 1}, 
  {"Waterdruk", 1, -10, 56, 1, 2, 1, 1}, {"Regel_temperatuur", 1, -100, 58, 2, 2, 2, 1}, {"SWW_Tapdebiet", 1, -100, 60, 2, 2, 2, 1}, {"Solar_temperatuur", 1, -100, 63, 2, 2, 2, 1},
 // Counters 1 - offset 79
  {"HoursRunPumpCHDHW", 1, 2, 7, 2, 2, 0, 0},{"HoursRun3wayDHW", 1, 2, 9, 2, 2, 0, 0}, {"HoursRunCHDHW", 1, 2, 11 ,2, 2, 0, 0},{"HoursrunDHW", 1, 0, 13, 2, 2, 0, 0},
  {"Powersupply_availablehours", 1, 2, 15, 2, 2, 0, 0}, {"Pump_startsCHDHW", 1, 8, 17, 2, 2, 0, 0},{"Number_3Wayvalvecycles", 1, 8, 19, 2, 2, 0, 0}, {"BurnerStartsDHW", 1, 8, 21, 2, 2, 0, 0},
  // Counters 2 - offset 87
  {"Total_burnerStartsCHDHW", 1, 8, 7, 2, 2, 0, 0}, {"Failed_burnerstarts", 1, 0, 9, 2, 2, 0, 0}, {"Number_offlameloss", 1, 0, 11, 2, 2, 0, 0},
 // Counters 3 - 4 (not used on Calenta)
  // Parameters 1 - offset 90  (name, publish, mply_div, index, length, type, bit/precision and sign)
  {"MaxflowTempCH",1,0,7,1,2,0,0},{"DHWsetTemp",1,0,8,1,2,0,0},{"Boilercontrols",1,0,9,1,2,0,0},
  {"ComfortDHW",1,0,10,1,2,0,0},{"Anticipation",1,0,11,1,2,0,0},{"InfoDisplay",1,0,12,1,2,0,0},
  {"PumpPostRunTime",1,0,13,1,2,0,0},{"Displaymode",1,0,14,1,2,0,0},
  // Parameters 2 - offset 98
  {"MaxFanspeedCH",1,100,7,1,2,0,0},
  {"MaxFanspeedDHW",1,100,8,1,2,0,0},{"MaxFanspeedCHDHW",1,100,9,1,2,0,0},{"Offsetpartload",1,0,10,1,2,0,0},
  {"StartFanspeed",1,100,11,1,2,0,0},{"MinWaterPressure",1,-10,12,1,2,1,0},{"MaxflowTemp",1,0,13,1,2,0,0},
  {"FootpointoutsideT",1,0,15,1,2,0,0},{"FootpointflowT",1,0,16,1,2,0,0},{"ClimaPoutsideT",1,1,17,1,2,0,1},
  {"PumpCHmin",1,10,18,1,2,0,0},{"PumpCH_max",1,10,19,1,2,0,0},{"FrostprotectminT",1,0,20,1,2,0,1},
  {"Antilegionella",1,0,21,1,2,0,0},{"SetpointraiseDHW",1,0,22,1,2,0,0},
  // Parameters 3  offset 113
  {"Hysteresis_calorifier",1,0,7,1,2,0,0},
  {"3WayValveStand_CD_DHW",1,0,8,1,2,0,0},{"Boilertype",1,0,9,1,2,0,0},{"Blocking_input",1,0,10,1,2,0,0},
  {"Relaese_input",1,0,11,1,2,0,0},{"Release_WaitTime",1,0,12,1,2,0,0},{"Fluegas_valvetime",1,0,13,1,2,0,0},
  {"Status_report",1,0,14,1,2,0,0},{"Min_gaspressure",1,0,15,1,2,0,0},{"HRUactive",1,0,16,1,2,0,0},
  {"Mains_LN_active",1,0,17,1,2,0,0},{"Service_Notification",1,0,18,1,2,0,0},{"Service_hours",1,100,19,1,2,0,0},
  {"Service_burning ",1,100,20,1,2,0,0},{"Factor_avgflow",1,0,21,1,2,0,0},
  // Parameters 4 - offset 128
  {"DHWinGradient",1,-100,17,1,2,0,0},
  {"dTpumpOffset",1, 0,18,1,2,0,0},{"Offset_controlTemp",1,0,19,1,2,0,0},{"DHW_flowatRPmmin",1,-10,20,1,2,1,0},
 // Parameters 5 - offset 132
  {"Deairation_cycles",1,0,7,1,2,0,0},{"Gradient_dTMax_1",1,-100,8,1,2,1,0},{"Gradient_dTMa2_1",1,-100,9,1,2,2,0}
  // Parameters 6 - offset 148
 // {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, 
  // Parameters 7 - offset 164
 // {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {},
  // Parameters 8 - offset 178
 // {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}, {}
};


const int8_t moduleCnt = sizeof(boilerModule) / sizeof(boilerModule[0]); // count of Modules in struct

int wifi_errorcount = 0;
int mqtt_errorcount = 0;
int boiler_errorcount = 0;

uint32_t last_change = millis(); // timestamp last changed value update
uint32_t last_full_update = millis(); // timestamp last full update

const uint32_t changed_interval = 10000; // send updates for changed values every x milliseconds (default 10 seconds)
const uint32_t full_interval = 300000; // send full updates every x milliseconds (default 5 min)
const uint32_t failed_interval = 1000; // on failure wait x milliseconds (default 1 sec) and retry

byte rawData[74]; // Remeha boiler returns maximum of 74 byte messages

ESPTelnet telnet; // ESPTelnet for debug 
WiFiClient mqttClient;
PubSubClient MQTTclient(mqttClient);


void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_MSG(".");
    delay(500);
    if (wifi_errorcount++ > 10) ESP.reset();
  } 
  wifi_errorcount = 0;

}


void setupOTA() {
  ArduinoOTA.setHostname(HOSTNAME);

  if (strcmp(OTA_PASSWORD, "") != 0) {
    ArduinoOTA.setPassword(OTA_PASSWORD);
  }

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    DEBUG_MSG("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    DEBUG_MSG("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    char buffer[40];
    sprintf(buffer, "Progress: %u%%\r", (progress / (total / 100)));
    DEBUG_MSG(buffer);
  });

  ArduinoOTA.onError([](ota_error_t error) {
    char buffer[40];
    sprintf(buffer, "Error[%u]: ", error);
    DEBUG_MSG(buffer);
    if (error == OTA_AUTH_ERROR) {
      DEBUG_MSG("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      DEBUG_MSG("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      DEBUG_MSG("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      DEBUG_MSG("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      DEBUG_MSG("End Failed");
    }
  });
  
  ArduinoOTA.begin();
  DEBUG_MSG("OTA Ready");
}


void MQTTconnect() {

  MQTTclient.setServer(MQTT_SERVER,MQTT_PORT);
  MQTTclient.connect(HOSTNAME, MQTT_USER, MQTT_PASSWORD);
  DEBUG_MSG("Connecting to MQTT-server: " + String(MQTT_SERVER));

  while (! MQTTclient.connected()) {
    DEBUG_MSG(".");
    if (mqtt_errorcount++ > 10) ESP.reset();
    delay(500);
  }
}


bool getBoilerData(int modID) {

  while (Serial.available() > 0) { 
    Serial.read(); // flush input buffer before next request
  }

  // Send 10-byte command to boiler 
  Serial.write(boilerModule[modID].command, 10);
  DEBUG_MSG("Sending Request Data for Module: " + String(boilerModule[modID].name) + "-" + String(boilerModule[modID].index));
  // EXTENDED DEBUG 
  char cmdString [61];
  for (int i = 0; i < 10; i++) {
    char buffer [6];
    sprintf(buffer, "0x%02x ", boilerModule[modID].command[i]);
    strcat (cmdString, buffer);
  }
  DEBUG_MSG(String(cmdString));

  int SerialTimeout = millis() + 2000; 
  while (!Serial.available() && (SerialTimeout > millis()) ) {
    delay(100); // wait for response
  }
  
  if (Serial.available()) {
    DEBUG_MSG("Data available for reading");

    int read = Serial.readBytes(rawData, boilerModule[modID].length);
    DEBUG_MSG(String(read) + " Bytes received");
    boiler_errorcount = 0;

    // EXTENDED DEBUG 
    char rawString [500];
    for (int i = 0; i < read; i++) {
      char buffer [6];
      sprintf(buffer, "0x%02x ", rawData[i]);
      strcat (rawString, buffer);

    }
    DEBUG_MSG(String(rawString));

    return(1);

  } else {
    DEBUG_MSG("No response, try again");
    if ( boiler_errorcount++ > 50 ) ESP.reset();
    return(0);
  }
}


void decode(byte modID, byte topicID) {

  byte index = topics[topicID].index;
  byte length = topics[topicID].length;
  uint8_t type = topics[topicID].type;
  char output [18];

  switch (type) {

    case 1: 
    // type: Status (1 byte codes or 1 bit)
    {
      byte bit_index = topics[topicID].bit_prec;
      byte byteVal = rawData[index];
      byte bitVal;

      if (bit_index > 0) {
        itoa(bitRead(byteVal, bit_index-1), output, 10);
      } else {
        itoa(byteVal, output, 10);
        // to do: lookup table status codes
      }
      
      break;
    }

    case 2: 
    // type: numbers (int/float with multiply/divide)
    {
      int8_t mply_div = topics[topicID].mply_div;
      uint8_t precision = topics[topicID].bit_prec;
      bool sign = topics[topicID].sign;
      bool endian = boilerModule[modID].endian;
      int32_t intVal;

      // decode 
      if (length == 1) {
        // input/output: 1-byte unsigned integer
        uint8_t tmpInt = rawData[index];
        // convert to 2-byte int and recalculate if signed
        intVal = tmpInt - (sign * (tmpInt >> 7) *256);
      } else {
          // input/output 2-byte unsigned integer
          uint16_t tmpInt;
          if (endian) {
            //Big-endian
            tmpInt = rawData[index] * 256 + rawData[index + 1]; 
          } else {
            //Little-endian
            tmpInt = rawData[index + 1] * 256 + rawData[index]; 
          }
        // convert to 2-byte signed int and recalculate if signed
        intVal = tmpInt - (sign * (tmpInt >> 15) * 65536);
      }

      // translate 
      if (intVal == -32768) {
        // value 0x8000: not available 
        strcpy (output, "(n/a)");
      } else if (mply_div > 1) {
        // multiply as int, convert to output
        itoa((intVal * mply_div), output, 10);
      } else if (mply_div < -1) {
        // divide as float with precision, convert to output 
        dtostrf(float(intVal) / -mply_div, 1, precision, output); 
      } else {
        // convert to output 
        sprintf(output, "%d", intVal);
      }

      break;
    }

    case 3: 
    // type: 1 byte hexadecimal as decimal (0x99 as 99) multiply to Integer / divide to Float (prec.)
    {
      int8_t mply_div = topics[topicID].mply_div;
      uint8_t precision = topics[topicID].bit_prec;
      char hexAsDec[3];
      int8_t intVal;

      // decode
      sprintf(hexAsDec, "%02x", rawData[index]);
      intVal = atoi(hexAsDec);

      // translate 
      if (mply_div > 1) {
        // multiply (int), convert to output 
        itoa((intVal * mply_div), output, 10);
      } else if (mply_div < -1) {
        // divide (as float w precision), convert to output 
        dtostrf(float(intVal) / -mply_div, 1, precision, output); 
      } else {
        // convert to output 
        sprintf(output, "%d", intVal);
      }
      break;
    }

    case 4: 
    // ASCII string (up to 16 bytes)
    {

      byte buffer[17];

      // decode 
      memcpy(buffer, rawData + index, length); // read into buffer
      sprintf(output, "%s", buffer);         // format as ASCII
      // trim whitespace 
      char * ptrSpace = strchr(output, ' '); // search for whitespace
      if (ptrSpace) *ptrSpace = 0;           // and trim

      break;
    }

    case 5: 
    // Serial (5 bytes: 4 digits (2 bytes HEX), 1 letter (1 byte ASCII), 4 digits. Example: 0123Z0123)
    {
      // decode / format as Serial
      sprintf(output, "%02x%02x%c%02x%02x", rawData[index], rawData[index + 1], rawData[index + 2], rawData[index + 3], rawData[index + 4]);

      break;
    }

    default: {
      DEBUG_MSG("Decode error: Undefined type");

      break;
    }

  }

  // store output in struct
  strncpy (topics[topicID].old_value, topics[topicID].value, 16);
  strncpy (topics[topicID].value, output, 16);

}

bool publishMQTT(int modID, int topicID) {
  char topicPath[100] = MQTT_PREFIX;

  // concatenate and convert MQTT topic to String
  strcat(topicPath, boilerModule[modID].name);
  strcat(topicPath, "/");
  strcat(topicPath, topics[topicID].name);

  // publish MQTT
  if (MQTTclient.publish(topicPath, topics[topicID].value, boilerModule[modID].retained) ) {
    DEBUG_MSG(String("Published: ") + topicPath + String(" -> ") + String(topics[topicID].value));
    return(1);
  } else {
    DEBUG_MSG(String("Publish failed: ") + topicPath + String(" -> ") + String(topics[topicID].value));
   return(0);
  }

}


void setup() {
  Serial.begin(9600);
  Serial.setTimeout(250); // wait for serial to init

  setupWifi();
  
  // start telnet for debug
  telnet.begin();
  delay(5000); // give telnet client a shot at connecting before first message
  telnet.loop();
  DEBUG_MSG("WiFi connected");
  DEBUG_MSG(WiFi.localIP().toString());
    
  setupOTA();
  MQTTconnect();
  telnet.loop();

}

void loop() {
  if (!WiFi.isConnected() ) {
    DEBUG_MSG("WiFi disconnected. Start reconnect");
    setupWifi();
  }
  if (!MQTTclient.connected() ) {
    DEBUG_MSG("MQTT-client disconnected. Start reconnect");
    MQTTconnect();
  }


  bool full_update = 0; 
  // check if full update is due
  if (millis() - last_full_update >= full_interval) {
    last_full_update = millis();
    full_update = 1;
    DEBUG_MSG("FULL UPDATE");
  }
  // check if update is due
  if ( (millis() - last_change >= changed_interval) || (full_update) ) {

    // update last_change
    last_change = millis();
    // loop through all Modules
    for (int8_t modID = 0; modID < moduleCnt; modID++) { 

      // module is enabled 
      if ( boilerModule[modID].enable ) {

        // boiler data received
        if ( getBoilerData(modID) ) {
            // get offset
            uint8_t offset = boilerModule[modID].offset;

            // loop through all topics for this module start from offset
            for (int topicID = offset; topicID < boilerModule[modID].numVal + offset; topicID++) { 

               // decode start at offset
               decode(modID, topicID); 

                // Topic set to publish 
               if (topics[topicID].publish) {

                 // publish if value is changed or full update is due
                  if (strcmp(topics[topicID].value, topics[topicID].old_value) != 0) {

                     // publish updated topic value
                     publishMQTT(modID, topicID); 

                  } else if (full_update) {
                     // publish topic value (full update)
                     publishMQTT(modID, topicID); 

                  } else {
                 
                // needed to throttle (emulator only?)
                delay(150); 


              }

            } else {
             //Serial.println("value not published");
            }

          } // next value

          //Serial.println("[Result end]");
     
        } else {
          // nothing received; wait and retry 
          DEBUG_MSG(String("Failure - waiting ") + String(failed_interval) + String(" ms"));
          delay(failed_interval);
        }
      }
    }
  }  

 telnet.loop();
 MQTTclient.loop();
 ArduinoOTA.handle();

 delay(100); 

}
