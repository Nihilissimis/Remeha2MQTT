#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>  
#include "Remeha2MQTT.h"
#include "ESPTelnet.h"          

//const byte getID1[10] = {0x02,0xfe,0x00,0x05,0x08,0x01,0x0b,0xd4,0x9c,0x03}; // for future use: command string to request ID from Boiler
const byte cmdSampleData[10] = {0x02,0xfe,0x01,0x05,0x08,0x02,0x01,0x69,0xab,0x03}; // command string to request sample data from the boiler

byte rawData[74]; // Remeha boiler sends 74-byte messages

int changed_interval = 10000; // send updates for changed values every 10 seconds
int full_interval = 300000; // send full updates every 5 minutes
int failed_interval = 1000; // on failure wait 1 sec and retry

struct data {
  String name; // Variable name
  bool publish; // Publish to MQTT or not
  byte index; // Location in bytestring (start at 1)
  byte length;  // Number of bytes in bytestring
  byte bit_index; // If > 0: is bitmap (bit index starts at 1)
  int value; // Values are int
  int old_value; // Value change track
  unsigned long last_update; // Millis() last update
};

// struct array static data (name, publish, index, length, bit_index)
data values[] = {
  {"Aanvoer_temperatuur",1,7,2},{"Retour_temperatuur",1,9,2},{"Zonneboiler_temperatuur",0,11,2},{"Buiten_temperatuur",1,13,2},
  {"Boiler_temperatuur",0,15,2},{"Automaat_temperatuur",0,19,2},{"Ruimte_temperatuur",0,21,2},{"CV_setpunt",1,23,2},
  {"SWW_setpunt",1,25,2},{"Ruimte_setpunt",0,27,2},{"Ventilator_setpunt",1,29,2},{"Ventilator_toeren",1,31,2},
  {"Ionisatie_stroom",0,33,1},{"Intern_setpunt",1,34,2},{"Beschikbaar_vermogen",0,36,1},{"Pomp_snelheid",1,37,1},
  {"Gevraagd_vermogen",0,39,1},{"Geleverd_vermogen",0,40,1},
  // bitmaps
  {"Modulerende_regelaar",0,43,1,1},{"Warmtevraag_modulerend",0,43,1,2},{"Warmtevraag_aan-uit",1,43,1,3},{"Vorstbeveiliging",0,43,1,4},
  {"SWW_ecostand",0,43,1,5},{"SWW_blokkering",0,43,1,6},{"Anti_Legionella",0,43,1,7},{"SWW_warmtevraag",1,43,1,8},
  {"Ingang_blokkerend",0,44,1,1},{"Ingang_vrijgave",0,44,1,2},{"Ionisatie",0,44,1,3},{"SWW_tapschakelaar",0,44,1,4},
  {"Gasdruk_minimaal",0,44,1,6},{"CV_ingeschakeld",0,44,1,7},{"SWW_ingeschakeld",0,44,1,8},{"Gasklep",1,45,1,1},
  {"Onsteking_actief",0,45,1,3},{"3-wegklep_positie",0,45,1,4},{"Externe_3-wegklep",0,45,1,5},{"Externe_gasklep",0,45,1,7},
  {"Pomp_actief",1,46,1,1},{"Boilerpomp",0,46,1,2},{"Externe_CV_pomp",0,46,1,3},{"Status_rapport",0,46,1,5},
  {"Opentherm_SmartPower",0,46,1,8},
  // end bitmaps
  {"Status",1,47,1},{"Vergrendeling",0,48,1},{"Blokkering",1,49,1},{"Sub-status",1,50,1},
  {"Waterdruk",1,56,1},{"Regel_temperatuur",1,58,2},{"SWW_Tapdebiet",1,60,2},{"Solar_temperatuur",0,63,2}
};

ESPTelnet telnet; // ESPTelnet for debug 
WiFiClient mqttClient;
PubSubClient MQTTclient(mqttClient);

void setupWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  DEBUG_MSG("Connecting to WiFi: " + String(WIFI_SSID));

  while (WiFi.status() != WL_CONNECTED) {
    DEBUG_MSG(".");
    delay(500);
  }
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
    delay(500);
  }
}

bool getSampleData() {
  while (Serial.available() > 0) { 
    Serial.read(); // flush input buffer before next request
  }

  Serial.write(cmdSampleData,10);
  DEBUG_MSG("Sending 'request data' command to boiler");
  delay(500); // wait for response

  if (Serial.available()) {
    DEBUG_MSG("Data available for reading");
    
    int read = Serial.readBytes(rawData, 74)
    DEBUG_MSG(String(read) + "Bytes received");
    return(1);

  } else {
    DEBUG_MSG("No response, try again");
    return(0);

  }
}


int decode (byte index, byte length, byte bit_index) {
  int byteVal;
  int bitVal;

  while (length-- > 0) {
    byteVal += (rawData[index + length] << (length)*8);
  }

  if (bit_index > 0) {
    bitVal = bitRead(byteVal,bit_index-1);
    return bitVal;
  } else if (byteVal > 32767) {
    return 65535;
  } else {
    return byteVal;
  }
}

bool publishMQTT(int i) {
  char value[5];
  char topic[57];
  char buffer[30];

  itoa(values[i].value,value,10); // convert int value to char 

  // concatenate and convert MQTT topic to String
  strcpy(topic,MQTT_PREFIX);
  values[i].name.toCharArray(buffer, values[i].name.length()+1);
  strcat(topic, buffer);

  // publish MQTT
  if (MQTTclient.publish(topic, value) ) {
    DEBUG_MSG(String("Published: ") + topic + String(" -> ") + String(values[i].value));
    values[i].old_value = values[i].value;
    values[i].last_update = millis();
    return(1);
  } else {
    DEBUG_MSG(String("Publish failed: ") + topic + String(" -> ") + String(values[i].value));
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

  if (getSampleData()) {

    for (int i = 0; i < 51; i++) {
      values[i].value = decode(values[i].index, values[i].length, values[i].bit_index); 
      //DEBUG_MSG(values[i].name); // extended debug

      if (  (values[i].publish) && 
            ( 
              (values[i].value != values[i].old_value) || 
              (millis() - values[i].last_update > full_interval) 
            ) 
          ) { 
        // value to publish
        publishMQTT(i);
      }
    } // next value

    DEBUG_MSG(String("Waiting ") + String(changed_interval) + String(" ms"));
    delay(changed_interval); //

  } else {
    // nothing received wait and retry 
    DEBUG_MSG(String("Waiting ") + String(failed_interval) + String(" ms"));
    delay(failed_interval);
  }

  telnet.loop();
  MQTTclient.loop();
  ArduinoOTA.handle();

}