#include <Arduino.h>
#include <IRrecv.h>
#include <IRsend.h>
#include <IRremoteESP8266.h>
#include <IRutils.h>
#include <IRac.h>

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ArduinoJson.h>

#define action_led D0
#define state_led D1
#define btn_config D8
//#define btn_lear
#define IR_LED D2
#define recvPin D5

unsigned int buttonTimer = 0;
unsigned int longPressTime = 2000;

boolean buttonActive = false;
boolean longPressActive= false;
boolean smartConfigStart = false;
boolean upToken = false;
boolean learn = false;
boolean control = false;

const char* mqtt_server = "chika.gq";
const int mqtt_port = 2502;
const char* mqtt_user = "chika";
const char* mqtt_pass = "2502";

char* IR_module = "CA-IRX0.01";
char* topicLearn = "CA-IRX0.01/learn";
char* topicControl = "CA-IRX0.01/control";
char* topicDone = "CA-IRX0.01/done";
char* topicCancel = "CA-IRX0.01/cancel";


String IR_value;   // value IR using send data to control

const uint16_t kCaptureBufferSize = 1024;
const uint8_t kTimeout = 50;
const uint16_t kFrequency = 38000;

char data_char[500];
char protocol_char[50];


IRrecv irrecv(recvPin,kCaptureBufferSize,kTimeout, false);
IRsend irsend(IR_LED);
IRac ac(IR_LED);

decode_results results;

Ticker ticker;

WiFiClient esp;
PubSubClient client(esp);

void setup() {
  Serial.begin(115200);
  Serial.println("IR Device is ready");

  irsend.begin();
  irrecv.enableIRIn(); // Start the receiver
  Serial.println("IRrecv is currently running and waiting for IR message");
  Serial.println(recvPin);
  
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.mode(WIFI_STA);

  pinMode(action_led,OUTPUT);
  pinMode(btn_config,INPUT);
  pinMode(state_led,OUTPUT);

  setupAC();
  
  delay(10000);
  if(!WiFi.isConnected()){
    startSmartConfig();
  }else{
    digitalWrite(state_led,HIGH);
    Serial.println("WIFI CONNECTED");
    Serial.println(WiFi.SSID());
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  }

  

  client.setServer(mqtt_server,mqtt_port);
  client.setCallback(callback);
}


//-------------------------------------------------------------
void loop() {
  longPress();
  if(WiFi.status() == WL_CONNECTED){
    if(client.connected()){
      client.loop();
      
    if(control){
      Serial.println("IR_send");
      StaticJsonDocument<500>JsonDoc;
      DeserializationError error = deserializeJson(JsonDoc,IR_value);
      String protocol_str = JsonDoc["protocol"];
      uint16_t bits = JsonDoc["bit"];
      uint16_t size = JsonDoc["size"];
      uint64_t irVal = JsonDoc["value"];

      uint16_t arraySize = JsonDoc["state"].size();
      uint8_t stateVal[arraySize + 1];
      for(uint16_t i = 0 ; i < arraySize ; i++){
        stateVal[i] = JsonDoc["state"][i].as<uint8_t>();
      }

      protocol_str.toCharArray(protocol_char,protocol_str.length() + 1);
      decode_type_t protocol = strToDecodeType(protocol_char);

      if(hasACState(protocol)){
        irsend.send(protocol, stateVal, size / 8);
        Serial.println("Send AC");
      }else if(protocol_str == "UNKNOWN"){
        irsend.sendNEC(irVal);
        Serial.println("send Unknown");
      }else{
        irsend.send(protocol,irVal,bits);
        Serial.println("Send has protocol");
      }
      
      control = false;
    }
    
    while(learn){
      digitalWrite(state_led,LOW);
      tick();
      client.loop();
      if(irrecv.decode(&results)){
        uint16_t size = getCorrectedRawLength(&results);
        StaticJsonDocument<1000>JsonDoc;
        JsonDoc["protocol"] = typeToString(results.decode_type);
        JsonDoc["bit"] = results.bits;
        JsonDoc["size"] = size;
        JsonDoc["value"] = results.value;
        
        JsonArray arrState = JsonDoc.createNestedArray("state");
        for(uint16_t i = 0 ; i < results.bits / 8 ; i++){
          arrState.add(results.state[i]);
        }

        String data_str;
        serializeJson(JsonDoc,data_str);
        Serial.println(data_str);
        data_str.toCharArray(data_char,data_str.length() + 1);
        for(uint16_t i = 0 ; i < data_str.length() + 1; i++){
          Serial.print(data_char[i]);
        }
        Serial.println();
        client.publish(topicDone,data_char); // publish funtion expect char[]
        client.publish(topicLearn,"DONE");
        learn = false;
        digitalWrite(state_led,HIGH);
        digitalWrite(action_led,LOW);
        irrecv.resume();
        delay(200);
     }
     delay(200);
    }

    defaultLed();
    
    }else{
      reconnect();
    }
  }else {
    Serial.println("WiFi Connected Fail");
    WiFi.reconnect();
  }
  irrecv.resume();
  delay(200);
}

//----------------------------------------------------------------------
void defaultLed(){
      digitalWrite(state_led,HIGH);
      digitalWrite(action_led,LOW);
}

void callback(char * topic, byte* payload , unsigned int length){
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String data;
  String mtopic = (String)topic;
  
  for(int i = 0; i < length; i++){
    data += (char)payload[i];
  }
  Serial.println(data);

  if(mtopic == "CA-IRX0.01/control"){
    IR_value = data;
    Serial.println(IR_value);
    control = true;
  }
    
  if(mtopic == "CA-IRX0.01/learn"){
    Serial.println("learn");
    if(data == "DONE"){
     learn = false;
    }else learn = true;
  }

  if(mtopic == "CA-IRX0.01/cancel"){
    learn = false;
  }
}

void reconnect(){
    Serial.println("Attempting MQTT connection ...");
    String clientId = "ESP8266Client-testX";
    clientId += String(random(0xffff),HEX);
    if(client.connect(clientId.c_str(),mqtt_user,mqtt_pass)){
      Serial.println("connected");
      client.subscribe(topicLearn);
      client.subscribe(topicControl);
      client.subscribe(topicCancel);
    }else {
      Serial.print("MQTT Connected Fail, rc = ");      Serial.print(client.state());
      Serial.println("try again in 5 seconds");
    }
}

void setupAC(){
  ac.next.protocol = decode_type_t::TOSHIBA_AC;  // Set a protocol to use.
  ac.next.model = 1;  // Some A/Cs have different models. Try just the first.
  ac.next.mode = stdAc::opmode_t::kCool;  // Run in cool mode initially.
  ac.next.celsius = true;  // Use Celsius for temp units. False = Fahrenheit
  ac.next.degrees = 25;  // 25 degrees.
  ac.next.fanspeed = stdAc::fanspeed_t::kMedium;  // Start the fan at medium.
  ac.next.swingv = stdAc::swingv_t::kOff;  // Don't swing the fan up or down.
  ac.next.swingh = stdAc::swingh_t::kOff;  // Don't swing the fan left or right.
  ac.next.light = false;  // Turn off any LED/Lights/Display that we can.
  ac.next.beep = false;  // Turn off any beep from the A/C if we can.
  ac.next.econo = false;  // Turn off any economy modes if we can.
  ac.next.filter = false;  // Turn off any Ion/Mold/Health filters if we can.
  ac.next.turbo = false;  // Don't use any turbo/powerful/etc modes.
  ac.next.quiet = false;  // Don't use any quiet/silent/etc modes.
  ac.next.sleep = -1;  // Don't set any sleep time or modes.
  ac.next.clean = false;  // Turn off any Cleaning options if we can.
  ac.next.clock = -1;  // Don't set any current time if we can avoid it.
  ac.next.power = false;  // Initially start with the unit off.
}



//--------------------------------------------------------------------------------

void longPress(){
  if (digitalRead(btn_config) == HIGH) {
    Serial.println(digitalRead(btn_config));
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      longPressActive = true;
      digitalWrite(state_led,LOW);
      startSmartConfig();
    }
  } else {
    if (buttonActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
      } 
      buttonActive = false;
    }
  }
}

void tick(){
  int state = digitalRead(action_led);
  digitalWrite(action_led,!state);
}

boolean startSmartConfig(){
  int t = 0;
//  ignore = 0;
  Serial.println("Smart Config Start");
  WiFi.beginSmartConfig();
  delay(500);
  ticker.attach(0.1,tick);
  while(WiFi.status() != WL_CONNECTED){
    t++;
    Serial.print(".");
    delay(500);
    if(t > 120){
      Serial.println("Smart Config Fail");
      smartConfigStart = false;
      ticker.attach(0.5,tick);
      delay(3000);
      exitSmartConfig();
      return false;
    }
  }
  smartConfigStart = true;
  Serial.println("WIFI CONNECTED");  
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  Serial.println(WiFi.SSID());
  exitSmartConfig();
  return true;
}

void exitSmartConfig(){
  WiFi.stopSmartConfig();
  ticker.detach();
  digitalWrite(action_led,LOW);
  digitalWrite(state_led,HIGH);
}
