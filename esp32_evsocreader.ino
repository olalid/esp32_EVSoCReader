/* 
 * Written for MACCHINA A0 OBD-II
 * Reads VIN, HV battery SoC, ambient temperature, 12V battery voltage, gear and odometer and transfers data to MQTT server
 */
#include <WiFi.h>
#include <WiFiMulti.h>
#include <esp32_can.h>
#include <PubSubClient.h>

// Configuration is stored in a separate file.
// Rename config_template.h as config.h and edit to provide the required details
#include "config.h"

WiFiMulti wifiMulti;
WiFiClient wifiClient;
PubSubClient MQTTclient(wifiClient);

#define APPNAME "EVSoCReader"

#define PID_VEHICLE_SPEED 0x0D
#define PID_CONTROL_MODULE_VOLTAGE 0x42
#define PID_AMBIENT_AIR_TEMPERATURE 0x46
#define PID_BATTERY_PACK_SOC 0x5B
#define PID_VIN 0x02

#define CAN_MODE_CURRENT 0x01
#define CAN_MODE_INFORMATION 0x09
#define CAN_MODE_CUSTOM 0x22

// The car can be using either 11-bit or 29-bit IDs as defined below.
#define LONG_SEND_ID 0x18DB33F1
#define LONG_RECV_ID 0x18DAF100
#define LONG_RECV_MASK 0x1FFFFF00
#define SHORT_SEND_ID 0x7DF
#define SHORT_RECV_ID 0x7E8
#define SHORT_RECV_MASK 0x7F8
#define LONGBC_RECV_ID 0x1FFF0000
#define LONGBC_RECV_MASK 0x1FFFF000
#define ODOMETER_ID 0x1FFF0120 
#define GEAR_ID 0x1FFF00A0

// The list of PIDs that we read. Not all cars will respond to everything.
#define NUM_PIDS 5
const uint16_t pids[][2] = {
  { CAN_MODE_INFORMATION, PID_VIN },
  { CAN_MODE_CURRENT,     PID_CONTROL_MODULE_VOLTAGE },
  { CAN_MODE_CURRENT,     PID_AMBIENT_AIR_TEMPERATURE },
  { CAN_MODE_CURRENT,     PID_BATTERY_PACK_SOC },
  { CAN_MODE_CURRENT,     PID_VEHICLE_SPEED }
};

// GPIO pin definitions
const int led_pin = 13;
const int can_en_pin = 21;

// Global variables
volatile bool dirty_data = 0;

volatile int lastKMPH=-1;
volatile int lastSoC=-1;
volatile float lastVoltage=-1;
volatile int lastAmbient=-100;
volatile char lastGear='U';
volatile unsigned int lastODO=-1;
volatile unsigned int lastSupported[7];
volatile long lastRssi=-1;
volatile unsigned long recv_l=0;
volatile char globalVIN[18];

void setup()
{

  // Disable LED power
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, LOW);

  // Pull CAN S pin low for non-idle state.
  pinMode(can_en_pin, OUTPUT);
  digitalWrite(can_en_pin, LOW);
  
  // Start Serial port
  Serial.begin(115200);
  Serial.println(APPNAME);
  Serial.println("Serial Ready...");

  // Init CAN pins, baudrate and setup filter and callback for expected response IDs.
  CAN0.setCANPins(GPIO_NUM_4, GPIO_NUM_5);
  CAN0.begin(500000);  
  int filter;
  filter = CAN0.watchForRange(SHORT_RECV_ID, SHORT_RECV_MASK);
  CAN0.setCallback(filter, receiveCallback);
  filter = CAN0.watchFor(LONG_RECV_ID, LONG_RECV_MASK);
  CAN0.setCallback(filter, receiveCallback);
  filter = CAN0.watchFor(LONGBC_RECV_ID, LONGBC_RECV_MASK);
  CAN0.setCallback(filter, receiveBCCallback);
  Serial.println("CAN Ready...");

  // Configure the APs we want to connect to.
  int i;
  for(i = 0; i < NUM_SSID; i++)
    wifiMulti.addAP(ssid[i], pass[i]);
  Serial.println("WiFi Ready...");

  // Configure the MQTT lib
  MQTTclient.setServer(mqtt_server, atoi(mqtt_port));
  Serial.println("MQTT ready...");

  // Reduce power, we do not need high performance
  setCpuFrequencyMhz(80);
   
  for(int i = 0; i < 18; i++)
    globalVIN[i] = 0;
}

void loop()
{
  
  static unsigned long wifi_l = 0;
  unsigned long wifi_t = millis();

  // If we did not receive any CAN messages in 5 minutes, the car is sleeping and we can turn off WiFi
  if ((wifi_t - recv_l) > 300000) {
    wifiOff();
  } else { // We have recevied data recently
    // If 30 seconds has passed since last time we tried, attempt connection
    if ((wifi_t - wifi_l) > 30000) {
      wifi_l = wifi_t;
      // Attempt connection
      if(wifiOn() == WL_CONNECTED) {
        long rssi = WiFi.RSSI();
        Serial.println("WiFi connected");
        Serial.print("SSID: ");
        Serial.println(WiFi.SSID());
        Serial.print("RSSI: ");
        Serial.println(rssi);
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        if(lastRssi != rssi) {
          lastRssi = rssi;
          dirty_data = 1;
        }
        // If data has been updated, send to MQTT server
        if(dirty_data) {
          Serial.print("VIN: ");
          Serial.println((const char *)globalVIN);
          Serial.print("SoC: ");
          Serial.println(lastSoC);
          Serial.print("Voltage: ");
          Serial.println(lastVoltage);
          Serial.print("Ambient: ");
          Serial.println(lastAmbient);
          Serial.print("ODO: ");
          Serial.println(lastODO);
          Serial.print("Gear: ");
          Serial.println(lastGear);
          sendMQTT();
          
        }
      }  
    }
  }

  static unsigned long can_l = 0;
  unsigned long can_t = millis();
  
  // Send ODB2 update request once every 2 seconds
  // Each request is sent both to 11-bit and 29-bit IDs. Only one should render a response.
  if ((can_t - can_l) > 2000) {
    sendCAN();
    can_l = can_t;
  }
  
  // Sleep 100 ms.
  delay(100);
  
}

void sendMQTT()
{

  char mqttStr[16];

  Serial.println("Connecting to MQTT server.");
  
  // Try to connect to MQTT server
  if(MQTTclient.connect(APPNAME)) {
    Serial.println("Connected...");
    MQTTclient.loop();
    if(lastSoC != -1) {
      snprintf(mqttStr, 16, "%d", lastSoC);
      MQTTclient.publish(mqtt_status_topic_soc, mqttStr, true);
      MQTTclient.loop();
    }
    if(lastVoltage != -1) {
      snprintf(mqttStr, 16, "%0.2f", lastVoltage);
      MQTTclient.publish(mqtt_status_topic_voltage, mqttStr, true);
      MQTTclient.loop();
    }
    if(lastAmbient != -100) {
      snprintf(mqttStr, 16, "%d", lastAmbient);
      MQTTclient.publish(mqtt_status_topic_ambient, mqttStr, true);
      MQTTclient.loop();
    }
    if(lastODO != -1) {
      snprintf(mqttStr, 16, "%d", lastODO);
      MQTTclient.publish(mqtt_status_topic_odo, mqttStr, true);
      MQTTclient.loop();      
    }
    if(lastGear != 'U') {
      snprintf(mqttStr, 16, "%c", lastGear);
      MQTTclient.publish(mqtt_status_topic_gear, mqttStr, true);
      MQTTclient.loop();      
    }
    if(strlen((const char*)globalVIN) == 17) {
      MQTTclient.publish(mqtt_status_topic_vin, (const char*)globalVIN, true);
      MQTTclient.loop();            
    }    
    snprintf(mqttStr, 16, "%d", lastRssi);
    MQTTclient.publish(mqtt_status_topic_rssi, mqttStr, true);
    MQTTclient.loop();
    MQTTclient.disconnect();
    MQTTclient.loop();
    while(MQTTclient.connected()) {
      MQTTclient.loop();
      delay(1);
    }
    dirty_data = 0;
    return;
  } else {
    Serial.println("Failed to connect to MQTT server.");
  }
  
}

uint8_t wifiOn()
{

  WiFi.disconnect(false);
  return wifiMulti.run();
  
}

void wifiOff()
{

  WiFi.disconnect(true);
      
}

void sendCAN()
{

  static unsigned long canLoop = 0;

  // Send the requests to both 11 and 29 bit IDs.
  sendPIDRequest(SHORT_SEND_ID, pids[canLoop][0], pids[canLoop][1]);
  sendPIDRequest(LONG_SEND_ID, pids[canLoop][0], pids[canLoop][1]);
  canLoop++; 
  if(canLoop >= NUM_PIDS) {
    canLoop = 0;
  }

}

void sendPIDRequest(uint32_t id, uint8_t mode, uint16_t PID)
{

  CAN_FRAME frame;
  frame.id = id;
  frame.extended = (id > 0x7ff)?1:0; // Set to 1 if 29-bit address.
  frame.length = 8;
  frame.rtr = 0;

  // Set unused data bytes to 0 
  for (int i = 3; i < 8; i++)
    frame.data.bytes[i] = 0x00;

  // For mode 0x01 and 0x09 this is the packet format
  if(mode == 0x01 || mode == 0x09) {
    frame.data.bytes[0] = 2; //2 more bytes to follow
    frame.data.bytes[1] = mode;
    frame.data.bytes[2] = PID;
  }
  // For mode 0x22 (custom) this is the packet format.
  if(mode == 0x22) {
    frame.data.bytes[0] = 3; //3 more bytes to follow
    frame.data.bytes[1] = mode;
    frame.data.bytes[2] = (PID >> 8) & 0xff;
    frame.data.bytes[3] = PID & 0xff;
  }

  // Send the frame
  int ret = CAN0.sendFrame(frame);
  if(ret)
    Serial.println("S");

}

void sendFlowRequest(uint32_t id)
{

  CAN_FRAME frame;
  frame.extended = (id > 0x7ff)?1:0; // Set to 1 if 29-bit address.
  frame.length = 8;
  frame.rtr = 0;
  // The input ID is the senders ID, so must be converted before sending request
  // For 29 bit addresses the 2 least significant bytes sholuld swap place.
  // For 11 bit addresses the adress should be decreased by 8.
  if(id > 0x7ff)
    frame.id = (id & 0xffff0000) | ((id & 0xff) << 8) | ((id & 0xff00) >> 8);
  else
    frame.id = id - 8; 

  // Set unused data bytes to 0 
  for (int i = 1; i < 8; i++)
    frame.data.bytes[i] = 0x00;

  // Flow frame request
  frame.data.bytes[0] = 0x30;

  // Send the frame
  int ret = CAN0.sendFrame(frame);
  if(ret)
    Serial.println("F");

}

// This function is called by the CAN lib when a matching frame is received
void receiveCallback(CAN_FRAME *frame)
{

  int RPM;
  int KMPH;
  int Ambient;
  float Voltage;
  int SoC;
  unsigned int supported;
  int index;
  static int mode;
  static int len;
  static int pid;

  // Print the received frame
  Serial.print("R ");
  Serial.print(frame->id, HEX);
  for(int i=0; i < 8; i++) {
    Serial.print(" ");
    Serial.print(frame->data.bytes[i], HEX);
  }
  Serial.println("");

  // Check the frame code
  switch((frame->data.bytes[0] & 0xf0) >> 4) {

    case 0: // Single frame
      len = frame->data.bytes[0];
      mode = frame->data.bytes[1];
      pid = frame->data.bytes[2];
      break;

    case 1: // First frame of multiple frame
      len =  frame->data.bytes[1] | ((frame->data.bytes[0] & 0xf) << 8) - 6;
      mode = frame->data.bytes[2];
      pid =  frame->data.bytes[3];
      // Ask for the consecutive frames
      sendFlowRequest(frame->id); //0x18DA10F1); // 18DAF118
      break;
      
    case 2: // Consecutive frames of multiple
      // Note, mode and pid is remembered from last frame
      len = len - 7;
      index = frame->data.bytes[0] & 0xf;
      break;
      
    default:
      mode = -1;
      break;
  }

  // Is this a mode 1 response?
  if(mode == 0x41) {
    
    // Save a timestamp
    recv_l = millis();
  

    // Decode the data
    switch (frame->data.bytes[2]) {
      
      case PID_VEHICLE_SPEED:
        KMPH = frame->data.bytes[3];
        if (KMPH!=lastKMPH) {
          lastKMPH = KMPH;
          dirty_data = 1;
        }
        break;
  
      case PID_BATTERY_PACK_SOC:
        SoC = (int)((frame->data.bytes[3] * 100.0/255.0)+0.5);
        if (SoC != lastSoC) {
          lastSoC = SoC;
          dirty_data = 1;
        }
        break;
  
      case PID_CONTROL_MODULE_VOLTAGE:
        Voltage = ((frame->data.bytes[3] << 8) | frame->data.bytes[4])/1000.0;
        if (Voltage != lastVoltage) {
          lastVoltage = Voltage;
          dirty_data = 1;
        }
        break;
  
      case PID_AMBIENT_AIR_TEMPERATURE:
        Ambient = frame->data.bytes[3] - 40;
        if (Ambient != lastAmbient) {
          lastAmbient = Ambient;
          dirty_data = 1;
        }
        break;
      
    }
    
  // Is this a mode 9 response?
  } else if(mode == 0x49) {

    switch (pid) {
      
      case PID_VIN:
        if(frame->data.bytes[0] == 0x10) {
          for(int i = 0; i < 3; i++)
            globalVIN[i] = frame->data.bytes[i+5];
        } else {
          if(index == 1 || index == 2)
            for(int i = 0; i < 7 ; i++)
              globalVIN[3+(index-1)*7+i] = frame->data.bytes[i+1];
        }
        break;
        
    }
 
  }
  
}


// This function is called by the CAN lib when a matching broadcast frame is received
// These are likeley Polestar specific, but might work on some Volvo models as well
void receiveBCCallback(CAN_FRAME *frame)
{

  unsigned int ODO;
  char Gear;
  static char gearTranslate[] = {'P', 'R', 'N', 'D'};

  if(frame->id == ODOMETER_ID) {
    ODO = ((frame->data.bytes[0] & 0x0f) << 16) |
          (frame->data.bytes[1] << 8) |
          frame->data.bytes[2];
    if(ODO != lastODO) {
      lastODO = ODO;
      dirty_data = 1;
    }
  }

  if(frame->id == GEAR_ID) {
    Gear = gearTranslate[frame->data.bytes[6] & 3];
    if(Gear != lastGear) {
      lastGear = Gear;
      dirty_data = 1;
      Serial.print("Gear changed: ");
      Serial.println(Gear);
    }
  }
  
}
