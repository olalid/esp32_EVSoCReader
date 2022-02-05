// This is a list of SSIDs and passowrds for the WiFi networks you want to connect to.
#define NUM_SSID 2
const char *ssid[] = {
  "SSID1",
  "SSID2"
};
const char *pass[] = {
  "PASSWORD1",
  "PASSWORD2"
};

// MQTT configuration, change at least server to your server address/ip.
char mqtt_server[40] = "MQTT-SERVER";
char mqtt_port[6] = "1883";
char mqtt_user[64] = "";
char mqtt_pass[64] = "";
char mqtt_status_topic_soc[64] = "EV/soc";
char mqtt_status_topic_voltage[64] = "EV/voltage";
char mqtt_status_topic_ambient[64] = "EV/ambient";
char mqtt_status_topic_rssi[64] = "EV/rssi";

