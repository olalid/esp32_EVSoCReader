# esp32_EVSoCReader

## Description
Some electric or hybrid vehicles lack a public API to be able to integrate its charging procedure in to an home automation system to display and act up on it's battery stage of charge (SoC).
This project aims to solve that by using the OBD2 interface in the vehicle to read SoC and a few ohter things and transfer this information over to an MQTT server when the vehicle is within WiFi range.
To run this code, this OBD2 hardware is required: [Macchina A0](https://www.macchina.cc/catalog/a0-boards/a0-under-dash)

## How it works
OBD2 is a standardized interface that most modern cars implements to provide an interface that is used at workshops to diagnose faults.
There are usually many vendor specific features that enable workshops with the right equipment to read and change a lot of details and even do software updates to a vehicle.
But, there are also a smaller set of standarized features, known as "PIDs", which can be read in the same way in most vehicles.
It is not mandatory to implement everything, but at least some of these are usually implemented.
This code reads the 0x42, 0x46, and 0x5B PIDs. (Control module voltage, Ambient temperature, Hybrid battery pack remaining life). See [Wikipedia](https://en.wikipedia.org/wiki/OBD-II_PIDs) for more details on OBD2 PIDs.

To transfer the information to your home automation it will try to conenct to a list of pre-programmed WiFi networks, and when it succeds it will transfer the information to a pre-programmed MQTT server.

Note that the OBD2 interface typically only works when the vehcile is active (e.g. ignition on). So this means that the SoC will not update during charging.
The code turns of WiFi when the vehicle is inactive to save some power.

## Compilation
Download the [Arduino IDE](https://www.arduino.cc/en/software) and download this project to a folder on your computer.
Install support for ESP32 in Arduino IDE by selecting the Tools->Board->Board Manager... from the menu, search for esp32 and install.
Install pubsubclient lib by selecting the Sketch->Include Library->Library Manager... from the menu, search for pubsubclient and install.
Open the esp32_evsocreader.ino file in the Arduino IDE and copy the config_template.h to config.h.
Edit the config.h with SSIDs, passwords and MQTT server information according to your needs.
Configure the Arduino IDE to compile for "ESP32 Dev Module".
Connect the Macchina A0 to the computer using its USB port and select the correct port under Tools->Port.
Press the Upload button and the code should compile and upload to the Macchina A0.

## Supported vehicles
In theory, this code should support all cars that use 500 kbit/s CAN in their OBD2 interface, if they respond to either 11 or 29 bit IDs and if they implement the required PIDs.
At the point in time when this is written, the code has only been tested using a Polestar 2.

## Example configuration for Home Assistant

Assuming that Home Assistant is already configured to use an MQTT server, the following can be used to set up sensors for the provided data:
```
sensor:
  - platform: mqtt
    name: "EV battery SoC"
    state_topic: "EV/soc"
    unit_of_measurement: "%"
    device_class: battery
  - platform: mqtt
    name: "EV ambient temperature"
    state_topic: "EV/ambient"
    unit_of_measurement: "°C"
    device_class: temperature
  - platform: mqtt
    name: "EV WiFi RSSI"
    state_topic: "EV/rssi"
    unit_of_measurement: "dBm"
    device_class: signal_strength
  - platform: mqtt
    name: "EV 12V battery voltage"
    state_topic: "EV/voltage"
    unit_of_measurement: "V"
    device_class: voltage
```

As mentioned above, data can only be read when the car is active, which means that typically a SoC value will be received when the car returns home and is parked and it will not update during charging.
However, if you have an EVSE (charing station) that is also integrated in Home Assistant and provides a sensor for charing session energy, it is possible to get a fairly well working estimation of the current SoC.
By muptilpying the energy supplied in the charing session by a constant and adding it to the last known SoC, the current SoC can be estimated within a couple of %.
The example below is for a Polestar 2 and an Easee charger.
The easee charger supplies a charger status sensor and a charging session energy sensor, while the last known SoC is supplied by this project.
When the status is disconnected, it is assumed that the last known SoC is the more likely correct value (i.e. when you disconned the car and drive away, the SoC will likely be updated before you leave WiFi range).
When the status is not disconnected, the session enery is multiplied by constant 1.3456 and added to the SoC.
The constant will be different depending on the specific vehicle and can be first estimated by dividing 100 with the battery size in kWh.
But it is better to use actual data from a charging session, example:
If you start charing with 20% battery SoC and charge to 90%, you will have charged 90-20 = 70% of the battery.
The energy used to charge this was 52,02 kWh.
The constant will be 70/52,02 = 1,3456.

```
sensor:
  - platform: template
    sensors:
      ev_calculated_soc:
        friendly_name: "EV calculated SoC"
        value_template: "{% if states('sensor.charger_status') == 'disconnected' %}{{ states('sensor.ev_battery_soc') }}{% else %}{{ ((states('sensor.ev_battery_soc')|float) + ((states('sensor\
.charger_session_energy')|float) * 1.3456) +0.5) | int }}{% endif %}"
        unit_of_measurement: "%"
        device_class: battery
```
