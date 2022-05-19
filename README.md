# MQTT smart fan

This is an updated project of my homekit smart fan that uses ESP32 S2 to run a smart fan.

## Requirements

- [PubSubClient](https://github.com/knolleary/pubsubclient/releases/tag/v2.8)

- [OneWireNG](https://github.com/pstolarz/OneWireNg)

- [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)

- [ButtonFever](https://github.com/mickey9801/ButtonFever)

## Features

- MQTT interface

- Support for 3-speed fans

- Support for internal DS18B20 Temperature sensor

- Supprot for dimmable led indicators and analog photoresistor or phototransistor for auto-brightness

## Usage

First, apply your local WiFi network credentials and MQTT broker addresss in the code. Then, apply pin configuration and custom MQTT topics. 

## Example Home Assistant configuration.yaml

The fan

```
fan:
  - platform: mqtt
    name: "Fan"
    state_topic: "thafan/on/state"
    command_topic: "thafan/on/set"
    percentage_state_topic: "thafan/spd/state"
    percentage_command_topic: "thafan/spd/set"
    qos: 0
    speed_range_min: 1
    speed_range_max: 3
    payload_on: "t"
    payload_off: "f"
```

The indicator lights

```
light:
  - platform: mqtt
    name: "Fan light"
    state_topic: "thafan/led/state"
    command_topic: "thafan/led/set"
    brightness_state_topic: "thafan/led/brightness/state"
    brightness_command_topic: "thafan/led/brightness/set"
    qos: 0
    on_command_type: 'brightness'
    payload_on: "t"
    payload_off: "f"
    optimistic: false
    brightness_scale: 1023

```

The temperature sensor

```
sensor:
  - platform: mqtt
    state_topic: "thafan/temp"
    device_class: temperature
    name: "fan temp"
    unit_of_measurement: "°C"
    unique_id: "fan_temp"
```
