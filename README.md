# MQTT smart fan

This is an updated project of my homekit smart fan that uses ESP32 S2 to run a smart fan.

## Requirements

- [PubSubClient]([Release v2.8 · knolleary/pubsubclient · GitHub](https://github.com/knolleary/pubsubclient/releases/tag/v2.8)[Release v2.8 · knolleary/pubsubclient · GitHub](https://github.com/knolleary/pubsubclient/releases/tag/v2.8))

- [OneWireNG](https://github.com/pstolarz/OneWireNg)

- [DallasTemperature](https://github.com/milesburton/Arduino-Temperature-Control-Library)

- [ButtonFever](https://github.com/mickey9801/ButtonFever)

## Features

- MQTT interface

- Support for 3-speed fans

- Support for internal DS18B20 Temperature sensor

- Supprot for dimmable led indicators and analog photoresistor or phototransistor for auto-brightness

## Usage

First, apply your local WiFi network credentials and MQTT broker addresss in the code. Now, apply 
