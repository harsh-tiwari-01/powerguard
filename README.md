# PowerGuard: Power-Cut Aware Cold Room Monitor

A power cut in a cold storage can spoil the entire stock. PowerGuard monitors
temperature and power status and warns early, even without internet.

## Features
- FreeRTOS tasks on ESP32 for temperature and power monitoring
- Reads sensor/meter data over Modbus RTU
- Publishes real-time data to a dashboard using MQTT
- On-device early warning: detects risk on the edge and raises an alert without the cloud
- Tested in Wokwi simulation

## Tech Stack
ESP32, FreeRTOS, Modbus RTU, MQTT, Embedded C/C++, PlatformIO, Wokwi

## Status
Simulation stage. Hardware testing is the next step.
