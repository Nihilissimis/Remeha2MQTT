# Remeha2MQTT
Publish Remeha data through MQTT using ESP8266 board.

Building on the excellent work of @rjblake (https://github.com/rjblake/remeha) this code enables an ESP8266 (tested ESP-01) to periodically request sample data from a Remeha (Calenta) boiler, dissect to human readable (Dutch) and send as MQTT messages to a server so it can be integrated in e.g. HomeAssistant or stored for long-term analysis. 

Possibly other Remeha boilers than the Calenta may work as well, provided they use the same protocol.

Currently the code checks at configurable intervals (in this code every 10s) for changes and sends changed values only and sends full updates at longer intervals (every 5 mins) as regular interval data makes for prettier graphs. 

As the ESP-01 is typically not easily accessible during normal operation (being tucked away in the boiler), debugging information is provided over telnet using ESPtelnet, and OTA is enabled for software updates.

The hardware used is an ESP-01 with a serial breakout board. This also performs voltage-level shifting from/to 3.3V and 5V. This also enables to power the ESP-01 from the boiler itself. 
