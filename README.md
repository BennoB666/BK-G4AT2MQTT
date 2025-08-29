# BK-G4AT2MQTT
A ESP32-C3 Super Mini Reads the MBus interface of a Honeywell BK-G4AT and transmits the data to an MQTT server

Connect the ESP32 liike the picture below.

In src-folder you have to change ssid, password, mqtt_server, mqtt_port, mqtt_topic. The MBUS_POLL_INTERVAL ist the updateinterval in Milliseconds.

I've integrated OTA. In the platformio.ini file, you can specify the ESP32's IP address under upload_port = 192.168.178.20 and update it via Wi-Fi.

<img width="600" height="800" alt="wires" src="https://github.com/user-attachments/assets/be611be3-ce91-446a-a3be-2242b5ae99b2" />

No more to do. Simple and stupid.
