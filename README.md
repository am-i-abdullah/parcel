# parcel-cart

## Intialize the project
1. http://arduino.esp8266.com/stable/package_esp8266com_index.json
2. https://dl.espressif.com/dl/package_esp32_index.json
3. esp32 by espressif
3. Board as AI Thinker ESP32-CAM
4. mqtt by joel gaehwiler

## Setup MQTT server
sudo apt update
sudo apt install mosquitto mosquitto-clients
sudo systemctl start mosquitto
sudo systemctl status mosquitto
sudo systemctl stop mosquitto
sudo nano /etc/mosquitto/mosquitto.conf
listener 1883
allow_anonymous true
sudo systemctl restart mosquitto
mosquitto_sub -h localhost -t "test/topic"
mosquitto_pub -h localhost -t "test/topic" -m "Hello MQTT"

conda create -n parcel-cart python=3.10