# Altimeter

This project contains an [ESP32 driver](./barometer) to read pressure and temperature from a BMP280 sensor and a [ROS driver](./altimeter_node) estimate the current altitude related to sea level using the sensor's data.

## Materials
* [ESP32 Development KIT](https://docs.zerynth.com/latest/official/board.zerynth.doit_esp32/docs/index.html);
* [BMP280 I2C Pressure and Temperature Sensor](https://cdn-learn.adafruit.com/downloads/pdf/adafruit-bmp280-barometric-pressure-plus-temperature-sensor-breakout.pdf);
* a good computer. ( * ▽ * )=b

## Usage
1. Connect the BMP280 sensor to ESP32 correctly;
2. Connect the ESP32 to the computer (USB) and upload the barometer driver available at /barometer/barometer.ino;
3. Initiate ROS by typing the commands:
```
$ source /opt/ros/kinetic/setup.bash
$ roscore &
```
4. Activate the virtualenv in which the required python libraries are already installed:
```
$ source /venv/bin/activate
```
5. Run the altimeter node script at /altimeter_node/altimeter_node.py:
```
$ python altimeter_node/altimeter_node.py
```

## About the topics published
With the sensor and Altimeter node up, the pressure, temperature and altitude estimation are published on topics:
* /pressure/data
    - type: sensor_msgs.msg.FluidPressure
* /temperature/data
    - type: sensor_msgs.msg.Temperature
* /altitude/data
    - type: geometry_msgs.msg.PointStamped
    - considering it only provides the altitude estimation, on PointStamped point data only the 'z' is filled (PointStamped.data.z)