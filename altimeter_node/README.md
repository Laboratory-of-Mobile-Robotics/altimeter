# Altimeter ROS node

Once the sensor data is already been read and sent through the serial communication, in this directory (/altimeter_node) there are the scripts to read the temperature and pressure income data, estimmate the current altitude over the sea and publish them to ROS topics.

P.S.: Find out the serial port used and you can change the code.
port = /dev/ttyUSB0