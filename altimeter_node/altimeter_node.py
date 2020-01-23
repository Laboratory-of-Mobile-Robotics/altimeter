import rospy
from std_msgs.msg import String
from sensor_msgs.msg import FluidPressure, Temperature
from geometry_msgs.msg import PointStamped 
from altimeter import Altimeter
import csv
from datetime import datetime
import argparse
# import ipdb  # library for debugging

# From BMP280 datasheet
SENSOR_STDEV = 240 # (default 120 Pascal). Approximately 10 meters.
ALTITUDE_STDEV = 1e-1

parser = argparse.ArgumentParser()
parser.add_argument("--log", "-l", help="True to log topics data values int CSV file; False to not log the data. Default = False.", default=False)
parser.add_argument("--rate", "-r", help="Number to define the rate of data sampling in Hertz. Default = 2 Hz.", default=2)
parser.add_argument("--temperature", "-t", help="True to consider real temperature measurement on altitude estimation; False to not measure the temperature, i.e., consider 15 *C. Default = True", default=True)
parser.add_argument("--logfile", "-f", help="Name the log file. Default = log_data_<current date>.csv")
args = parser.parse_args()

class BMP280Node:
    def __init__(self):
        self.altitudePub = rospy.Publisher('altitude/data', PointStamped, queue_size=1)
        self.pressurePub = rospy.Publisher('pressure/data', FluidPressure, queue_size=1)
        self.temperaturePub = rospy.Publisher('temperature/data', Temperature, queue_size=1)
        self.filename = ""
        self.log = args.log
        self.rate = float(args.rate)
        self.use_temperature = args.temperature
        if(self.log):
            if(not args.logfile):
                self.filename = "log_data_" + datetime.now().strftime("%Y%m%d") + ".csv"
            else:
                self.filename = str(args.logfile) + ".csv"

            with open(self.filename, mode='w') as file:
                fields = ['temperature','pressure','altitude','timestamp']
                writer = csv.writer(file, delimiter=',')
                writer.writerow(fields)

    def pubAltitude(self):
        msg = PointStamped()
        msg.header.stamp = rospy.Time.now()
        msg.point.z = altimeterObj.get_altitude()
        # rospy.loginfo(msg)
        self.altitudePub.publish(msg)
        return

    def pubPressure(self):
        msg = FluidPressure()
        msg.header.stamp = rospy.Time.now()
        msg.fluid_pressure = altimeterObj.get_pressure()
        msg.variance = SENSOR_STDEV **2
        # rospy.loginfo(msg)
        self.pressurePub.publish(msg)
        return

    def pubTemperature(self):
        msg = Temperature()
        msg.header.stamp = rospy.Time.now()
        if(self.use_temperature):
            msg.temperature = altimeterObj.get_temperature() + 273.15
        else:
            msg.temperature = 15 + 273,15
        msg.variance = 0
        # rospy.loginfo(msg)
        self.temperaturePub.publish(msg)
        return

    def log_data(self):
        temperature = altimeterObj.get_temperature()
        pressure = altimeterObj.get_pressure()
        altitude = altimeterObj.get_altitude()
        # now = rospy.Time.now()
        now = datetime.now().strftime("%H:%M:%S")
        with open(self.filename,mode='a') as file:
            line = [temperature,pressure,altitude,now]
            writer = csv.writer(file,delimiter=',')
            writer.writerow(line)
        print(line)

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            altimeterObj.read_sensor()
            altimeterObj.estimate_altitude(self.use_temperature)
            self.pubAltitude()
            self.pubPressure()
            self.pubTemperature()
            if(self.log):
                self.log_data()
            rate.sleep()
        return


if __name__ == '__main__':
    try:
        altimeterObj = Altimeter()
        rospy.init_node('altimeter', anonymous=True)
        BMP280Node().spin()
    except rospy.ROSInterruptException:
        pass