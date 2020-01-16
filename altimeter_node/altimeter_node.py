import rospy
from std_msgs.msg import String
from sensor_msgs.msg import FluidPressure, Temperature
from geometry_msgs.msg import PointStamped 
from altimeter import Altimeter

# From BMP280 datasheet
SENSOR_STDEV = 240 # (default 120 Pascal). Approximately 10 meters.
ALTITUDE_STDEV = 1e-1

class BMP280Node:
    def __init__(self):
        self.altitudePub = rospy.Publisher('altitude/data', PointStamped, queue_size=1)
        self.pressurePub = rospy.Publisher('pressure/data', FluidPressure, queue_size=1)
        self.temperaturePub = rospy.Publisher('temperature/data', Temperature, queue_size=1)

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
        msg.temperature = altimeterObj.get_temperature() + 273.15
        msg.variance = 0
        # rospy.loginfo(msg)
        self.temperaturePub.publish(msg)
        return

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            altimeterObj.read_sensor()
            altimeterObj.estimate_altitude()
            self.pubAltitude()
            self.pubPressure()
            self.pubTemperature()
        return


def altimeter_publisher():
    pub = rospy.Publisher('altitude', String, queue_size=10)
    rospy.init_node('altimeter', anonymous=False)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        altimeterObj.read_sensor()
        altimeterObj.estimate_altitude()
        msg = str(altimeterObj.get_altitude())
        # msg = "870.56"
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        altimeterObj = Altimeter()
        # altimeter_publisher()
        rospy.init_node('altimeter', anonymous=True)
        BMP280Node().spin()
    except rospy.ROSInterruptException:
        pass