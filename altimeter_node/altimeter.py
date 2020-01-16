import serial
import time

class Altimeter:
    def __init__(self,relative=False,temperature=15.0,pressure=1013.25,altitude=0.0):
        self.raw_msg = ""
        self.relative = relative
        self.temperature = temperature
        self.pressure = pressure
        self.altitude_sensor = altitude
        self.altitude = altitude
        self.baseline = 1013.25
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)

    def parse_serial(self,delimiter=","):
        data = []
        aux = ""
        for c in self.raw_msg:
            if(c != delimiter):
                aux += c
            elif(c == delimiter):
                data.append(float(aux))
                aux = ""
        data.append(float(aux))
        try:
            self.temperature, self.pressure, self.altitude_sensor = data
        except:
            pass

    def read_sensor(self):
        self.raw_msg = self.ser.readline().decode()[:-2]
        self.parse_serial()

    def calculate_baseline(self):
        # print("Collecting baseline values for {:d} seconds. Do not move the sensor!\n".format(baseline_size))
        baseline_values = []
        baseline_size = 100
        for i in range(baseline_size):
            self.readline()
            baseline_values.append(self.pressure)
            time.sleep(1)
        self.baseline = sum(baseline_values[:-25]) / len(baseline_values[:-25])

    def estimate_altitude(self):
        # Remembering the sensor gives pressure in the SI units of Pascals, but the altitude formula requires mbar units, so the
        # measured pressure is divided by 100 to convert it.
        self.altitude = ((pow(self.baseline / (self.pressure / 100), (1.0 / 5.257)) - 1.0) * (self.temperature + 273.15)) / 0.0065
        
    def get_data(self):
        return [self.temperature, self.pressure, self.altitude]
    
    def get_temperature(self):
        return self.temperature
    
    def get_pressure(self):
        return self.pressure
    
    def get_altitude(self):
        return self.altitude


# EXAMPLE PLAY

# altimeter = Altimeter()
# while(True):
#     try:
#         altimeter.read_sensor()
#         # ipdb.set_trace()
#         altimeter.estimate_altitude()
#         print(altimeter.get_altitude())
#     except:
#         pass
