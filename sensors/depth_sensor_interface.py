from sensors.depth_sensor import DepthSensor
from multiprocessing import Value

class DepthSensorInterface:
    def __init__(self, z, running):
        self.z = z
        self.running = running
        self.depth_sensor = DepthSensor()

    def update(self):
        depth = self.depth_sensor.recieve_data()
        if depth != None and len(depth) > 3:
            if"Depth" in depth:
                try:
                    self.z.value = float(depth[depth.find(" ") + 1:])
                    #print(self.z.value)
                except:
                    pass

    def print_data(self):
        print(self.z.value)

    def run_loop(self):
        while self.running.value:
            #print(self.running.value)
            self.update()

if __name__ == "__main__":
    depth_z = Value('d', 0.0)
    running = Value('b', True)
    depth_sensor_interface = DepthSensorInterface(z=depth_z, running=running)
    depth_sensor_interface.run_loop()
