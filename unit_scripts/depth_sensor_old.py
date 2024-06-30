import python.ms5837_renamed as ms5837_renamed
import time


class depth_sensor:
    def __init__(self, busnum: int, model=ms5837.MODEL_02BA):
        self.busnum = busnum
        self.type=type
        self.is_ready = False

    @property
    def depth(self) -> float:
        return self._depth

    @property
    def pressure(self) -> float:
        return self._pressure

    @property
    def temperature(self) -> float:
        return self._temperature

    def connect(self) -> bool:
        try:
            self.sensor = ms5837_renamed.MS5837_30BA(self.busnum)
            self.sensor.init()
            self.sensor.setFluidDensity(ms5837_renamed.DENSITY_SALTWATER)
            self.is_ready = True
            return True
        except:
            self.is_ready = False
            return False

    def measure(self) -> bool:
        assert self.is_ready
        try:
            if not self.sensor.read():
                return False
            self._pressure = self.sensor.pressure(ms5837_renamed.UNITS_atm)
            self._depth = self.sensor.depth()
            if self.type=='02BA':
                self._pressure /=15
                self._depth /=15
            self._temperature = self.sensor.temperature(ms5837_renamed.UNITS_Centigrade)
            return True
        except:
            return False


if __name__ == "__main__":
    busnum = 3
    sensor = depth_sensor(busnum=busnum, type="02BA")

    while True:
        if sensor.connect():
            while True:
                success = sensor.measure()
                if not success:
                    print("Failed to measure.")
                    break
                print(
                    f"{sensor.depth:6.3f} [m] {sensor.pressure:3.1f} [atm] {sensor.temperature:4.1f} [deg. C]"
                )
                time.sleep(1)
        else:
            print("Failed to connect.")
