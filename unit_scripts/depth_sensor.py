import time
import ms5837


class depth_sensor:
    def __init__(self, busnum: int, model=ms5837.MODEL_30BA):
        self.busnum = busnum
        self.model = model
        self._is_ready = False

    @property
    def depth(self) -> float:
        return self._depth

    @property
    def pressure(self) -> float:
        return self._pressure

    @property
    def temperature(self) -> float:
        return self._temperature

    @property
    def is_ready(self) -> bool:
        return self._is_ready

    def connect(self) -> bool:
        try:
            self.sensor = ms5837.MS5837(bus=self.busnum, model=self.model)
            self.sensor.init()
            self.sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
            self._is_ready = True
            return True
        except:
            self._is_ready = False
            return False

    def measure(self) -> bool:
        assert self._is_ready
        try:
            if not self.sensor.read():
                return False
            self._pressure = self.sensor.pressure(ms5837.UNITS_atm)
            self._depth = self.sensor.depth()
            self._temperature = self.sensor.temperature(ms5837.UNITS_Centigrade)
            return True
        except:
            return False


if __name__ == "__main__":
    busnum = 3
    sensor = depth_sensor(busnum=busnum, model=ms5837.MODEL_02BA)

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
