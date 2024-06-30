import ina226
import time
from dataclasses import dataclass


@dataclass
class power_status:
    valid: bool
    voltage: float
    current: float
    power: float
    accumulated_power: float
    accumulated_time: float
    remaining_capacity_ratio: float


class power_monitor:
    def __init__(self, busnum=3, addr=0x40):
        self.busnum = busnum
        self.addr = addr
        self.connected = False
        self.initialized = False

    @property
    def current(self) -> float:
        return self._current

    @property
    def voltage(self) -> float:
        return self._voltage

    @property
    def power(self) -> float:
        return self._power

    @property
    def accumulated_power(self) -> float:
        """

        Returns:
            float: [Wh]
        """
        return self._accumulated_power

    @property
    def remaining_capacity_ratio(self) -> float:
        return (self.capacity - self._accumulated_power) / self.capacity

    @property
    def accumulated_time(self):
        """

        Returns:
            _type_: [s]
        """
        return self._accumulated_time

    @property
    def is_ready(self):
        return self.connected and self.initialized

    def connect(self) -> bool:
        try:
            self.sensor = ina226.INA226(
                busnum=self.busnum,
                address=self.addr,
                shunt_ohms=0.002,
                max_expected_amps=25,
            )
            self.sensor.configure()
            self.connected = True
            return True
        except:
            self.connected = False
            return False

    def reset_measurement(self, capacity: float) -> bool:
        """
        Params:
            capacity: [Wh]
        """
        assert self.connected

        self.capacity = capacity
        self._accumulated_power = 0
        self._accumulated_time = 0
        self.initialized = True

    def measure(self) -> power_status:      
        self._measure()
        ps = power_status(
            self.succeeded,
            self.voltage,
            self.current,
            self.power,
            self.accumulated_power,
            self.accumulated_time,
            self.remaining_capacity_ratio,
        )
        return ps

    def _measure(self) -> bool:
        assert self.is_ready
        try:
            self._current = self.sensor.current() / 1000
            self._power = self.sensor.power() / 1000
            self._voltage = self.sensor.voltage()
        except:
            self.succeeded = False
            return False

        now = time.perf_counter()
        try:
            diff = now - self.last_update_time  # [s]
        except:
            # For the first call.
            self.last_update_time = now
            return self._measure()

        self._accumulated_power += (
            diff / (60 * 60)
        ) * self._power  # Use trapezoidal method to integrate.
        self._accumulated_time += diff
        self.last_update_time = now
        self.succeeded = True

        return self.succeeded


if __name__ == "__main__":
    pm = power_monitor()
    while True:
        if pm.connect():
            pm.reset_measurement(7.2 * 2 * 5)
            while True:
                success = pm.measure()
                if not success:
                    break
                print(
                    f"{pm.voltage:4.1f} [V] {pm.current:+6.3f} [A] {pm.power:6.3f} [W] {pm.accumulated_power:6.3f} [Wh] {100*pm.remaining_capacity_ratio:3.0f} [%] {pm.accumulated_time/60:6.1f} [min]"
                )
                time.sleep(1)
