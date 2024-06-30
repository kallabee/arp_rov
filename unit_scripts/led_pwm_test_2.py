"""Simple test for a standard servo on channel 0 and a continuous rotation servo on channel 1."""

from board import SCL, SDA
import busio
import time

from adafruit_pca9685 import PCA9685


class light_controller:
    def __init__(self, i2c_bus, i2c_addr, freq):
        self.pca = PCA9685(i2c_bus, address=i2c_addr)
        self.pca.frequency = freq

    def set_duty(self, ch: int, value: float):
        assert 0 <= value <= 1
        self.pca.channels[ch].duty_cycle = int(0xFFFF * value)





def main():
    i2c_bus = busio.I2C(SCL, SDA)
    i2c_addr = 0x71
    freq = 1000  # [Hz]
    lc = light_controller(i2c_bus, i2c_addr, freq)

    ld = {
        "VisFwdTop": 0,
        "VisFwdBottom": 1,
        "VisDwn": 2,
        "VisFwdTel": 3,
        "Uv": 4,
        "Ir": 5,
    }

    def turnOn(v):
        for i in range(6):
            lc.set_duty(i, v)

    # pca.channels[2].duty_cycle=0x7FFF
    # time.sleep(3)
    # pca.channels[2].duty_cycle=0

    while True:
        v = input("Input value :")
        if v == "":
            break
        try:
            v = float(v)
        except:
            continue
        print(f"v = {v}")
        # lc.set_duty(ld["Ir"], v)
        turnOn(v)


if __name__ == "__main__":
    main()
