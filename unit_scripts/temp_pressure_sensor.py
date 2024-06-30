import board
import time
from adafruit_bme280 import basic as adafruit_bme280  # This library is deprecated...
from adafruit_blinka.microcontroller.bcm2711.pin import *
import adafruit_bmp280
import busio
from dataclasses import dataclass

@dataclass
class tph_sensor_value:
    valid: bool
    temperature: float
    pressure: float
    humidity: float

class temp_pressure_humidity_sensor:
    def __init__(self, i2c, addr:int, sensor_type:str):
        assert sensor_type=="BMP280" or sensor_type=="BME280"
        self.i2c = i2c
        self.addr = addr
        self.sensor_type=sensor_type
        self.is_ready=False
    
    @property
    def temperature(self)->float:
        return self._temperature
    
    @property
    def pressure(self)->float:
        return self._pressure
    
    @property
    def humidity(self)->float:
        # assert self.sensor_type=="BME280"
        return self._humidity

    def connect(self)->bool:
        try:
            if self.sensor_type == "BMP280":
                self.sensor = adafruit_bmp280.Adafruit_BMP280_I2C(self.i2c, address=self.addr)
                self._humidity=0
            else:
                self.sensor = adafruit_bme280.Adafruit_BME280_I2C(self.i2c, address=self.addr)        
                self.sensor.sea_level_pressure = 1013.25
            self.is_ready=True
            return True  
        except:
            self.is_ready=False
            return False
    
    def measure(self)->tph_sensor_value:
        succeeded = self._measure()

        tph = tph_sensor_value(succeeded, self.temperature, self.pressure, self.humidity)
        return tph

    def _measure(self)->bool:
        # assert self.is_ready, 'This device is not ready.'
        try:
            self._temperature=self.sensor.temperature
            self._pressure=self.sensor.pressure
            if self.sensor_type == 'BME280':
                self._humidity=self.sensor.relative_humidity
            self.is_ready=True
            return True
        except:
            # Set dummy values.
            self._temperature=-1000
            self._pressure=-100000
            self._humidity=-100         
            self.is_ready=False
            return False

class temp_pressure_humidity_sensor_wrapper:
    """
    Wrapper class for removing care when connection is cut and exception occurs.
    """
    def __init__(self, i2c, addr:int, sensor_type:str) -> None:
        self.tphs = temp_pressure_humidity_sensor(i2c=i2c, addr=addr, sensor_type=sensor_type)
        self.tphs.connect()

    def measure(self)->tph_sensor_value:
        if not self.tphs.is_ready:
            self.tphs.connect()        
        tph = self.tphs.measure()

        return tph


if __name__ == '__main__':
    # i2c = board.I2C()   # uses board.SCL and board.SDA
    i2c = busio.I2C(D5, D4)  # I2C ch3 (SDC, SDA)
    # i2c=busio.I2C(D23, D22) #I2C ch6

    addr = 0x76
    # addr = 0x77

    # sensor_type = 'BMP280'
    sensor_type = 'BME280'

    sensor = temp_pressure_humidity_sensor(sensor_type=sensor_type,i2c=i2c, addr=addr)

    while True:
        if sensor.connect():
            while True:
                result = sensor.measure()
                if not result.valid:
                    print('Failed to measure.')
                    break
                print(f"{sensor.temperature:4.1f} [deg. C] {sensor.pressure:6.1f} [hPa] {sensor.humidity:5.1f} [%]")

        else:
            print("Failed to connect.")
        
        time.sleep(1)