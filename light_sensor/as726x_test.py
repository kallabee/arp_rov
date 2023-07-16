# SPDX-FileCopyrightText: 2020 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import busio
from adafruit_blinka.microcontroller.bcm2711.pin import *
from adafruit_blinka.microcontroller.bcm2711.pin import *
# for I2C use:
from adafruit_as726x import AS726x_I2C

# for UART use:
# from adafruit_as726x import AS726x_UART

# maximum value for sensor reading
max_val = 16000

# max number of characters in each graph
max_graph = 80


def graph_map(x):
    return min(int(x * max_graph / max_val), max_graph)

i2c=busio.I2C(D5, D4) #i2c3 (Ch B)
# i2c=busio.I2C(D23, D22) #i2c6 (Ch C)

sensor = AS726x_I2C(i2c)

sensor.conversion_mode = sensor.MODE_2

while True:
    # Wait for data to be ready
    while not sensor.data_ready:
        time.sleep(0.1)

    # plot plot the data
    print("\n")
    print("V: " + graph_map(sensor.violet) * "=")
    print("B: " + graph_map(sensor.blue) * "=")
    print("G: " + graph_map(sensor.green) * "=")
    print("Y: " + graph_map(sensor.yellow) * "=")
    print("O: " + graph_map(sensor.orange) * "=")
    print("R: " + graph_map(sensor.red) * "=")

    time.sleep(1)