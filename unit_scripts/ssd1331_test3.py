#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2014-18 Richard Hull and contributors
# See LICENSE.rst for details.
# PYTHON_ARGCOMPLETE_OK

"""
Display a bouncing ball animation and frames per second.

Attribution: https://github.com/rogerdahl/ssd1306/blob/master/examples/bounce.py
"""

from dataclasses import dataclass
import time
import datetime
from datetime import timezone, timedelta
import os

from PIL import Image, ImageDraw, ImageFont

# from demo_opts import get_device
from luma.oled.device import ssd1331
from luma.core.interface.serial import spi
import luma.core.render

from adafruit_blinka.microcontroller.bcm2711.pin import *
import busio

import depth_sensor
import current_sensor
import temp_pressure_sensor
from my_lib import *


class oled_64x96px_display:
    def __init__(self, device) -> None:
        self.device = device
        self.canvas = luma.core.render.canvas(self.device)
        # font = ImageFont.truetype("fonts-japanese-gothic.ttf", 16)
        # font = ImageFont.truetype("Arial.ttf", 24)
        self.font = ImageFont.truetype(
            os.path.join(os.path.dirname(__file__), "SourceHanSansJP-Medium.otf"), 8
        )

    def create_text(
        self,
        ip_addr,
        ping_result,
        temp,
        tph: temp_pressure_sensor.tph_sensor_value,
        depth: depth_sensor_value,
        power_status: current_sensor.power_status,
        elapsed_time,
    ) -> str:
        msg = ""
        # msg = "ARP01 Kankai\n"

        ct = datetime.datetime.now(timezone(timedelta(hours=+9), "JST")).time()
        # ct_str = time.strftime("%H:%M:%S", ct)
        ct_str = f"{ct:%H:%M:%S}"

        s = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
        msg += f"{ct_str}  {s}\n"

        msg += f"{ip_addr}  {ping_result}\n"

        msg += f"{temp:2.0f}  "

        if tph.valid:
            msg += f"{tph.temperature:2.0f}  {tph.pressure:4.0f}  {tph.humidity:2.0f}\n"
        else:
            msg += f"XX  XXXX  XX\n"

        if power_status.valid:
            msg += f"{power_status.voltage:4.1f}  {power_status.current:4.1f}  {power_status.power:4.1f}  {power_status.remaining_capacity_ratio*100:3.0f}\n"
        else:
            msg += f"XXXX  XXXX  XXXX  XXX\n"

        if depth.valid:
            msg += f"{depth.depth: 6.2f}  {depth.temperature:2.0f}\n"
        else:
            msg += f"XXXXXXX  XX\n"

        return msg

    def create_image(self, msg) -> Image.Image:
        img = Image.new("RGB", (96, 64), (0, 0, 0))
        img_d = ImageDraw.Draw(img)
        img_d.text((0, 0), msg, fill="white", spacing=2, align="left")

        return img

    def display(self, img: Image.Image):
        self.device.display(img)


def main():
    device = ssd1331(spi(device=0, port=0))
    sd = oled_64x96px_display(device)

    i2c = busio.I2C(D5, D4)  # I2C ch3 (SDC, SDA)
    # i2c=busio.I2C(D23, D22) #I2C ch6

    addr = 0x76
    # addr = 0x77

    sensor_type = "BMP280"

    tphsw = temp_pressure_sensor.temp_pressure_humidity_sensor_wrapper(
        i2c=i2c, addr=addr, sensor_type=sensor_type
    )
    pmw = power_monitor_wrapper(busnum=1)
    dsw = depth_sensor_wrapper(busnum=3, model=depth_sensor.ms5837.MODEL_02BA)
    start = time.perf_counter()
    st = status_logger(log_dir=r"./log")

    while True:
        ip_addr = get_ip_address("eth0")  # '192.168.0.11'
        ping_result = test_ping()
        temp = get_temp()

        tph = tphsw.measure()
        ps = pmw.measure()
        depth = dsw.measure()

        e = time.perf_counter() - start

        msg = sd.create_text(
            ip_addr=ip_addr,
            ping_result=ping_result,
            temp=temp,
            tph=tph,
            depth=depth,
            power_status=ps,
            elapsed_time=e,
        )

        img = sd.create_image(msg)
        sd.display(img)

        st.log(msg.replace("\n", " "))

        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
