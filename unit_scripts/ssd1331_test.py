﻿#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) 2014-18 Richard Hull and contributors
# See LICENSE.rst for details.
# PYTHON_ARGCOMPLETE_OK

"""
Display a bouncing ball animation and frames per second.

Attribution: https://github.com/rogerdahl/ssd1306/blob/master/examples/bounce.py
"""


import sys
import random
from PIL import Image, ImageDraw, ImageFont
import os
# from demo_opts import get_device
from luma.oled.device import ssd1331
from luma.core.interface.serial import spi
import luma.core.render
import time
import datetime
import socket
import fcntl
import struct
import subprocess


def get_temp() -> float:
    with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
        tmp = f.read()
        return int(tmp) / 1000



def get_ip_address(interface: str) -> str:
    """
    Uses the Linux SIOCGIFADDR ioctl to find the IP address associated
    with a network interface, given the name of that interface, e.g.
    "eth0". Only works on GNU/Linux distributions.
    Source: https://bit.ly/3dROGBN
    Returns:
        The IP address in quad-dotted notation of four decimal integers.
    """

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    packed_iface = struct.pack("256s", interface.encode("utf_8"))
    packed_addr = fcntl.ioctl(sock.fileno(), 0x8915, packed_iface)[20:24]
    return socket.inet_ntoa(packed_addr)


def exec_ping(dest):
    p = subprocess.run(["ping", dest, '-c', '1'], capture_output=True)
    return p.returncode == 0


def test_ping():
    target_ip = "192.168.0.5"
    if exec_ping(target_ip):
        return "OK"
    else:
        return "NG"


def main():
    canvas = luma.core.render.canvas(device)
    # font = ImageFont.truetype("fonts-japanese-gothic.ttf", 16)
    # font = ImageFont.truetype("Arial.ttf", 24)
    font = ImageFont.truetype(os.path.join(os.path.dirname(__file__),"SourceHanSansJP-Medium.otf"), 8)

    start = time.perf_counter()

    while True:
        with canvas as c:
            c.rectangle(device.bounding_box, outline="black", fill="black")

            ipaddr = get_ip_address("eth0")  # '192.168.0.110'
            ping_result = test_ping()

            e = time.perf_counter() - start
            s = time.strftime("%H:%M:%S", time.gmtime(e))
            # c.text((2, 0), f"ARP01 Kankai", font=font, fill="white")
            # # c.text((2, 30), f"{e:%H:%M:%S}", fill="white")
            # c.text((2, 20), f"{e}", font=font, fill="white")
            # # c.text((2, 30), e.strftime("%H:%M:%S"), fill="white")
            # c.text((2, 40), f"{get_temp():4.1f}", font=font, fill="white")

            msg = ''
            # msg = "ARP01 Kankai\n"
            msg += f"{s}\n"
            msg += f"{get_temp():4.1f}\n"
            msg += f"{ipaddr} {ping_result}\n"
            # msg += f"{}\n"
            print(msg)
            c.text((0, 0), msg, fill="white", spacing=2, align="left")
        time.sleep(1)


if __name__ == "__main__":
    try:
        # device = get_device()
        device = ssd1331(spi(device=0, port=0))
        main()
    except KeyboardInterrupt:
        pass
