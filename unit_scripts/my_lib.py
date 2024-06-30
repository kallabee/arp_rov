import os
from dataclasses import dataclass
import logging
from logging import handlers
import socket
import struct
import fcntl
import struct
import subprocess

import tkinter
from PIL import Image, ImageTk
import numpy as np
import cv2

import depth_sensor
import current_sensor
import temp_pressure_sensor


def show_full_screen(img: np.array) -> None:
    pilImage = Image.fromarray(img)
    showPIL(pilImage)


def showPIL(pilImage):
    root = tkinter.Tk()
    w, h = root.winfo_screenwidth(), root.winfo_screenheight()
    root.overrideredirect(1)
    root.geometry("%dx%d+0+0" % (w, h))
    root.focus_set()
    root.bind("<Escape>", lambda e: (e.widget.withdraw(), e.widget.quit()))
    canvas = tkinter.Canvas(root, width=w, height=h)
    canvas.pack()
    canvas.configure(background="black")
    imgWidth, imgHeight = pilImage.size
    if imgWidth > w or imgHeight > h:
        ratio = min(w / imgWidth, h / imgHeight)
        imgWidth = int(imgWidth * ratio)
        imgHeight = int(imgHeight * ratio)
        pilImage = pilImage.resize((imgWidth, imgHeight), Image.ANTIALIAS)
    image = ImageTk.PhotoImage(pilImage)
    imagesprite = canvas.create_image(w / 2, h / 2, image=image)
    root.mainloop()


def prepare_display_env(val=r":0.0"):
    display = "DISPLAY"
    if display in os.environ and os.environ[display] != "":
        print(
            f"env variable {display} has already been set to '{os.environ[display]}'."
        )
    else:
        os.environ[display] = val
        print(f"'{os.environ[display]}' has been set into env variable {display}.")


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
    p = subprocess.run(["ping", dest, "-c", "1"], capture_output=True)
    return p.returncode == 0


def test_ping(addr_list):
    msg = ''
    for addr in addr_list:
        okng = 'OK' if exec_ping(addr) else 'NG'
        l = f"{addr} {okng}\n"
        msg += l
    return msg



@dataclass
class depth_sensor_value:
    valid: bool
    depth: float
    temperature: float


class status_logger:
    def __init__(self, log_dir: str) -> None:
        # formatter = logging.Formatter(
        #     "%(asctime)s - %(levelname)s:%(name)s - %(message)s"
        # )
        formatter = logging.Formatter("%(message)s")

        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        self.logger.propagate = False

        os.makedirs(log_dir, exist_ok=True)

        rh = logging.handlers.TimedRotatingFileHandler(
            os.path.join(log_dir, r"status_log.txt"),
            encoding="utf-8",
            when="H",
            interval=1,
            backupCount=0,
        )
        rh.setFormatter(formatter)
        self.logger.addHandler(rh)

        stream_handler = logging.StreamHandler()
        stream_handler.setFormatter(formatter)
        self.logger.addHandler(stream_handler)

    def log(self, msg: str):
        self.logger.info(msg)


class depth_sensor_wrapper:
    def __init__(self, busnum=3, model=depth_sensor.ms5837.MS5837_30BA) -> None:
        self.ds = depth_sensor.depth_sensor(busnum, model)

    def measure(self) -> depth_sensor_value:
        if self.ds.is_ready:
            if self.ds.measure():
                depth = depth_sensor_value(True, self.ds.depth, self.ds.temperature)
            else:
                depth = depth_sensor_value(False, 0, 0)
        else:
            depth = depth_sensor_value(False, 0, 0)
            self.ds.connect()

        return depth


class power_monitor_wrapper:
    def __init__(self, busnum=3, addr=0x40) -> None:
        self.pm = current_sensor.power_monitor(busnum=busnum, addr=addr)

    def measure(self) -> current_sensor.power_status:
        if self.pm.is_ready:
            ps = self.pm.measure()
            return ps
            # if not ps.succeeded:
            #     break
            # print(f"{pm.voltage:4.1f} [V] {pm.current:+6.3f} [A] {pm.power:6.3f} [W] {pm.accumulated_power:6.3f} [Wh] {100*pm.remaining_capacity_ratio:3.0f} [%] {pm.accumulated_time/60:6.1f} [min]")
            # time.sleep(1)
        else:
            if self.pm.connect():
                self.pm.reset_measurement(7.2 * 2 * 5)
            return current_sensor.power_status(False, 0, 0, 0, 0, 0, 0)
