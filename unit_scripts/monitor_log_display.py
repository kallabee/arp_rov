from dataclasses import dataclass
import time
import datetime
from datetime import timezone, timedelta
import os
from typing import Tuple
from PIL import Image, ImageDraw, ImageFont


from adafruit_blinka.microcontroller.bcm2711.pin import *
import busio

import depth_sensor
import current_sensor
import temp_pressure_sensor
from my_lib import *
import status_display


class lcd_2dot8inch(status_display.status_display):
    def __init__(self):
        prepare_display_env()
        self.window_title = "window"
        cv2.namedWindow(self.window_title, cv2.WND_PROP_FULLSCREEN)
        cv2.setWindowProperty(self.window_title, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)


    # def draw(self, img: np.array):
    #     show_full_screen(img)

    # def draw(self, img: Image.Image):
    #     # showPIL(img)
    def draw(self, img: np.array):
        cv2.imshow(self.window_title, img)
        cv2.waitKey(1)

    def create_image_from_status(self,
        ip_addr:str,
        ping_result:str,
        temp:float,
        tph: temp_pressure_sensor.tph_sensor_value,
        depth: depth_sensor_value,
        power_status: current_sensor.power_status,
        elapsed_time,
    ) ->Tuple[ Image.Image, str]:
        h = 480
        w = 640
        c = 3
        # img = np.zeros((h,w, c))

        font_size = 20
        x_pos = 10 # [px]
        y_pos = x_pos  # [px]
        y_interval = int(font_size*1.5)  # [px]

        img = Image.new("RGB", (w, h), (0, 0, 0))
        img_d = ImageDraw.Draw(img)

        total_msg=''

        font = ImageFont.truetype(
            os.path.join(os.path.dirname(__file__), "SourceHanSansJP-Medium.otf"), font_size
        )

        msg = "ARP-01 Kankai"
        total_msg += msg+ ', '
        img_d.text((x_pos, y_pos), msg,font=font, fill="white", spacing=2, align="left")
        y_pos = y_pos + y_interval

        ct = datetime.datetime.now(timezone(timedelta(hours=+9), "JST")).time()
        ct_str = f"{ct:%H:%M:%S}"
        s = time.strftime("%H:%M:%S", time.gmtime(elapsed_time))
        msg = f"System time : {ct_str}, Time from script started : {s}\n"
        total_msg += f"{ct_str}, {s}, "
        img_d.text((x_pos, y_pos), msg, font=font, fill="white", spacing=2, align="left")
        y_pos = y_pos + y_interval

        msg = f"LAN IP : {ip_addr}, Ping Result : \n{ping_result}\n"
        total_msg += f"{ip_addr}\n {ping_result}, "
        img_d.text((x_pos, y_pos), msg, font=font, fill="white", spacing=2, align="left")
        y_pos = y_pos + 3*y_interval

        msg = f"CPU Temp : {temp:2.0f} [deg.C]"
        total_msg += f"{temp:2.0f}, "
        img_d.text((x_pos, y_pos), msg, font=font, fill="white", spacing=2, align="left")
        y_pos = y_pos + y_interval

        if tph.valid:
            msg = f"Hull Temp : {tph.temperature:2.0f} [deg.C], Press : {tph.pressure:4.0f} [hPa], Humid : {tph.humidity:2.0f} [%]"
            total_msg += f"{tph.temperature:2.0f}, {tph.pressure:4.0f}, {tph.humidity:2.0f}, "
        else:
            msg = f"XX  XXXX  XX\n"
            total_msg += f"XX,  XXXX,  XX, "
        img_d.text((x_pos, y_pos), msg, font=font, fill="white", spacing=2, align="left")
        y_pos = y_pos + y_interval

        if power_status.valid:
            volt = f"{power_status.voltage:4.1f}"
            current = f"{power_status.current:4.1f}"
            power = f"{power_status.power:4.1f}"
            rcr = f"{power_status.remaining_capacity_ratio*100:3.0f}"
        else:
            volt = f"XXXX"
            current = f"XXXX"
            power = f"XXXX"
            rcr = f"XXX"

        msg = f"Volt : {volt} [V], Current : {current} [A], \n    Power : {power} [W], Rem Cap Ratio : {rcr} [%]"
        total_msg += f"{volt}, {current}, {power}, {rcr}, "
        img_d.text((x_pos, y_pos), msg, font=font,fill="white", spacing=2, align="left")
        y_pos = y_pos +2* y_interval

        if depth.valid:
            dd = f"{depth.depth: 6.2f}"
            dt = f"{depth.temperature:2.0f}"
        else:
            dd = "XXXXXXX"
            dt = "XX"
        msg = f"Depth : {dd} [m], Water Temp : {dt} [deg.C]"
        total_msg += f"{dd}, {dt}"
        img_d.text((x_pos, y_pos), msg, font=font,fill="white", spacing=2, align="left")
        y_pos = y_pos + y_interval

        return img, total_msg


def main():
    display = lcd_2dot8inch()

    # i2c = busio.I2C(D5, D4)  # I2C ch3 (SDC, SDA), GPIO5 & 4
    i2c=busio.I2C(D23, D22) #I2C ch6, GPIO23, 22

    addr = 0x76
    # addr = 0x77

    # sensor_type = "BMP280"
    sensor_type = "BME280"

    tphsw = temp_pressure_sensor.temp_pressure_humidity_sensor_wrapper(
        i2c=i2c, addr=addr, sensor_type=sensor_type
    )
    pmw = power_monitor_wrapper(busnum=3)
    dsw = depth_sensor_wrapper(busnum=3, model=depth_sensor.ms5837.MODEL_02BA)
    start = time.perf_counter()
    st = status_logger(log_dir=r"./log")

    while True:
        ip_addr = get_ip_address("eth0")  # '192.168.0.11'

        host_ip_addr_list = ['192.168.0.5', '192.168.0.20']
        ping_result = test_ping(host_ip_addr_list)

        temp = get_temp()

        tph = tphsw.measure()
        ps = pmw.measure()
        depth = dsw.measure()

        e = time.perf_counter() - start

        img ,msg= display.create_image_from_status(
            ip_addr=ip_addr,
            ping_result=ping_result,
            temp=temp,
            tph=tph,
            depth=depth,
            power_status=ps,
            elapsed_time=e,
        )

        img = np.array(img)
        display.draw(img)

        st.log(msg)

        time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
