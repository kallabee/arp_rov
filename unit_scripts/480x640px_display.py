from status_display import *

import numpy as np
import cv2

import os
from my_lib import *








if __name__ == "__main__":
    width = 640
    height = 480
    img = np.zeros((height, width))
    img[:, width // 2 :] = 255

    disp = lcd_2dot8inch()
    disp.draw(img)
    input("input any chars to quit:")
