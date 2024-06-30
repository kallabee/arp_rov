import numpy as np
import cv2

import os

width = 640
height = 480

img = np.zeros((height, width))
img[:, width // 2 :] = 255



    


prepare_env()

window_title = "window"

cv2.namedWindow(window_title, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(window_title, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
# cv2.setWindowProperty(window_title, cv2.WINDOW_FULLSCREEN, cv2.WND_PROP_FULLSCREEN)
# cv2.setWindowProperty(window_title, cv2.WND_PROP_FULLSCREEN | cv2.WND_PROP_TOPMOST, cv2.WINDOW_FULLSCREEN)

cv2.imshow(window_title, img)
cv2.waitKey(1)

cv2.imshow(window_title, img)
cv2.waitKey(1)

input("input to quit")
