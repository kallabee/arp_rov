# https://stackoverflow.com/questions/47316266/can-i-display-image-in-full-screen-mode-with-pil

import time
from my_lib import *


def main():
    width = 640
    height = 480

    img = np.zeros((height, width))
    img[:, width // 2 :] = 255
    val = 30

    prepare_env()
    while True:
        val += 50
        val = val % 255
        img[:,:] = val
        pilImage = Image.fromarray(img)
        showPIL(pilImage)
        time.sleep(1)

    input("input to quit")


if __name__ == "__main__":
    main()
