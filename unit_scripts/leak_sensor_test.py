import RPi.GPIO as GPIO
import time

input_gpio_no = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(input_gpio_no, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

while True:
    data = GPIO.input(input_gpio_no)
    print(f"{data}")
    time.sleep(1)