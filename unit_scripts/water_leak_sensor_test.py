import pigpio
import time

input_gpio_no = 24

pi = pigpio.pi()

pi.set_mode(input_gpio_no, pigpio.INPUT)
pi.set_pull_up_down(input_gpio_no, pigpio.PUD_DOWN)

while True:
    data = pi.read(input_gpio_no)
    print(f"{data}")
    time.sleep(1)
