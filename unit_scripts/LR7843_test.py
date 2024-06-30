import pigpio
import time

output_gpio_no = 5

pi = pigpio.pi()

pi.set_mode(output_gpio_no, pigpio.OUTPUT)
# pi.set_pull_up_down(input_gpio_no, pigpio.PUD_DOWN)

pi.write(output_gpio_no, 1)
time.sleep(5)

pi.write(output_gpio_no, 0)