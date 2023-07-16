import accelerometer

device = accelerometer.Accelerometer()

print(device.readAccData())

for i in range(100):
    x, y, z = device.readAccData()
    print(x, y, z)






