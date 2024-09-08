import INA228
import time

ina228 = INA228.INA228(address = 0x40, shunt_ohms = 0.015)

ina228.configure()

i = 0

while True:    

    ina228.get_vbus_voltage()

    ina228.get_current()

    ina228.get_power()

    if i < 1000:

        i = i +1
        print(i, time.ticks_ms())

    else:

        break
