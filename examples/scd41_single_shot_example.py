# SPDX-FileCopyrightText: 2021 by Keith Murray, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import adafruit_scd4x

i2c = board.I2C()
scd4x = adafruit_scd4x.SCD4X(i2c)
print("Serial number:", [hex(i) for i in scd4x.serial_number])

scd4x.measure_single_shot()
print("Waiting for single show CO2 measurement from SCD41....")

sample_counter = 0
while sample_counter < 3:
    if scd4x.data_ready:
        print("CO2: %d ppm" % scd4x.CO2)
        print("Temperature: %0.1f *C" % scd4x.temperature)
        print("Humidity: %0.1f %%" % scd4x.relative_humidity)
        print()
        sample_counter += 1
    else:
        print("Waiting...")
    time.sleep(1)


scd4x.measure_single_shot_rht_only()
print("Waiting for single show Humidity and Temperature measurement from SCD41....")

sample_counter = 0
while sample_counter < 3:
    if scd4x.data_ready:
        print("CO2: %d ppm" % scd4x.CO2) # Should be 0 ppm
        print("Temperature: %0.1f *C" % scd4x.temperature)
        print("Humidity: %0.1f %%" % scd4x.relative_humidity)
        print()
        sample_counter += 1
    else:
        print("Waiting...")
    time.sleep(1)