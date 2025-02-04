# Untitled - By: User - Sat Jan 18 2025

import sensor
import time
import image
import pyb


# pyb.
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
try:
    # The camera will now focus on whatever is in front of it.
    # sensor.ioctl(sensor.IOCTL_TRIGGER_AUTO_FOCUS)
    # sensor.ioctl(sensor.IOCTL_RESET_AUTO_FOCUS)
    # sensor.ioctl(sensor.IOCTL_SET_NIGHT_MODE, True)
    print('kek')
    # sensor
    # a[0] += 1
except:
    # print(e)
    raise (Exception("Auto focus is not supported by your sensor/board combination."))

sensor.skip_frames(time=2000)

clock = time.clock()


while True:
    # try:
    #     print('1')
    #     a[0] += 1
    #     print('2')
    # except Exception as e:
    #     print(e)
    clock.tick()
    img = sensor.snapshot()
    for blob in img.find_blobs([(0, 100, 7, 127, 4, 127)], area_threshold=300, pixels_threshold=250,  merge = True, margane = 30):
        img.draw_cross(blob.cx(), blob.cy(), (0, 181, 18))
        img.draw_rectangle(blob.x(), blob.y(), blob.w(), blob.h(), (0, 0, 255))
    print(image.TAG36H11)

