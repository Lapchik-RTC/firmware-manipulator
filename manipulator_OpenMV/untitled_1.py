import time
import math
import sensor
import image
import pyb
import array
from pyb import UART

# Инициализация UART
uart = UART(3)
uart.init(115200, timeout_char=1000)
# bytearray
def crc8(pc_block, length):
    crc = 0xFF  # Начальное значение CRC
    for byte in pc_block[1:length]:  # Проходим по каждому байту в блоке
        crc ^= byte  # Выполняем XOR с текущим байтом
        for _ in range(8):  # Для каждого бита в байте
            if crc & 0x80:  # Если старший бит установлен
                crc = (crc << 1) ^ 0x31  # Сдвиг влево и XOR с полином
            else:
                crc <<= 1  # Просто сдвиг влево
    return crc & 0xFF  # Возвращаем CRC, ограниченный одним байтом


# Настройка сенсора
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)  # Отключаем автоматическое усиление
sensor.set_auto_whitebal(False)  # Отключаем автоматическую балансировку белого
clock = time.clock()

while True:
    clock.tick()
    img = sensor.snapshot()
    length = 11
    res_array = array.array('B', [0] * length)
    res_array[0] = 0xFF


    for tag in img.find_apriltags():
        print_args = (tag.name, tag.id, (180 * tag.rotation) / math.pi)
        res = tag.cx
        value_y_tag = tag.cy
        value_lo_tag = res & 0xFF
        value_hi_tag = (res >> 8) & 0xFF
        res_array[1] = value_lo_tag;
        res_array[2] = value_hi_tag;
        res_array[3] = value_y_tag;
        img.draw_rectangle(tag.rect, color=(255, 0, 255))

        img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))  # Используем без скобок

    for blob in img.find_blobs([(10, 90, 36, 97, -14, 127)], area_threshold=200, pixels_threshold=300, merge=True, margin=30):
        img.draw_rectangle(blob.x(), blob.y(), blob.w(), blob.h(), (0, 0, 255))
        zahvat = blob.cx()
        value_y_zahvat = blob.cy()
        value_lo_zahvat = zahvat & 0xFF
        value_hi_zahvat = (zahvat >> 8) & 0xFF
        res_array[4] = value_lo_zahvat;
        res_array[5] = value_hi_zahvat;
        res_array[6] = value_y_zahvat;

    for blob1 in img.find_blobs([(10, 95, -126, -30, -1, 127)], area_threshold=100, pixels_threshold=30, merge=True, margin=40):
        img.draw_rectangle(blob1.x(), blob1.y(), blob1.w(), blob1.h(), (255, 255, 0))

        pole = blob.cx()
        value_y_pole = blob.cy()
        value_lo_pole = pole & 0xFF
        value_hi_pole = (pole >> 8) & 0xFF
        res_array[7] = value_lo_pole;
        res_array[8] = value_hi_pole;
        res_array[9] = value_y_pole;
    res_array[10] = crc8(res_array, 10   )
    # res_array[11] = crc8(res_array[])
    uart.write(res_array)
    print(res_array)
