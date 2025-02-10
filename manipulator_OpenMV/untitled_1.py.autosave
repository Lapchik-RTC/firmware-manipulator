import time
import math
import sensor
import image
import pyb
from pyb import UART

# Инициализация UART
uart = UART(3)
uart.init(115200, timeout_char=1000)

def map_value(value, input_min, input_max, output_min, output_max):
    """
    Изменяет диапазон значения.

    :param value: Значение для изменения
    :param input_min: Минимальное значение входного диапазона
    :param input_max: Максимальное значение входного диапазона
    :param output_min: Минимальное значение выходного диапазона
    :param output_max: Максимальное значение выходного диапазона
    :return: Значение в новом диапазоне
    """
    if input_max == input_min:
        raise ValueError("input_min и input_max не могут быть равны.")

    # Нормализуем значение в диапазоне [0, 1]
    normalized_value = (value - input_min) / (input_max - input_min)

    # Переводим нормализованное значение в новый диапазон
    mapped_value = output_min + normalized_value * (output_max - output_min)

    return mapped_value

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

    for tag in img.find_apriltags():
        for blob in img.find_blobs([(0, 100, 19, 127, -128, 127)], area_threshold=100, pixels_threshold=100, merge=True, margin=30):
            img.draw_rectangle(blob.x(), blob.y(), blob.w(), blob.h(), (0, 0, 255))
            img.draw_rectangle(tag.rect, color=(255, 0, 0))
            img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))  # Используем без скобок

            # Печать параметров тега
            print_args = (tag.name, tag.id, (180 * tag.rotation) / math.pi)
            res = int(map_value(tag.cx, 0, 320, 0, 255))
            res = int(res/2)
            res = res*2
            zahvat = int(map_value(blob.cx(), 0, 320, 0, 255))
            zahvat = int(zahvat/2)
            zahvat = ((zahvat*2) + 1)
            print(zahvat)
            uart.writechar(res)  # Отправляем координату по UART
            uart.writechar(zahvat)  # Отправляем координату по UART
