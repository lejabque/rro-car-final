# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
import threading
from time import sleep
import serial
import time

DISPLAY = False
SERIAL = True
LIGHT = True
LINE = True

# SERIAL CONNECTION, поиск порта
if SERIAL:
    port_connected = False
    while not port_connected:
        for i in range(0, 10):
            port = '/dev/ttyACM' + str(i)
            # noinspection PyBroadException
            try:
                ser = serial.Serial(port, 9600, write_timeout=1)  # serial setup
                print(port)
                port_connected = port
                break
            except Exception:
                print('N', port)


# отправка в Serial-порт
def ser_send(msg):
    # noinspection PyBroadException
    try:
        ser.write(str.encode(msg))
    except Exception:
        pass


# ========== LINE ==========
# black line detector
def line_detect(dir_line):
    global img
    img_line = cv.resize(img, (240, 320))
    h, w, _ = img_line.shape
    roi_height = h // 16
    see_above_y = h
    # see_above_y = h
    see_below_y = see_above_y - roi_height
    roi_width_l = w // 2
    roi_width_r = w // 2
    roi_center = (roi_width_l + roi_width_r) // 2 + roi_width_l
    img_center = w // 2
    roi = img_line[see_below_y:see_above_y, img_center - roi_width_l:img_center + roi_width_r]

    h, w, _ = roi.shape
    # print(roi.shape)
    gray_line = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)  # grayscale
    blur = cv.GaussianBlur(gray_line, (9, 9), 2, 2)  # blur
    _, th = cv.threshold(blur, 192, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)  # threshold
    kernel = np.ones((6, 6), np.uint8)
    erosion = cv.erode(th, kernel, iterations=1)  # erode
    dilate = cv.dilate(erosion, kernel, iterations=1)  # dilate

    _, contours, _ = cv.findContours(dilate, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)  # contour find
    centers = {}  # dict 'center_x':'delta'
    line_found = False  # line
    for cnt in contours:
        if (cv.contourArea(cnt) > 100) and (cv.contourArea(cnt) < 1500):
            line_found = True
            # print(cv.contourArea(cnt))
            x, y, w, h = cv.boundingRect(cnt)
            m = cv.moments(cnt)  # contour moments
            # noinspection PyBroadException
            try:
                cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])
                # print(cy, cx)
                delta = img_center - (cx + img_center - roi_width_l)
                centers[cx] = delta
                print('Delta =', delta, 'px')
            except Exception:
                print('error')
    if not line_found:
        return -10000

    delta_all = []  # deltas, from left to right
    for k in sorted(centers.keys()):
        delta_all.append(centers[k])

    if len(delta_all) > 1:  # fork
        ser_send('T')
        print('TTTTTTTTTTTTTTTTTTTTTTTTTTTT')

    if dir_line == 'L':
        print('LEFT')
        return delta_all[0]
    elif dir_line == 'F':
        print('FORWARD')
        return delta_all[(len(delta_all)-1)//2]
    elif dir_line == 'R':
        print('RIGHT')
        return delta_all[-1]
    return delta_all[(len(delta_all)-1)//2]


# ========= TRAFFIC LIGHT ==========
def find_light():
    global img
    img_light = cv.blur(img, (10, 10))
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    green_light_f = 0
    red_light_f = 0

    # green mask
    lower_green = np.array([35, 60, 110], dtype="uint8")
    upper_green = np.array([91, 255, 255], dtype="uint8")
    green_mask = cv.inRange(img_hsv, lower_green, upper_green)

    # red-orange mask
    lower_red = np.array([0, 85, 110], dtype="uint8")
    upper_red = np.array([3, 255, 255], dtype="uint8")

    # red-violet mask
    lower_violet = np.array([165, 85, 110], dtype="uint8")
    upper_violet = np.array([180, 255, 255], dtype="uint8")

    red_mask_orange = cv.inRange(img_hsv, lower_red, upper_red)  # color mask
    red_mask_violet = cv.inRange(img_hsv, lower_violet, upper_violet)

    red_mask_full = red_mask_orange + red_mask_violet  # mask sum

    _, contours, _ = cv.findContours(green_mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)  # find mask cnt
    for cnt in contours:
        if (cv.contourArea(cnt) > 100) and (cv.contourArea(cnt) < 3000):
            # x, y, w, h = cv.boundingRect(cnt)
            cv.drawContours(img_light, [cnt], -1, (0, 255, 0), 3)
            green_light_f += 1

    _, contours, _ = cv.findContours(red_mask_full, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)  # find mask cnt
    for cnt in contours:
        if (cv.contourArea(cnt) > 100) and (cv.contourArea(cnt) < 3000):
            # x, y, w, h = cv.boundingRect(i)
            cv.drawContours(img_light, [cnt], -1, (0, 0, 255), 3)
            red_light_f += 1
    return red_light_f, green_light_f


if __name__ == "__main__":
    cam_found = False
    cap = 0
    # find camera
    for i in range(0, 10):
        if not cam_found:
            # noinspection PyBroadException
            try:
                # cap.release()
                cap = cv.VideoCapture(i)
                _, img = cap.read()
                print(img.shape)
                cam_found = True
            except Exception:
                print('not found', i)
    if not cam_found:
        print('Cam not found')
        exit(0)

    h_src, w_src, _ = img.shape
    last_delta = 0  # last line delta value
    i = 0
    sleep(1)
    print('qqqqqqqqqqqqqqqqqq')
    ser_send('q')


    while True:
        start = time.time()
        _, img = cap.read()
        dir_r = 'R'
        last_delta = 0
        if LINE:
            # dir_r = 'F'
            # noinspection PyBroadException
            try:
                delta_line = line_detect(dir_r)
            except Exception:
                delta_line = last_delta

            if delta_line == -10000:
                delta_line = last_delta

            last_delta = delta_line
            delta_str = 'l' + str(delta_line)
            ser_send(delta_str)

        i += 1


    cap.release()
    cv.destroyAllWindows()
    exit(0)
