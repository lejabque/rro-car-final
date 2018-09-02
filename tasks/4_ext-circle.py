# -*- coding: utf-8 -*-
import cv2 as cv
import numpy as np
import threading
from time import sleep
import serial
import time

DISPLAY = False
SERIAL = True
SIGN = True
LIGHT = False
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


# ========== SIGNS ==========
# CASCADES
if SIGN:
    ped_cascade = cv.CascadeClassifier('sign/haar/ped_cascade.xml')
    stop_cascade = cv.CascadeClassifier('sign/haar/stop_cascade.xml')
    right_cascade = cv.CascadeClassifier('sign/lbp/right_lbp.xml')
    left_cascade = cv.CascadeClassifier('sign/lbp/left_lbp.xml')
    # forward_cascade = cv.CascadeClassifier('sign/lbp/forward_lbp.xml')


def signs_detect():  # функция распознавания знаков, через каскады
    while True:
        global sign
        global img
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        stops = stop_cascade.detectMultiScale(gray, 1.3, 5)
        peds = ped_cascade.detectMultiScale(gray, 1.3, 5)
        right_turns = right_cascade.detectMultiScale(gray, 1.3, 5)
        left_turns = left_cascade.detectMultiScale(gray, 1.3, 5)
        # forward_go = forward_cascade.detectMultiScale(gray, 1.3, 5)

        s_max = 0
        sign = 'F'

        for (x, y, w, h) in stops:
            print(w*h)
            # cv.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
            if (w * h > s_max) and (w*h > 7000):
                s_max = w * h
                sign = 'S'
        for (x, y, w, h) in peds:
            print(w*h)
            # cv.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)
            if (w * h > s_max) and (w*h > 7000):
                s_max = w * h
                sign = 'P'

        for (x, y, w, h) in right_turns:
            # cv.rectangle(img, (x, y), (x+w, y+h), (0, 255, 0), 2)
            if w * h > s_max:
                s_max = w * h
                sign = 'R'
        for (x, y, w, h) in left_turns:
            # cv.rectangle(img, (x, y), (x+w, y+h), (0, 0, 255), 2)
            if w * h > s_max:
                s_max = w * h
                sign = 'L'
        # print('SSSSSSSSSSSSSSSSSSIGN')
        if DISPLAY:
            cv.imshow('img', img)
            if cv.waitKey(1) & 0xFF == ord('q'):
                cv.destroyAllWindows()
        # for (x, y, w, h) in forward_go:
        #     if w * h > S_max:
        #         S_max = w * h
        #         sign = 'F'
        # if sign != 'F':
        print(sign, s_max)
        ser_send(sign)
        if (sign == 'L') or (sign == 'R'):
            sleep(3)
        # return sign


# ========== LINE ==========
# black line detector
def line_detect(dir_line):
    global img
    img_line = cv.resize(img, (240, 320))
    h, w, _ = img_line.shape
    roi_height = h // 16
    see_above_y = h - h // 10
    # see_above_y = h
    see_below_y = see_above_y - roi_height

    roi_width_l = w // 2
    roi_width_r = w // 2
    roi_center = w // 2
    img_center = w // 2
    roi = img_line[see_below_y:see_above_y, roi_center - roi_width_l:roi_center + roi_width_r]

    h, w, _ = roi.shape
    # print(roi.shape)
    gray_line = cv.cvtColor(roi, cv.COLOR_BGR2GRAY)  # grayscale
    blur = cv.GaussianBlur(gray_line, (9, 9), 2, 2)  # blur
    _, th = cv.threshold(blur, 192, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)  # threshold
    kernel = np.ones((6, 6), np.uint8)
    erosion = cv.erode(th, kernel, iterations=1)  # erode
    dilate = cv.dilate(erosion, kernel, iterations=1)  # dilate

    # if DISPLAY:
    #     res = img.copy()
    #     cv.rectangle(res, (roi_center - roi_width, see_below_y),
    #                  (roi_center + roi_width, see_above_y), (0, 0, 255), 2)  # roi rectangle
    #     cv.circle(res, (roi_center, see_below_y), 5, (0, 255, 255), -1)

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
                # if DISPLAY:
                #     res = img.copy()
                #     cv.rectangle(res, (x + roi_center - roi_width, y + see_below_y),
                #                  (x + w + roi_center - roi_width, y + h + see_below_y), (0, 255, 0), 2)
                #     cv.circle(res, (cx + roi_center - roi_width, cy + see_below_y), 5, (255, 0, 0), -1)
            except Exception:
                print('error')
    if not line_found:
        return -10000
    # if DISPLAY:
    #     res = img.copy()
    #     cv.namedWindow('img', cv.WINDOW_NORMAL)
    #     cv.resizeWindow('img', h_src * 3, w_src * 3)
    #     cv.imshow('img', res)
    #     if cv.waitKey(1) & 0xFF == ord('q'):
    #         cap.release()
    #         cv.destroyAllWindows()
    #         exit(0)

    delta_all = []  # deltas, from left to right
    for k in sorted(centers.keys()):
        delta_all.append(centers[k])

    if len(delta_all) > 1:  # fork
        ser_send('T')
        print('TTTTTTTTTTTTTTTTTTTTTTTTTTTT')
    # if len(delta_all) >= 2:
    #    return 0

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
    h, w, _ = img.shape
    img = img[:, w // 2:, :]
    img_light = cv.blur(img, (10, 10))
    img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    green_light_f = 0
    red_light_f = 0

    # green mask
    lower_green = np.array([45, 134, 100], dtype="uint8")
    upper_green = np.array([90, 255, 255], dtype="uint8")
    green_mask = cv.inRange(img_hsv, lower_green, upper_green)

    # red-orange mask
    lower_red = np.array([130, 135, 255], dtype="uint8")
    upper_red = np.array([255, 255, 255], dtype="uint8")

    # red-violet mask
    # lower_violet = np.array([165, 85, 110], dtype="uint8")
    # upper_violet = np.array([180, 255, 255], dtype="uint8")

    red_mask_orange = cv.inRange(img_hsv, lower_red, upper_red)  # color mask
    # red_mask_violet = cv.inRange(img_hsv, lower_violet, upper_violet)
    red_mask_full = red_mask_orange
    # red_mask_full = red_mask_orange + red_mask_violet  # mask sum

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
    # cap.set(3, 320)
    # cap.set(4, 240)
    last_delta = 0  # last line delta value
    i = 0
    sleep(1)
    f_red = False
    while not f_red:
        _, img = cap.read()
        red, green = find_light()
        print(red, green)
        if red > 0:
            f_red = True
        elif green > 0:
            f_red = True
            start = True

    start = False
    while not start:
        _, img = cap.read()
        red, green = find_light()
        print(red, green)
        if red == 0:
            start = True
        # if green != 0:
        #     start = True
        # elif green+red == 0:
        #     start = True
    print('qqqqqqqqqqqqqqqqqq')
    ser_send('q')

    sign = 'F'
    if SIGN:
        t1 = threading.Thread(target=signs_detect)  # sign detector thread
        t1.start()
        # t1.join()

    while True:
        start = time.time()
        _, img = cap.read()
        # print(find_light())
        sign_found = sign
        #  pprint(sign_found)
        # if sign != 'F':
        #   ser_send(sign_found)
        if sign_found == 'R':
            dir_r = 'R'
        elif sign_found == 'L':
            dir_r = 'L'
        elif sign_found == 'F':
            dir_r = 'F'
        dir_r = 'R'
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

        if LIGHT and (i % 10 == 0):
            red_light, green_light = find_light()
            # print(red_light, green_light)
            if DISPLAY:
                cv.imshow('img', img)
                if cv.waitKey(1) & 0xFF == ord('q'):
                    break

        i += 1
        if i == 120:
            end = time.time()
            seconds = end - start
            fps = 1 / seconds
            print("Estimated frames per second : ", fps)
            i = 0


    cap.release()
    cv.destroyAllWindows()
    exit(0)
