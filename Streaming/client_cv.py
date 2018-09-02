import cv2
import numpy as np
import socket
import sys
import pickle
import struct ### new code
cap=cv2.VideoCapture(5)
clientsocket=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
clientsocket.connect(('172.24.1.97',1085))
# clientsocket.connect(('localhost',8089))
while True:
    ret,frame=cap.read()
    frame=cv2.resize(frame,(240,160))
    # print(frame)
    data = pickle.dumps(frame) ### new code

    try:
        clientsocket.sendall(struct.pack("L", len(data))+data) ### new code
    except:
        print("socket")