import cv2
import numpy as np
import math
from djitellopy import tello
import time

# Define the aruco dictionary 
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)

#initialze drone object
me = tello.Tello()
me.connect()

#if drone detects aruco object it will land
def land():
    me.land()
    time.sleep(1)