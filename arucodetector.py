import cv2
import numpy as np
import cv2.aruco as aruco
import os
from djitellopy import tello
from time import *
import csv

me = tello.Tello()#beforeze drone 
me.connect()#connect to the drone
print(me.get_battery())
me.streamon()
me.takeoff()
me.send_rc_control(0, 0, 25, 0) #go up at a speed of 25 cm/s for 1.5 seconds
sleep(1.5)



pidX = [0.15, 0.1,0]
pidY = [0.15, 0.4,0.2]
pid_theta = [0.4, 0.4, 0]
pError_theta = 0
pErrorX = 0
pErrorY = 0
#w, h = 540, 340
w, h = 960, 720


def findArUco(img, markerSize= 4, totalMarkers = 250, draw =True):
    """ Detecting ArUco markers in the image and returning the 
    center of the marker relative to the image center, the area of the marker
    & the top corners of the marker in the y-direction."""

    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters_create()
    #corners: A list containing the (x, y)-coordinates of our detected ArUco markers
    #ids: A list containing the ids of our detected ArUco markers
    #rejectedImgPoints: A list containing the rejected points of our detected ArUco markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(imgGray, aruco_dict, parameters=parameters)
    if type(ids) is np.ndarray:
    #if ids != None:
        #cameraMatrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
        #distCoeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])
        cameraMatrix = np.array([[4.20160861e+03, 0.000000, 6.82522572e+02], [0.000000, 4.37676882e+03, 3.72848172e+02], [0.000000, 0.000000, 1.000000]])
        distCoeffs = np.array([3.55373130e-01, -2.29822225e+01, 5.45797079e-03, -4.05893858e-03, -1.13433864e+00])
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs)
        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
        top_left_y = corners[0][0][0][1]
        top_right_y = corners[0][0][1][1]
        top_right_x = corners[0][0][1][0]
        bottom_right_x = corners[0][0][2][0]
        area = (top_right_y - top_left_y) * (top_right_x - bottom_right_x)
        #area = abs((corners[0][0][0][0] - corners[0][0][1][0])*(corners[0][0][0][1] - corners[0][0][1][1]))
        x_centerPixel = x_sum*.25
        y_centerPixel = y_sum*.25
        if draw:
            cv2.aruco.drawAxis(img, cameraMatrix, distCoeffs, rvecs , tvecs, 10)

        return x_centerPixel,y_centerPixel,top_left_y,top_right_y,area
        
    else:
        #if the marker is not detected return everything as 0/not detected
        return 0,0,0,0,0

def trackMarker(me, info, pErrorY, pErrorX, pError_theta, speed_dn):
    """ This function is used to track the marker and return the velocity error terms of the drone."""

    #pidY = [0.45, 0.3]
    x,y,top_left_y,top_right_y,area = info
    error_theta = top_right_y - top_left_y
    errorY = y - h//2
    errorX = x - w//2
    integralX = integral[0] + (errorX * dt)
    integralY = integral[1] + (errorY * dt)
    speed_lr = pidX[0]*errorX + pidX[1]*(errorX-pErrorX)+ pidX[2]*integralX
    speed_fb = pidY[0]*errorY + pidY[1]*(errorY-pErrorY)+pidY[2]*integralY
    speed_yaw = pid_theta[0]*error_theta + pid_theta[1]*(error_theta-pError_theta)
    #print(speed_fb)
    speed_fb = int(np.clip(speed_fb, -60, 60))
    speed_lr = int(np.clip(speed_lr, -10, 10))
    speed_yaw = int(np.clip(speed_yaw, -15, 15))
    if y == 0:
    #if -10 <= y <= 10:
        speed_fb = 0
        errorY = 0
    if x == 0:
    #if -3 <= x <= 3:
         speed_lr = 0
         errorX = 0
    if top_left_y == top_right_y:
        speed_yaw = 0
        error_theta = 0
    #print(errorX, errorY)
    me.send_rc_control(speed_lr, speed_fb, speed_dn, speed_yaw)# add speed l_r
    return errorY,errorX,error_theta,speed_lr,speed_fb,area
#cap = cv2.VideoCapture(0)

t_before = time()
dt = 0
integral = [0,0]
f = open("ErrorInY.csv", "a+")
writer = csv.writer(f, delimiter=',')
while True:
    t_initial = time()-t_before
    img = me.get_frame_read().frame
    #_, img = cap.read()
    img=cv2.resize(img,(w,h))
    info = findArUco(img)
    if int(t_final) > 10:
        #if the specfied time has passed, then the drone will slowly descend
        pErrorY,pErrorX,pError_theta,speed_lr,speed_fb,area = trackMarker(me,info, pErrorY, pErrorX, pError_theta,speed_dn = -1)
        #print(area)
        if area > 350:
            #if the area of the marker relative to the camera frame is greater than 350, then the drone will land
            me.land()
    else:
        pErrorY,pErrorX,pError_theta,speed_lr,speed_fb,area = trackMarker(me,info, pErrorY, pErrorX, pError_theta, speed_dn = 0)
    t_final = time()-t_before
    data = [t_final,pErrorY,0]
    dt = t_final-t_initial 
    print(1/dt) #frequency
    cv2.imshow("Output",img)
    writer.writerow(data)
    f.flush()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        print(me.get_battery())
        me.streamoff()
        break
     
