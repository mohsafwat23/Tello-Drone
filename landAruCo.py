import cv2
import numpy as np
import cv2.aruco as aruco
import os
from djitellopy import tello
from time import sleep

#me = tello.Tello()#initialze drone 
#me.connect()#connect to the drone
#print(me.get_battery())
#me.streamon()
#me.takeoff()
#me.send_rc_control(0, 0, 10, 0)
#sleep(0.5)

pid = [0.15, 0.05,0]
pErrorX = 0
pErrorY = 0
w, h = 540, 340


def findArUco(img, markerSize= 4, totalMarkers = 250, draw =True):
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)
    parameters = cv2.aruco.DetectorParameters_create()
    #corners: A list containing the (x, y)-coordinates of our detected ArUco markers
    #ids: A list containing the ids of our detected ArUco markers
    #rejectedImgPoints: A list containing the rejected points of our detected ArUco markers
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(imgGray, aruco_dict, parameters=parameters)
    if type(ids) is np.ndarray:
    #if ids != None:
        cameraMatrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
        distCoeffs = np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])
        rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs)
        #_centerY = int((corners[0][1] + corners[2][1]) / 2)
        #_centerX = int((corners[0][0] + corners[2][0]) / 2)
        #x_center = (corners[0][0][0][0]+corners[0][0][1][0])/2
        #y_center = (corners[0][0][0][1]+corners[0][0][0][1])/2
        #print(corners[0])
        x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
        y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
        #print(tvecs)    
        x_centerPixel = x_sum*.25
        y_centerPixel = y_sum*.25
        #print(x_centerPixel, y_centerPixel)
        # x_corner_1 = corners[0][0][0][0]
        # x_corner_2 = corners[0][0][1][0]
        # x_corner_2 = corners[0][0][1][0]
        # x_corner_3 = corners[0][0][2][0]
        # x_corner_4 = corners[0][0][3][0]
        # print(x_sum, y_sum)
        #cv2.aruco.drawDetectedMarkers(img, corners, ids)
        if draw:
            cv2.aruco.drawAxis(img, cameraMatrix, distCoeffs, rvecs , tvecs, 10)
        #x_pos = tvecs[0]
        #y_pos = tvecs[1]
        #print(tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2])
        #cv2.circle(img,(x_centerPixel,y_centerPixel),5,(0,255,0),cv2.FILLED)
        #return tvecs[0][0][0], tvecs[0][0][1], tvecs[0][0][2]
        return x_centerPixel,y_centerPixel
        #area = abs((corners[0][0][0][0] - corners[0][0][1][0])*(corners[0][0][0][1] - corners[0][0][1][1]))
    else:
        return 0,0
    # if draw:
    #      cv2.aruco.drawDetectedMarkers(img, corners, ids)
    # return x_centerPixel, y_centerPixel
def trackMarker(info,h, pid, pErrorY, pErrorX):
    x,y = info

    errorY = y - h//2
    errorX = x - w//2
    speed_fb = pid[0]*errorY + pid[1]*(errorY-pErrorY)
    speed_lr = pid[0]*errorX + pid[1]*(errorX-pErrorX)
    speed_fb = int(np.clip(speed_fb, -50, 50))
    speed_lr = int(np.clip(speed_lr, -30, 30))
    if y == 0:
        speed_fb = 0
        errorY = 0
    if x == 0:
        speed_lr = 0
        errorX = 0
    print(speed_fb, speed_lr)
    #me.send_rc_control(speed_lr, speed_fb, 0, 0)
    return errorY, errorX
cap = cv2.VideoCapture(0)

while True:
    #img = me.get_frame_read().frame
    _, img = cap.read()
    img=cv2.resize(img,(w,h))
    info = findArUco(img)
    #pErrorY, pErrorX = trackMarker(me,info,h, pid, pErrorY, pErrorX)
    pErrorY, pErrorX = trackMarker(info,h, pid, pErrorY, pErrorX)
    #pError = trackMarker(info,h, pid, pError)

    cv2.imshow("Output",img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        me.land()
        me.streamoff()
        break
     
