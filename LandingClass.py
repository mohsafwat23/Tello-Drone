import cv2
import numpy as np
import cv2.aruco as aruco
import os
from djitellopy import tello
from time import *
import csv

class Landing:
    def __init__(self, markerSize= 17.78, totalMarkers = 250, draw =True):
        self.pidX = [0.15, 0.1,0]
        self.pidY = [0.4, 0.1,0]
        self.pidZ = [0.15, 0.1,0]
        self.pid_theta = [0.4, 0.4, 0]
        self.pError_theta = 0
        self.pErrorX = 0
        self.pErrorY = 0
        self.pErrorZ = 0
        self.w, self.h = 720,480#900, 600
        self.dt = 1/50
        self.markerSize = markerSize
        #self.img = cv2.resize(img,(self.w, self.h))
        self.dist_target = 100.0
        self.integralX, self.integralY, self.integralZ = 0, 0, 0
        self.cameraMatrix = np.array([
        [1.41751417e+03, 0.00000000e+00, 5.73407595e+02],
        [0.00000000e+00, 1.42339298e+03, 3.92504178e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

        self.distCoeffs = np.array([ 1.00585204e+00, -3.01089540e+01,  9.82743988e-03, -1.41835250e-02,
                2.87673404e+02])
        

        # self.cameraMatrix = np.array([
        # [743.41116567, 0.          , 479.16745299],
        # [0.          , 742.16273303, 269.83681487],
        # [0.          , 0.          , 1.          ]])
        # self.distCoeffs = np.array([ 0.24915784, -0.60878258,  0.00273825,  0.0115768,   0.52518434])

    def findArUco(self, draw =True):
        imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)#DICT_4X4_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(imgGray, aruco_dict, parameters=parameters)
        if type(ids) is np.ndarray:
            #cameraMatrix = np.array([[1070.7148775074063, 0.000000, 525.967705495989], [0.000000, 1071.0529493852516, 360.77368834556813], [0.000000, 0.000000, 1.000000]])
            #distCoeffs = np.array([-0.10919219311566612, 2.5119183355562478, -0.006027127624687258, 8.475322686476765e-05, -17.6619436608614])
            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, self.markerSize, self.cameraMatrix, self.distCoeffs)
            center = np.mean(corners[0][0], axis=0)
            self.x_centerPixel = center[0]
            self.y_centerPixel = center[1]
            self.dist = np.linalg.norm(tvecs)
            if draw:
                cv2.aruco.drawAxis(img, self.cameraMatrix, self.distCoeffs, rvecs , tvecs, 10)
        
        else:
            #if the marker is not detected return everything as 0/not detected
            self.x_centerPixel = 0
            self.y_centerPixel = 0
            self.dist = 0

    def pid_control(self):
        t_init = time()
        self.findArUco()
        errorX = self.x_centerPixel - self.w/2
        errorY = self.y_centerPixel - self.h/2
        errorZ = self.dist - self.dist_target
        #error_theta = self.theta_centerPixel - self.theta_target
        print(errorX, errorY, errorZ)
        self.integralX = self.integralX + (errorX * self.dt)
        self.integralY = self.integralY + (errorY * self.dt)
        self.integralZ = self.integralZ + (errorZ * self.dt)
        speed_lr = self.pidX[0]*errorX + self.pidX[1]*(errorX-self.pErrorX)/self.dt + self.pidX[2]*self.integralX
        speed_ud = self.pidY[0]*errorY + self.pidY[1]*(errorY-self.pErrorY)/self.dt + self.pidY[2]*self.integralY
        speed_fb = self.pidZ[0]*errorZ + self.pidZ[1]*(errorZ-self.pErrorZ)/self.dt + self.pidZ[2]*self.integralZ
        speed_fb = int(np.clip(speed_fb, -80, 80))
        speed_lr = int(np.clip(speed_lr, -80, 80))
        speed_ud = int(np.clip(speed_ud, -80, 80))
        #speed_yaw = int(np.clip(speed_yaw, -15, 15))
        if self.x_centerPixel == 0:
            speed_lr = 0
            errorX = 0
        if self.y_centerPixel == 0:
            speed_ud = 0
            errorY = 0
        if self.dist == 0:
            speed_fb = 0
            errorZ = 0
        self.pErrorX = errorX
        self.pErrorY = errorY
        self.pErrorZ = errorZ
        t_final = time()
        #drone.send_rc_control(speed_lr, speed_fb,speed_ud, 0)
        self.dt = t_final - t_init

if __name__ == "__main__":
    drone = tello.Tello() 
    drone.connect()
    print(drone.get_battery())
    drone.streamon()
    #drone.takeoff()
    #cap = cv2.VideoCapture(0)
    initLand = Landing()
    sleep(2)
    while True:
        img = drone.get_frame_read().frame
        #_, img = cap.read()
        img = cv2.resize(img,(initLand.w, initLand.h))
        cv2.imshow("Output",img)
        initLand.pid_control()
        if initLand.dist > 0 and initLand.dist < initLand.dist_target:
            drone.land()
            drone.streamoff()
            break
        if cv2.waitKey(1) & 0xFF == ord('q'):
            drone.land()
            drone.streamoff()
            break
        #sleep(0.001)