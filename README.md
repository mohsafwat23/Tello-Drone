# Landing a Tello Drone on a moving platform
Video: https://youtu.be/Vv7nIcAhIz4

## Setup
* An AruCo Marker is printed and attached to a square platform
* A 3D printed mirror holder and a mirror are attatched to the drone
* Laptop Camera and Drone Camera are calibrated

### AruCo Marker
The printed marker has to be reflected about its y-axis because of the attatched mirror.
The image file is `Assets/Images/reflectedMarker.png`

### Mirror Holder
The 3D printed mirror holder is `Assets/Parts/Tello_Mirror_Clip_2.STL`

### Camera Calibration
Use the following Repo: https://github.com/smidm/video2calibration 
Use `recorder.py` to record the video required for calibration or use the Tello mobile app. 

## Installation
Required libraries are OpenCV for the visual detection algorithm and the Tellopy for controlling the drones
Install requirements:

`pip3 install requirements.txt`

## Running the Code on your Laptop Camera
*For testing first(without the drone)*

`python3 laptopAruco.py`

## Running the Code on the Drone

`python3 arucoDetector.py`

