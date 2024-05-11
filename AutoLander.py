import sys
import socket
from dronekit import Command, connect, VehicleMode, mavutil
import time
import numpy as np
import serial
import cv2

droneLink = '127.0.0.1:14550'
camPortName = '/dev/ttyS2'  #TODO verify serial connection and orangepi-config
camId = 0
cameraRotation = 0

# we know the altitude???
altitude = 20   #TODO set altitude to loiter alt
fov = 160
pfov = 120


repeat = True

class Coordinate:
    lat: float
    lon: float
    alt: float

def main():
    #setup camera
    camPort = open_camPort()
    #connect tot drone
    vehicle = connectToDrone()
    #download mission
    cmds = getMission(vehicle)
    #wait for mission end
    waitForMissionEnd(vehicle, cmds)
    #find landing target
    lzCoord = getTargetPosition(vehicle, camPort)
    #modify and execute mission for landing
    executeLanding(vehicle, cmds, lzCoord)
    #close everything
    #TODO

def connectToDrone():
    connected = False
    while not connected:
        try:
            vehicle = connect(droneLink, wait_ready=True)
            connected = True
        except:
            #wait before trying to connect again
            time.sleep(10)
    return vehicle

def getMission(vehicle):
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    return cmds

def waitForMissionEnd(vehicle, cmds):
    readyToLand = False
    while(not readyToLand):
        #loiter at mission end
        if vehicle.mode.name == VehicleMode('LOITER'):
            #ready to land when num of mission items = next
            if cmds.next == cmds.count-1:
                readyToLand = True

def getTargetPosition(vehicle, camPort):
    #TODO 1. get drone gps coordinate
    droneCoord = vehicle.location.global_relative_frame

    #TODO 2. get image
    #TODO 3. find target
    #TODO 3.1 find blue
    #TODO 3.2 find target area
    #TODO 3.3 find target center
    #TODO 4 calculate target gps coord

    #[48.51079433248238, -71.64953214980738, 0]      # Alma
    #lzCoord = vehicle.home_location  # not observable??? check with takeoff mission item??
    #TODO verify Alma coordinates
    lzCoord = Coordinate
    lzCoord.lat = 48.51079433248238
    lzCoord.lon = -71.64953214980738
    lzCoord.alt = 15
    return lzCoord

def executeLanding(vehicle, cmds, lzCoord):
    landCmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
                               0, mavutil.mavlink.PRECISION_LAND_MODE_DISABLED, 0, 0, lzCoord.lat, lzCoord.lon, lzCoord.alt)

    #TODO check if its better to
    # 1. add a waypoint at the end and continue mission
    # 1.1 make sure that MIS_RESTART is properly set to CONTINUE mission from start
    # 2. clear and create new mission with one point : land
    # 2.1 make sure that MIS_RESTART is properly set to RESTART mission from start
    cmds.clear()
    cmds.add(landCmd)
    cmds.upload()

    #continue mission with landing nav point
    vehicle.mode = VehicleMode("AUTO")


def get_image():
    img = None
    global camId
    camOpen = False
    cap = None
    while not camOpen:
        if camId > 10:
            camId = 0
        try:
            cap = cv2.VideoCapture(camId)
            camOpen = True
            # waitTime = datetime.datetime.now() + datetime.timedelta(seconds=10)
            # pause.until(waitTime)
        except:
            print("Unable to open camera with camId == " + str(camId))
            camId += 1

    while(True):
        ret, frame = cap.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if cv2.countNonZero(gray) > 1:
            img = frame
            print("Got picture")
            break
        else:
            print("Could not take picture of the ground")

    if cap is not None:
        cap.release()

    if cameraRotation != 0:
        cv2.imshow("non-rotated", img)
        cv2.waitKey(2000)
        img = cv2.rotate(img, cameraRotation)
        cv2.imshow("rotated", img)
        cv2.waitKey(2000)

    return img

def find_target(img, largest_contour):
    # Get the center coordinates of the blue blob
    M = cv2.moments(largest_contour)
    if M['m00']:
        target_x = int(M['m10'] / M['m00'])
        target_y = int(M['m01'] / M['m00'])
        return target_x, target_y
    else:
        return 0, 0

def find_blue(img):
    # Define range of blue color in RGB
    #lower_blue = np.array([[np.array([100, 200, 150], dtype=np.uint8)]])
    #upper_blue = np.array([[np.array([220, 255, 255], dtype=np.uint8)]])
    lower_blue = np.array([30,  100, 200])
    upper_blue = np.array([180, 255, 255])

    # Convert to HSV
    #lower_blue = cv2.cvtColor(lower_blue, cv2.COLOR_RGB2HSV)
    #upper_blue = cv2.cvtColor(upper_blue, cv2.COLOR_RGB2HSV)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    #cv2.imshow('mask', mask)

    # Find contours in the binary image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Get the largest contour, which should be the blue blob
    if len(contours):
        largest_contour = max(contours, key=cv2.contourArea)
        return largest_contour, mask
    else:
        print("Could not find blue color in image")
        return [], None
def find_image_center(img):
    x = img.shape[1] // 2
    y = img.shape[0] // 2

    print(f"Center coordinates: ({x}, {y})")

    return x, y

def open_camPort():
    camConnected = False
    port = None
    while not camConnected:
        try:
            port = serial.Serial(camPortName, baudrate=57600, timeout=1.0)
            camConnected = True
        except:
            time.sleep(5)
    return port


if __name__ == "__main__":
    main()