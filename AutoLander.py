from dronekit import Command, connect, VehicleMode, mavutil
import time
import numpy as np
import serial
import cv2
from geographiclib.geodesic import Geodesic as geo

droneLink = '127.0.0.1:14550'   #TODO verify mavlink-router udp port used
camPortName = '/dev/ttyS2'      #TODO verify serial connection and orangepi-config
camId = 0
maxRetries = 100
# we know the altitude???
altitude = 15   #TODO set altitude to loiter alt

debugPrint = True

class Coordinate:
    lat: float
    lon: float
    alt: float

def main():
    if debugPrint:
        print("Starting AutoLander")

    #setup camera
    if debugPrint:
        print("Connecting to camera")
    camPort = open_camPort()

    #connect totdrone
    if debugPrint:
        print("Connecting to drone")
    vehicle = connectToDrone()

    #download mission
    if debugPrint:
        print("Getting mission from drone")
    cmds = getMission(vehicle)

    #wait for mission end
    if debugPrint:
        print("Waiting for mission end")
    waitForMissionEnd(vehicle, cmds)

    #find landing target
    if debugPrint:
        print("Starting lz detection")
    lzCoord = getTargetPosition(vehicle)

    #modify and execute mission for landing
    if debugPrint:
        print("Sending nav point to drone")
    executeLanding(vehicle, cmds, lzCoord)

    #close everything
    if debugPrint:
        print("Program finished")
    camPort.close()

def connectToDrone():
    connected = False
    vehicle = None
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

    #wait for mission start
    while vehicle.mode.name != VehicleMode('AUTO'):
        if debugPrint:
            print("Waiting for mission start")
            time.sleep(5)

    if debugPrint:
        print("Waiting for mission end")
    while(not readyToLand):
        #loiter at mission end
        #TODO verify end of mission mode
        if vehicle.mode.name == VehicleMode('LOITER'):
            #ready to land when num of mission items = next
            if cmds.next == cmds.count-1:
                readyToLand = True

def getTargetPosition(vehicle):
    #
    for i in range(0, maxRetries):
        #1. get drone gps coordinate
        droneCoord = vehicle.location.global_relative_frame
        #2. get image
        img = get_image()

        #3. find target
        #3.1 find blue
        contour, mask = find_blue(img)
        #3.2 find target area
        target_x, target_y = find_target(img, contour)
        if [target_x, target_y] != [0, 0]:
            break
        time.sleep(1)

    #if 0,0 no target land there
    if [target_x, target_y] == [0, 0]:
        center_x = 0
        center_y = 0
    else:
        #3.3 find target center
        center_x, center_y = find_image_center(img)
    #4 calculate target gps coord
    x, y = distance(target_x, target_y, center_x, center_y)

    # [48.51079433248238, -71.64953214980738, 0]      # Alma
    # lzCoord = vehicle.home_location  # not observable??? check with takeoff mission item??
    # TODO verify Alma coordinates
    #lzCoord = Coordinate
    #lzCoord.lat = 48.51079433248238
    #lzCoord.lon = -71.64953214980738
    #lzCoord.alt = 15

    lzCoord = move_gps(x, y, droneCoord)
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

def distance(target_x, target_y, center_x, center_y):

    pixel_size_metre = 0.000729394*altitude + 0.0000000013

    x = (target_x - center_x) * pixel_size_metre
    y = (center_y - target_y) * pixel_size_metre

    print("Moving by: " + str(x) + "m east; " + str(y) + "m north")

    return x, y

def move_gps(x, y, droneCoord):
    # https://gis.stackexchange.com/a/412001
    geod = geo.WGS84
    theta = np.arctan2(y, x)
    azimuth = theta - 90
    dist = np.sqrt(x**2 + y**2)

    g = geod.Direct(droneCoord.lat, droneCoord.lon, azimuth, dist)
    targetCoord = Coordinate
    targetCoord.lat = g["lat2"]
    targetCoord.lon = g["lon2"]
    targetCoord.alt = 0 #assuming that we are in relative alt to home and takeoff is 0

    print("Moving " + str(dist) + "m at " + str(y) + "°")

    return targetCoord

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