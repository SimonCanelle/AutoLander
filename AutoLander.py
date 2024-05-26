from dronekit import Command, connect, VehicleMode, mavutil
import time
import numpy as np
import serial
import cv2
import argparse
from geographiclib.geodesic import Geodesic as geo

camId = 0
maxRetries = 100
# we know the altitude???
altitude = 15   #TODO set altitude to loiter alt

camFOV = 55 #in degrees
camRes = [1080, 720] #x ,y

pargs = None

class Coordinate:
    lat: float
    lon: float
    alt: float

def debPrint(message:str):
    global pargs
    if pargs.debugprints:
        print(message)

def main():
    debPrint("Starting AutoLander")

    #setup camera
    debPrint("Connecting to camera")
    camPort = open_camPort()

    #connect totdrone
    debPrint("Connecting to drone")
    vehicle = connectToDrone()

    #download mission
    debPrint("Getting mission from drone")
    cmds = getMission(vehicle)

    #wait for mission end
    debPrint("Waiting for mission end")
    waitForMissionEnd(vehicle, cmds)

    #find landing target
    debPrint("Starting lz detection")
    lzCoord = getTargetPosition(vehicle)

    #modify and execute mission for landing
    debPrint("Sending nav point to drone")
    executeLanding(vehicle, cmds, lzCoord)

    #close everything
    debPrint("Program finished")
    camPort.close()
    vehicle.close()

def connectToDrone():
    global pargs
    connected = False
    vehicle = None
    while not connected:
        try:
            vehicle = connect(pargs.dronelink, wait_ready=True)
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
    while vehicle.mode.name != VehicleMode('AUTO') and cmds.next > 0:
        debPrint("Waiting for mission start")
        time.sleep(5)

    debPrint("Waiting for mission end")
    while(not readyToLand):
        #loiter at mission end
        #TODO verify end of mission mode
        if vehicle.mode.name == VehicleMode('LOITER') or vehicle.mode.name == VehicleMode('QLOITER'):
            #ready to land when num of mission items = next
            if cmds.next == cmds.count-1:
                readyToLand = True
        time.sleep(1)

def getTargetPosition(vehicle):
    global pargs
    for i in range(0, maxRetries):
        #1. get drone gps coordinate
        if pargs.fakegps:
            droneCoord = Coordinate
            droneCoord.lat = pargs.fakegps[0]
            droneCoord.lon = pargs.fakegps[1]
            droneCoord.alt = pargs.fakegps[2]

        else:
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

    if pargs.showimages:
        imgshow = cv2.drawContours(img, contour, -1, (0, 255, 0), 2)
        cv2.line(imgshow, (center_x, center_y), (center_x+3, center_y), (0, 255, 0), 2)
        cv2.line(imgshow, (center_x, center_y), (center_x-3, center_y), (0, 255, 0), 2)
        cv2.line(imgshow, (center_x, center_y), (center_x, center_y+3), (0, 255, 0), 2)
        cv2.line(imgshow, (center_x, center_y), (center_x, center_y-3), (0, 255, 0), 2)
        cv2.imshow("detection", imgshow)
        cv2.waitKey(0)

    lzCoord = move_gps(x, y, droneCoord)
    debPrint("lzCoord lat:"+str(lzCoord.lat)+" long:"+str(lzCoord.lon)+" alt:"+str(lzCoord.alt))
    return lzCoord
    cmds = []

def executeLanding(vehicle, cmds, lzCoord):
    global pargs

    landCmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                      mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
                      0, mavutil.mavlink.PRECISION_LAND_MODE_DISABLED, 0, 0, lzCoord.lat, lzCoord.lon, lzCoord.alt)

    if pargs.addoroverwrite is False: #false = no overwrite
        missionList = []
        for cmd in cmds:
            missionList.append(cmd)
        missionList.append(landCmd)
        cmds.clear()
        for cmd in missionList:
            cmds.add(cmd)
    else:
        cmds.clear()
        cmds.add(landCmd)

    #TODO check if its better to
    # 1. add a waypoint at the end and continue mission
    # 1.1 make sure that MIS_RESTART is properly set to CONTINUE mission from start
    # 2. clear and create new mission with one point : land
    # 2.1 make sure that MIS_RESTART is properly set to RESTART mission from start
    if pargs.printmission:
        for cmd in cmds:
            print(cmd.command)
            print(cmd.location.global_relative_frame)

    cmds.upload()
    #continue mission with landing nav point
    vehicle.mode = VehicleMode("AUTO")

def distance(target_x, target_y, center_x, center_y):
    #TODO calcul de grosseur de pixel
    pixel_size_metre = 0.000729394*altitude + 0.0000000013

    x = (target_x - center_x) * pixel_size_metre
    y = (center_y - target_y) * pixel_size_metre

    debPrint("Moving by: " + str(x) + "m east; " + str(y) + "m north")

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

    debPrint("Moving " + str(dist) + "m at " + str(y) + "°")

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
            debPrint(str("Unable to open camera with camId == " + str(camId)))
            camId += 1

    while(True):
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if cv2.countNonZero(gray) > 1:
            img = frame
            debPrint("Got picture")
            break
        else:
            debPrint("Could not take picture of the ground")

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
    # Define range of blue color in RGB ##why RGB? BGR is fine...
    #lower_blue = np.array([[np.array([100, 200, 150], dtype=np.uint8)]])
    #upper_blue = np.array([[np.array([220, 255, 255], dtype=np.uint8)]])
    lower_blue = np.array([30,  100, 200])
    upper_blue = np.array([180, 255, 255])

    # Convert to HSV
    #lower_blue = cv2.cvtColor(lower_blue, cv2.COLOR_RGB2HSV)
    #upper_blue = cv2.cvtColor(upper_blue, cv2.COLOR_RGB2HSV)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

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
        debPrint("Could not find blue color in image")
        return [], None

def find_image_center(img):
    x = img.shape[1] // 2
    y = img.shape[0] // 2

    debPrint(f"Center coordinates: ({x}, {y})")

    return x, y

def open_camPort():
    #unused with usb camera
    global pargs
    if pargs.serialcamera:
        camConnected = False
        port = None
        while not camConnected:
            try:
                port = serial.Serial(pargs.serialcamera, baudrate=57600, timeout=1.0)
                camConnected = True
            except:
                time.sleep(5)
        return port
    else:
        return None

def argParser():
    parser = argparse.ArgumentParser()
    parser.add_argument("-pf",  "--picturefile",    default=None, type=str,
                        help="To use a photo instead of camera, path to image")

    parser.add_argument("-gps", "--fakegps",        default=None, type=float, nargs=3,
                        help="fake gps coordinates for testing, lat lon alt")

    parser.add_argument("-pm",  "--printmission",   default=None,  action="store_true",
                        help="print mission to terminal") #deactivate send to drone???

    parser.add_argument("-d",   "--debugprints",    default=None, action="store_true",
                        help="activate debug debug prints")

    parser.add_argument("-si",  "--showimage",      default=None, action="store_true",
                        help="show images on screen")

    parser.add_argument("-sc",  "--serialcamera",   default=None, type=str,
                        help="activate serial camera with path $VALUE$")

    parser.add_argument("-dl",  "--dronelink",      default='127.0.0.1:14550', type=str,
                        help="drone comm link")

    parser.add_argument("-ao",  "--addoroverwrite", default=True, type=bool, #action store false???
                        help="Overwrite mission (True) or add landing point to end of mission (False)")

#TODO
    parser.add_argument("-sd",  "--savedetection",  default=None, action="store_true",
                        help="save captured detection picture")

    parser.add_argument("-rtl", "--returntolanding", default=None,  action="store_true",
                        help="return to home location (launch point) instead of \
                        landing straight down in case of no detection")

    global pargs
    pargs = parser.parse_args()

if __name__ == "__main__":
    argParser()
    main()