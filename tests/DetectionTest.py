from dronekit import Command, connect, VehicleMode, mavutil
import time
import numpy as np
import serial
import cv2
import argparse
from geographiclib.geodesic import Geodesic as geo

class Coordinate:
    lat: float
    lon: float
    alt: float

camId = 0
maxRetries = 100
# we know the altitude???
altitude = 15   #TODO set altitude to loiter alt

pargs = None

max_value = 255
max_value_H = 360
low_H = 0
low_S = 0
low_V = 0
high_H = max_value_H
high_S = max_value
high_V = max_value
window_capture_name = 'Video Capture'
window_detection_name = 'Object Detection'
low_H_name = 'Low H'
low_S_name = 'Low S'
low_V_name = 'Low V'
high_H_name = 'High H'
high_S_name = 'High S'
high_V_name = 'High V'


## [low]
def on_low_H_thresh_trackbar(val):
    global low_H
    global high_H
    low_H = val
    low_H = min(high_H-1, low_H)
    cv2.setTrackbarPos(low_H_name, window_detection_name, low_H)
## [low]

## [high]
def on_high_H_thresh_trackbar(val):
    global low_H
    global high_H
    high_H = val
    high_H = max(high_H, low_H+1)
    cv2.setTrackbarPos(high_H_name, window_detection_name, high_H)
## [high]

def on_low_S_thresh_trackbar(val):
    global low_S
    global high_S
    low_S = val
    low_S = min(high_S-1, low_S)
    cv2.setTrackbarPos(low_S_name, window_detection_name, low_S)

def on_high_S_thresh_trackbar(val):
    global low_S
    global high_S
    high_S = val
    high_S = max(high_S, low_S+1)
    cv2.setTrackbarPos(high_S_name, window_detection_name, high_S)

def on_low_V_thresh_trackbar(val):
    global low_V
    global high_V
    low_V = val
    low_V = min(high_V-1, low_V)
    cv2.setTrackbarPos(low_V_name, window_detection_name, low_V)

def on_high_V_thresh_trackbar(val):
    global low_V
    global high_V
    high_V = val
    high_V = max(high_V, low_V+1)
    cv2.setTrackbarPos(high_V_name, window_detection_name, high_V)



def main():
    print("Starting AutoLander")

    ## [window]
    cv2.namedWindow(window_capture_name)
    ## [window]

    ## [trackbar]
    cv2.createTrackbar(low_H_name, window_capture_name, low_H, max_value_H, on_low_H_thresh_trackbar)
    cv2.createTrackbar(high_H_name, window_capture_name, high_H, max_value_H, on_high_H_thresh_trackbar)
    cv2.createTrackbar(low_S_name, window_capture_name, low_S, max_value, on_low_S_thresh_trackbar)
    cv2.createTrackbar(high_S_name, window_capture_name, high_S, max_value, on_high_S_thresh_trackbar)
    cv2.createTrackbar(low_V_name, window_capture_name, low_V, max_value, on_low_V_thresh_trackbar)
    cv2.createTrackbar(high_V_name, window_capture_name, high_V, max_value, on_high_V_thresh_trackbar)
    ## [trackbar]

    #find landing target
    print("Starting lz detection")
    while True:
        lzCoord = getTargetPosition()
    
    #modify and execute mission for landing
        print("Sending nav point to drone")
        if pargs.fakegps is not None :
            executeLanding(lzCoord)

    #close everything
    print("Program finished")
    
def getTargetPosition():
    global pargs
    for i in range(0, maxRetries):
        #1. get drone gps coordinate
        if pargs.fakegps:
            droneCoord = Coordinate
            droneCoord.lat = pargs.fakegps[0]
            droneCoord.lon = pargs.fakegps[1]
            droneCoord.alt = pargs.fakegps[2]

        else:
            # [48.51079433248238, -71.64953214980738, 0]      # Alma
            droneCoord = Coordinate
            droneCoord.lat = 48.51079433248238
            droneCoord.lon = -71.64953214980738
            droneCoord.alt = 15

        #2. get image
        while True:
            img = get_image()
            cv2.imshow(window_capture_name, img)
            cv2.waitKey(1)
            #3. find target
            #3.1 find blue
            contour, mask = find_blue(img)
            if mask is None :
                time.sleep(1)
            else:
                break
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
        cv2.line(imgshow, (target_x, target_y), (target_x+3, target_y), (0, 255, 0), 2)
        cv2.line(imgshow, (target_x, target_y), (target_x-3, target_y), (0, 255, 0), 2)
        cv2.line(imgshow, (target_x, target_y), (target_x, target_y+3), (0, 255, 0), 2)
        cv2.line(imgshow, (target_x, target_y), (target_x, target_y-3), (0, 255, 0), 2)
        cv2.imshow(window_detection_name, imgshow)
        cv2.waitKey(1)

    lzCoord = move_gps(x, y, droneCoord)
    print("lzCoord lat:"+str(lzCoord.lat)+" long:"+str(lzCoord.lon)+" alt:"+str(lzCoord.alt))
    return lzCoord


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
            print(str("Unable to open camera with camId == " + str(camId)))
            camId += 1

    while(True):
        ret, frame = cap.read()
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
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
    lower_blue = np.array([100,  80, 100])
    upper_blue = np.array([300, 255, 255])
    #lower_blue = np.array([low_H,  low_S, low_V])
    #upper_blue = np.array([high_H, high_S, high_V])

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
        print("Could not find blue color in image")
        return [], None

def find_image_center(img):
    x = img.shape[1] // 2
    y = img.shape[0] // 2

    print(f"Center coordinates: ({x}, {y})")

    return x, y

    
def executeLanding(lzCoord):
    global pargs
    landCmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                               mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0,
                               0, mavutil.mavlink.PRECISION_LAND_MODE_DISABLED, 0, 0, lzCoord.lat, lzCoord.lon, lzCoord.alt)
    print(landCmd)

def argParser():
    parser = argparse.ArgumentParser()
    parser.add_argument("-gps", "--fakegps",     default=None, type=float, nargs=3, help="fake gps coordinates for testing lat lon alt")
    parser.add_argument("-si",  "--showimages", default=None, action="store_true", help="show images on screen")
    global pargs
    pargs = parser.parse_args()

if __name__ == "__main__":
    argParser()
    main()