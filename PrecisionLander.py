from dronekit import Command, connect, VehicleMode, mavutil
import time
import numpy as np
import cv2
import argparse
import timeit

camId = 0
exchange = None
landing_alt = 0
def connectToDrone():
    global vehicle
    global pargs
    global cmds
    connected = False
    vehicle = None
    while not connected:
        try:
            vehicle = connect(pargs.dronelink, wait_ready=True)
            cmds = vehicle.commands
            cmds.download()
            cmds.wait_ready()
            connected = True
        except:
            #wait before trying to connect again
            time.sleep(10)

def connectCam():
    global cap
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

def detectionLoop():
    global cap
    global vehicle
    global pargs
    landDone = False
    loopStart = 0
    loopEnd = 0
    while (not landDone):
        if cmds[cmds.next].command == mavutil.mavlink.MAV_CMD_NAV_LAND:
            if pargs.frequency:
                loopStart = time.perf_counter_ns()
            #take capture
            ret, frame = cap.read()
            if frame.shape[0] > 0 and frame.shape[1] > 0:
                #find blue lz
                contour, mask = find_blue(frame)
                if contour != []:
                    #find detection center
                    target_x, target_y = find_target(frame, contour)
                    if target_x is not None and target_y is not None:
                        #find target center
                        center_x, center_y = find_image_center(frame)
                        #calculate lz displacement in xy
                        lzCoord = distanceXY(target_x, target_y, center_x, center_y)
                        #calculate lz params
                        angle_x, angle_y, distance = calculateParams(lzCoord)
                        #send message
                        sendLandingTarget(angle_x, angle_y, distance)

                        if pargs.frequency:
                            loopEnd = time.perf_counter_ns()
                            print("Detection Loop time (ns): ", loopEnd - loopStart)

        else:
            landDone = True
            cap.release()
            vehicle.close()

def sendLandingTarget(angle_x, angle_y, distance):
    global vehicle
    send_message = vehicle.message_factory.landing_target(
        0,  # time unused time.time_ns()/1000.0 for usec posix
        0,  # target num
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        angle_x,
        angle_y,
        distance,
        0,  #size x/y unused
        0
    )
    vehicle.send_mavlink(send_message)

def calculateParams(lzCoord):
    global vehicle
    z = np.hypot(lzCoord[0], lzCoord[1])
    angle_x = np.arccos(lzCoord[0]/z)
    angle_y = np.arccos(lzCoord[1]/z)
    distance = np.hypot(vehicle.location.global_relative_frame.alt-landing_alt, z)
    return angle_x, angle_y, distance


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
        #debPrint("Could not find blue color in image")
        return [], None

def find_target(img, largest_contour):
    # Get the center coordinates of the blue blob
    M = cv2.moments(largest_contour)
    if M['m00']:
        target_x = int(M['m10'] / M['m00'])
        target_y = int(M['m01'] / M['m00'])
        return target_x, target_y
    else:
        return None, None

def find_image_center(img):
    x = img.shape[1] // 2
    y = img.shape[0] // 2

    #debPrint(f"Center coordinates: ({x}, {y})")
    return x, y

def distanceXY(target_x, target_y, center_x, center_y):
    #TODO calcul de grosseur de pixel
    pixel_size_metre = 0.000729394*vehicle.location.global_relative_frame.alt + 0.0000000013

    x = (target_x - center_x) * pixel_size_metre
    y = (center_y - target_y) * pixel_size_metre

    #debPrint("Moving by: " + str(x) + "m east; " + str(y) + "m north")
    return x, y

def argParser():
    parser = argparse.ArgumentParser()

    parser.add_argument("-dl", "--dronelink", default='/dev/ttyS0', type=str, help="drone comm link")
    parser.add_argument("-fr", "--frequency", default=False, type=bool, action="store_true",
                        help="activate detection loop frequency print")
    global pargs
    pargs = parser.parse_args()



if __name__ == "__main__":
    argParser()
    connectCam()
    connectToDrone()
    detectionLoop()
