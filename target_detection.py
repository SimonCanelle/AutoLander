import cv2
import datetime
import numpy as np
import pause
import serial
import sys, getopt
import time

from geographiclib.geodesic import Geodesic as geo


filename = None
autoPicture = True
camId = 0
noSerial = False
cameraRotation = 0

# we know the altitude???
altitude = 20
fov = 160
pfov = 120

matrix = None
distortion = None

repeat = True


def main():
    global repeat
    
    get_args(sys.argv[1:])

    load_camera_calibration_data()
    
    port = open_port()

    while repeat:
        print("========================================")
        initGPS = get_init_gps_coords(port)

        while repeat:
            try:
                # Find blue blob in image
                img = get_image(filename)
                contour, mask = find_blue(img) #or find_orange(img) WHICH ONE!!!
                target_x, target_y = find_target(img, contour)
                center_x, center_y = find_image_center(img)

                if autoPicture == False:
                    draw_circle(img, target_x, target_y, mask)
                    repeat = False

                # Get distance from drone
                x, y = distance(target_x, target_y, center_x, center_y)
                newGPS = move_gps(x, y, initGPS)

                print(initGPS)
                print(newGPS)

                send_new_gps_coords(port, newGPS)
                break;
            except KeyboardInterrupt:
                sys.exit()
            except Exception as e:
                print(e)
                pass


def distance(target_x, target_y, center_x, center_y):

    pixel_size_metre = 0.000729394*altitude + 0.0000000013

    x = (target_x - center_x) * pixel_size_metre 
    y = (center_y - target_y) * pixel_size_metre 

    print("Moving by: " + str(x) + "m east; " + str(y) + "m north")

    return x, y


def move_gps(x, y, initGPS):
    # https://gis.stackexchange.com/a/412001
    geod = geo.WGS84
    theta = np.arctan2(y, x)
    azimuth = theta - 90
    dist = np.sqrt(x**2 + y**2)

    g = geod.Direct(initGPS[0], initGPS[1], azimuth, dist)
    lat = g["lat2"]
    lng = g["lon2"]

    print("Moving " + str(dist) + "m at " + str(y) + "°")

    return [lat, lng]


def get_init_gps_coords(port):
    print("getting GPS coordinates...")
    if noSerial:
        return [48.51079433248238, -71.64953214980738, 15]      # Alma
    else:
        while True:
            msg = port.readline()
            if msg:
                msg = str(msg)
                print(msg)

                if "VAMUdeS" in msg:
                    print("Received GPS coordinates!")
                    msg = msg.replace("'", "")
                    msg = msg.replace("\\", ",")
                    msg = msg.replace(":", ",")
                    msg = msg.replace("{", "").replace("}", "")
                    msg = msg.replace(" ", "").replace("\n", "").replace("\r", "")
                    data = msg.split(",")

                    lat = int(data[1]) / 10000000
                    lng = int(data[3]) / 10000000
                    alt = int(data[5])
                    print("Latitude: " + str(lat) + " - Longitude: " + str(lng) + " - Altitude: " + str(alt))

                    return [lat, lng, alt]

def send_new_gps_coords(port, newGPS):
    #if noSerial == False:
    lat = str(int(newGPS[0] * 10000000))
    lng = str(int(newGPS[1] * 10000000))

    cmd = "{VAMUdeS _ lat: " + lat + ", lng: " + lng + "}\0"
    print(cmd)

    if noSerial == False and newGPS:
        port.write(cmd.encode())
    else:
        print("Command not sent through serial (--no-serial)")
        

def get_image(filename=None):
    img = None

    if filename:
        # Load image
        img = cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2RGB)
        img = clean_image(img)

    else:
        if autoPicture:
            print("Taking picture of the ground")
        else:
            print("Checking blue landing target from camera")
            print("Press 'c' to capture")
            print("Press 'q' to exit")
            print("Press 'n' to load next camera (give ~5s to load)")
            print("Press 'r' to reset capture (give ~5s to load)")

        global camId
        if camId > 10:
            camId = 0
        cap = None
        try:
            cap = cv2.VideoCapture(camId)
            #waitTime = datetime.datetime.now() + datetime.timedelta(seconds=10) 
            #pause.until(waitTime)
            
        except KeyboardInterrupt:
            sys.exit()
        except:
            print("Unable to open camera with camId == " + str(camId))
            camId += 1
            
        while(True):
            ret, frame = cap.read()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            if autoPicture:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if cv2.countNonZero(gray) > 1:
                    img = frame
                    print("Got picture")
                    break
                else:
                    print("Could not take picture of the ground")
            else:
                cv2.imshow("Webcam", frame[:, :, ::-1])

                key = cv2.waitKey(1)
                if key == ord('c'):         # Capture
                    img = frame
                    break
                elif key == ord('q'):       # Quit
                    break
                elif key == ord('n'):       # Next camera
                    print("loading next camera")
                    camId += 1
                    cap.release()
                    cv2.destroyAllWindows()
                    cap = cv2.VideoCapture(camId)

                elif key == ord('r'):       # Reset
                    print("resetting qr capture")
                    camId = 0
                    cap.release()
                    cv2.destroyAllWindows()
                    cap = cv2.VideoCapture(camId)

        cap.release()
        cv2.destroyAllWindows()


    if cameraRotation != 0:
        cv2.imshow("non-rotated", img)
        cv2.waitKey(2000)
        img = cv2.rotate(img, cameraRotation)
        cv2.imshow("rotated", img)
        cv2.waitKey(2000)
        
    logFile = ("log/" + datetime.datetime.now().isoformat() + ".png").replace(':', '-')
    cv2.imwrite(logFile, img)
    return img


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

"""def find_orange(img):
    # Define range of blue color in RGB
    #lower_blue = np.array([[np.array([100, 200, 150], dtype=np.uint8)]])
    #upper_blue = np.array([[np.array([220, 255, 255], dtype=np.uint8)]])
    lower_orange = np.array([10,  0, 195])
    upper_orange = np.array([70, 255, 255])

    # Convert to HSV
    #lower_blue = cv2.cvtColor(lower_blue, cv2.COLOR_RGB2HSV)
    #upper_blue = cv2.cvtColor(upper_blue, cv2.COLOR_RGB2HSV)
    hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_orange, upper_orange)
    #cv2.imshow('mask', mask)

    # Find contours in the binary image
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Get the largest contour, which should be the orange blob
    if len(contours):
        largest_contour = max(contours, key=cv2.contourArea)
        return largest_contour, mask
    else:
        print("Could not find orange color in image")
        return [], None
"""


def find_target(img, largest_contour):
    # Get the center coordinates of the blue blob
    M = cv2.moments(largest_contour)
    if M['m00']:
        target_x = int(M['m10'] / M['m00'])
        target_y = int(M['m01'] / M['m00'])
        return target_x, target_y
    else:
        return 0, 0


def find_image_center(img):
    x = img.shape[1] // 2
    y = img.shape[0] // 2

    print(f"Center coordinates: ({x}, {y})")

    return x, y


def draw_circle(img, x, y, mask, wait=True):
    # Draw a circle at the center of the blue or orange blob
    cv2.circle(img, (x, y), 15, (255, 0, 0), 2)

    # Print the center coordinates of the blue or orange blob
    print(f"Target coordinates: ({x}, {y})")

    # Display the image with the circle at the center of the blue or orange blob
    cv2.imshow('image', img[:, :, ::-1])
    cv2.imshow('mask', mask)

    if wait:
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def clean_image(image):
    # convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _,thresh = cv2.threshold(gray, 1, 255, cv2.THRESH_BINARY)

    # find image bounding box
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    x, y, w, h  = cv2.boundingRect(max(contours, key=cv2.contourArea))

    image = undistort_image(image[y:y+h,x:x+w])
    return image


def undistort_image(image):
    return image
    h, w = image.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(matrix, distortion, (w, h), 1, (w, h))

    # undistort
    return cv2.undistort(image, matrix, distortion, None, newCameraMatrix)


def open_port():
    if noSerial == False:
        try:
            port = serial.Serial("/dev/ttyUSB0", baudrate=57600, timeout=1.0)
        except:
            port = serial.Serial("/dev/ttyUSB1", baudrate=57600, timeout=1.0)
    
        print("Waiting for comm on " + port.name)
        return port
    else:
        return None
    

def load_camera_calibration_data():
    global matrix
    global distortion

    #FOR THE HD_USB_5MPIXEL 
    #Camera matrix:
    matrix = np.array([[971.18649399,   0.,         317.3983185 ],
                       [  0.,         999.43828656, 301.14003337],
                       [  0.,           0.,           1.        ]])

    #Distortion coefficient:
    distortion = np.array([[-2.68619503e-01, -7.92314116e+00, -2.48365076e-02,  6.06200146e-03, 8.40377809e+01]])

    #Rotation Vectors:
    r_vecs = np.array([[[-0.74389564], [ 0.73871537],[ 1.43447184]],
                       [[-0.80088309],[-0.58288342],[-0.97748316]],
                       [[-0.83978449],[-0.50538039],[-0.18819522]],
                       [[-0.97000256],[-0.09632152],[-0.02980104]],
                       [[-0.92176499],[-0.39732111],[-0.64083314]], 
                       [[-0.60562342],[-0.30584113],[-0.69608166]], 
                       [[-0.43686067],[-0.45053356],[-1.51073472]], 
                       [[-0.51181767],[ 0.37429553],[ 1.16561755]], 
                       [[-0.94950024],[ 0.11213924],[ 0.81062089]], 
                       [[-0.73592202],[ 0.35099987],[ 0.88015607]], 
                       [[-0.86982213],[-0.38782379],[-0.43768036]], 
                       [[-0.67101953],[-0.31623214],[-0.21048545]]])

    #Translation Vectors:
    t_vecs = np.array([[[ 2.5805008 ], [-3.7910744 ],[31.20359942]],
                       [[-4.16363233], [-1.10713786], [22.4069761 ]],
                       [[-2.58401296], [-5.44365188], [30.0330236 ]],
                       [[-3.40974208], [-6.26196546], [25.3419652 ]],
                       [[-4.5017457 ], [-1.56363447], [23.53909427]], 
                       [[-4.29223701], [-1.48742206], [28.89465423]], 
                       [[-2.97408123], [-0.96975281], [27.70274602]], 
                       [[ 0.65887606], [-6.52580707], [30.13378209]], 
                       [[ 1.42543253], [-5.06814622], [27.36347452]], 
                       [[ 1.0356917 ], [-5.05927334], [32.91445674]],
                       [[-4.85865114], [-1.94628257], [29.34353319]], 
                       [[-4.48529458], [-2.9189142 ], [28.80113041]]])


def get_args(argv):
    global filename
    global autoPicture
    global camId
    global noSerial
    global cameraRotation

    opts, args = getopt.getopt(argv, "hf:ar:", ["camera=","no-serial"])
    for opt, arg in opts:
        if opt == "-h":
            print("\n Blue landing target detection for VAMUdeS - AEAC 2023")
            print("\tBy Pascal-Emmanuel Lachance\n")
            print("\tusage: blue_detect.py [-h] [-f FILE] [-a AUTO] [--camera ID]\n")
            print("\toptional arguments:\n")
            print("\t\t-h\t\t show this help message and exit")
            print("\t\t-f\t\t <string> input image file as .png target detection. If absent, will read from camera")
            print("\t\t-a\t\t automatically take picture when a command is received")
            print("\t\t-r\t\t picture rotation (anti-clockwise) in degrees")
            print("\t\t--camera\t <int> ID of the camera to use if no input image is provided.")
            print("\t\t--no-serial\t <int> Don't take GPS positions from serial port.")

            exit()

        elif opt in ("-f"):
            filename = arg
            print("Detecting targets in: " + filename)

        elif opt in ("-a"):
            autoPicture = False
            print("Taking pictures automatically" if autoPicture else "Waiting for user input to take pictures")

        elif opt in ("-r"):
            val = int(arg)
            if val == 0:
                pass
            elif val == 90:
                cameraRotation = cv2.ROTATE_90_COUNTERCLOCKWISE
            elif val == 180:
                cameraRotation = cv2.ROTATE_180
            elif val == 270 or val == -90:
                cameraRotation = cv2.ROTATE_90_CLOCKWISE
            else:
                print("Invalid camera rotation value, keeping the rotation at 0")
                continue
                
            print("Rotating pictures by " + str(cameraRotation))

        elif opt in ("--no-serial"):
            noSerial = True;
            print("Skipping serial port initialization")

        elif opt in ("--camera"):
            camId = abs(int(arg))
            print("Using camera #" + str(camId))

if __name__ == "__main__":
    main()
