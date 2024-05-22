import cv2

def get_image():
    img = None
    camId = 2
    camOpen = False
    cap = None
    while not camOpen:
        if camId > 10:
            camId = 0
        try:
            cap = cv2.VideoCapture(camId)
            ret, frame = cap.read()
            if frame is not None:
                camOpen = True
            else:
                camId += 1
            # waitTime = datetime.datetime.now() + datetime.timedelta(seconds=10)
            # pause.until(waitTime)
        except:
            #debPrint(str("Unable to open camera with camId == " + str(camId)))
            camId += 1

    #ret, frame = cap.read()

    if cap is not None:
        cap.release()

    return frame

#while True:
while True:
    img = get_image()
    cv2.imshow("test",img)
    cv2.waitKey(1)