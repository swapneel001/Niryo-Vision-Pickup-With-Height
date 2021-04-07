import cv2
import numpy
from niryo_one_camera import *
import A3x3
import utils


cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)

if not cap.isOpened():
    print("Cannot open camera")
    exit()
while True:
    # Capture frame-by-frame
    ret, frame = cap.read()
    frame = utils.undistort_image(frame)
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break

    try:
        frame,_=extract_img_workspace(frame,frame, workspace_ratio=0.37)
    except:
        print("no workspace detected")
    cv2.imshow("workspace",frame)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    shape = frame.shape
    
    # calculate mask for frame (black and white image)
    mask = utils.objs_mask(frame)
    cv2.imshow("Mask",mask)
    bounding_box, rect, centre = utils.bounding_box(frame, mask)

    #get x,y
    x,y = centre
    print("Centre of co-ordinates (x,y) is {}".format(centre))

    # if frame is read correctly ret is True
    key = cv2.waitKey(1)
    if key == 'escape':
        break