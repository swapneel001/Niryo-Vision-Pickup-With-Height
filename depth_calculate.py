import pyrealsense2 as rs
import utils_cnt
import cv2
import numpy as np
import imutils
from niryo_one_camera import *

# set up camera
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)
config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
# config.enable_device_from_file("C:/Users/BHS2SGP/Documents/20210318_142227.bag")

# start reading frames
profile = pipe.start(config)
for x in range(5):
    try:
        pipe.wait_for_frames()
    except:
        x = x-1

# function to get height of object detected


def get_height_pixel(color_frame, color_image_workspace, rect, bigrect):
    centre = rect[0]
    box = cv2.boxPoints(rect)
    box = np.int0(box)

    h, w, _ = color_image_workspace.shape
    #print("workspace pixels",h,w)
    template = cv2.cvtColor(color_image_workspace, cv2.COLOR_BGR2GRAY)
    template = cv2.Canny(template, 50, 200)
    method = eval('cv2.TM_CCOEFF')
    #colorizer = rs.colorizer()
    color_image_full = np.asanyarray(color_frame.get_data())
    gray = cv2.cvtColor(color_image_full, cv2.COLOR_BGR2GRAY)
    # color_image_full = cv2.cvtColor(color_image_full, cv2.COLOR_BGR2RGB)
    found = None
    for scale in np.linspace(0.2, 1.0, 50)[::-1]:
        resized = imutils.resize(gray, width=int(gray.shape[1]*scale))
        r = gray.shape[1]/float(resized.shape[1])
        if resized.shape[0] < h or resized.shape[1] < w:
            break
        edged = cv2.Canny(resized, 50, 200)
        res = cv2.matchTemplate(edged, template, method)
        (_, maxVal, _, maxLoc) = cv2.minMaxLoc(res)
        if found is None or maxVal > found[0]:
            found = (maxVal, maxLoc, r)

    (_, maxLoc, r) = found
    (startX, startY) = (int(maxLoc[0]*r), int(maxLoc[1]*r))
    (endX, endY) = (int((maxLoc[0]+w)*r), int((maxLoc[1]+h)*r))

    centre = (int((centre[0]+maxLoc[0])*r), int((centre[1]+maxLoc[1])*r))
    newrect = []
    newrect.append(int((bigrect[0]+maxLoc[0])*r))
    newrect.append(int((bigrect[1]+maxLoc[1])*r))
    newrect.append(int(bigrect[2]*r))
    newrect.append(int(bigrect[3]*r))

    cv2.rectangle(color_image_full, (startX, startY), (endX, endY), 255, 2)
    cv2.rectangle(color_image_full, (newrect[0], newrect[1]), (
        newrect[0]+newrect[2], newrect[1]+newrect[3]), 255, 2)
    cv2.imshow("Location in full image ", color_image_full)
    key = cv2.waitKey(1)
    if key == 27:
        quit()
    height_pixel = (centre[0], centre[1])
    return height_pixel


# function to get object co-ordinates
# def distance():
while True:
    # read and align colour and depth frame
    try:
        frames = pipe.wait_for_frames()

    except:
        continue
    align = rs.align(rs.stream.color)
    frames = align.process(frames)
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        continue

    # convert frames into numpy arrays readable by opencv
    colorizer = rs.colorizer()
    depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    color_image = np.asanyarray(color_frame.get_data())

    # extract image workspace
    try:
        print("Trying to find workspace ")
        color_image, depth_image = extract_img_workspace(
            color_image, depth_image, workspace_ratio=0.37)
        print("workspace shape is", color_image.shape[0:2])
        # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        #print("Size of color image is {}".format(color_image.shape))
        color_image = cv2.rotate(color_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_image_dim = depth_image.shape
        color_colormap_dim = color_image.shape
        if depth_image_dim != color_colormap_dim:
            color_image = cv2.resize(color_image, dsize=(
                depth_image_dim[1], depth_image_dim[0]), interpolation=cv2.INTER_AREA)
        color_image = utils_cnt.standardize_img(color_image)
    except:
        continue

    # masking the green background of the conveyor belt
    # green will be white, everything else black
    boundaries = [([0, 50, 0], [100, 255, 100])]
    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(color_image, lower, upper)
    # cv2.imshow("mask",mask)

    thresh = cv2.threshold(mask, 0, 255,
                           cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    cv2.imshow("thresh", thresh)

    # getting contours of objects (black pixels) on belt (white pixels)
    cnts, heirarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)[-2:]

    cv2.line(color_image, (220, 0), (220, 200), (0, 0, 255), thickness_big)
    cv2.line(color_image, (520, 0), (520, 200), (0, 0, 255), thickness_big)
    selected_cnts = []
    if cnts is not None:
        # print(len(cnts))
        for cnt in cnts:
            # drawing the minimum area box
            bigrect = cv2.boundingRect(cnt)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            x_values = [box[0][0], box[1][0], box[2][0], box[3][0]]
            # cv2.drawContours(color_image,[box],0,(0,255),2)
            # cv2.imshow('Bounding Box',color_image)
            if min(x_values) > 230 and max(x_values) < 520:
                selected_cnts.append(cnt)

        cnt = max(selected_cnts, key=cv2.contourArea)
        bigrect = cv2.boundingRect(cnt)
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(color_image, [box], 0, (0, 255), 2)
        cv2.imshow('Bounding Box', color_image)
        cv2.imshow('depth frame', depth_image)
        obj_found = False
        y_values = [box[0][1], box[1][1], box[2][1], box[3][1]]
        maximum = max(y_values)
        minimum = min(y_values)
        if maximum < 195 and minimum > 5:
            x, y = get_height_pixel(color_frame, color_image, rect, bigrect)
            distance = depth_frame.get_distance(int(x), int(y))
            print("Distance is : ", distance)
        obj_found = True
        key = cv2.waitKey(1)
        if key == 27:
            quit()
        # if obj_found is True:
            # return distance
