import pyrealsense2 as rs
import utils
import cv2
import numpy as np
import imutils
from niryo_one_camera import *
from A3x3 import *

# set up camera
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)
config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
# config.enable_device_from_file(
#     "C:/Users/BHS2SGP/Documents/recording2.bag")

# start reading frames
# skip first 10 frames to allow auto exposure to adjust
profile = pipe.start(config)
for x in range(10):
    try:
        pipe.wait_for_frames()
    except:
        x = x-1


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
    #color_image_full = cv2.cvtColor(color_image_full, cv2.COLOR_BGR2RGB)
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
    #cv2.imshow("Location in full image ", color_image_full)
    Display3x3("Location in full image", color_image_full, 4)
    key = cv2.waitKey(1)
    if key == 27:
        quit()
    height_pixel = (centre[0], centre[1])
    return height_pixel


def get_frames():
    """Function to read camera frames"""
    frames = pipe.wait_for_frames()
    align = rs.align(rs.stream.color)
    frames = align.process(frames)
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    # convert frames into numpy arrays readable by opencv
    colorizer = rs.colorizer()
    depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    color_image = np.asanyarray(color_frame.get_data())
    return color_image


def distance():
    """Function to get object's height by calculating distance from camera"""
    counter = 0
    obj_found = False
    while True:
        counter += 1
        frames = pipe.wait_for_frames()
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
            #print("Trying to find workspace ")
            color_image, depth_image = extract_img_workspace(
                color_image, depth_image, workspace_ratio=0.37)
            #print("workspace shape is", color_image.shape[0:2])
            #color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            #print("Size of color image is {}".format(color_image.shape))
            color_image = cv2.rotate(color_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            depth_image = cv2.rotate(depth_image, cv2.ROTATE_90_COUNTERCLOCKWISE)
            depth_image_dim = depth_image.shape
            color_colormap_dim = color_image.shape
            if depth_image_dim != color_colormap_dim:
                color_image = cv2.resize(color_image, dsize=(
                    depth_image_dim[1], depth_image_dim[0]), interpolation=cv2.INTER_AREA)
            #color_image = utils_cnt_robot.standardize_img(color_image)
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

        # filling areas between markers and green belt with black -> region of no interest
        area1 = np.array([[[0, 0], [20, 0], [20, 200], [0, 200]]], dtype=np.int32)
        cv2.fillPoly(thresh, area1, 0)
        area2 = np.array(
            [[[520, 0], [541, 0], [541, 200], [520, 200]]], dtype=np.int32)
        cv2.fillPoly(thresh, area2, 0)
        #cv2.imshow("thresh", thresh)
        Display3x3("Thresh", thresh, 2)

        # getting contours of objects (black pixels) on belt (white pixels)
        cnts, heirarchy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                                        cv2.CHAIN_APPROX_SIMPLE)[-2:]
        # print(len(cnts))
        cv2.line(color_image, (220, 0), (220, 200), (0, 0, 255), thickness_small)
        cv2.line(color_image, (525, 0), (525, 200), (0, 0, 255), thickness_small)
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
                if min(x_values) > 220:
                    selected_cnts.append(cnt)

            cnt = max(selected_cnts, key=cv2.contourArea)
            bigrect = cv2.boundingRect(cnt)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            area = rect [1][0]*rect[1][1]
            print("Area of contour is :" ,area)
            cv2.drawContours(color_image, [box], 0, (0, 255), 2)
            if area> 500:
                if counter > 10:  # adding counter function to let contours auto adjust so that only the biggest contour is returned after exposure adjustment
                    #cv2.imshow('Bounding Box', color_image)
                    Display3x3("Bounding box", color_image, 1)
                    #cv2.imshow('depth frame', depth_image)
                    Display3x3("depth frame", depth_image, 3)
                    obj_found = False
                    y_values = [box[0][1], box[1][1], box[2][1], box[3][1]]
                    maximum = max(y_values)
                    minimum = min(y_values)
                    if maximum < 195 and minimum > 5:
                        x, y = get_height_pixel(
                            color_frame, color_image, rect, bigrect)
                        distance = depth_frame.get_distance(int(x), int(y))
                        #print("Distance is : ", distance)
                        obj_found = True
            key = cv2.waitKey(1)
            if key == 27:
                quit()
            if obj_found is True:
                return distance
