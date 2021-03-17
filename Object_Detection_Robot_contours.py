from __future__ import division
import os
import cv2
import re
import numpy as np
import sys
from threading import Thread
import importlib.util
import matplotlib.pyplot as plt
import time
from PIL import Image, ImageDraw, ImageFont
import io
import scipy.misc
from six import BytesIO
import pathlib

#utils_cnt has image processing functions, depth_calculate is code for RealSense
import utils_cnt
import depth_calculate

from niryo_one_tcp_client import *
from niryo_one_camera import *

# CAMERA_WIDTH = 640
# CAMERA_HEIGHT = 480

#functions to get slope, area, and center tendency of bounding box
def get_slope(rect):
    if rect is None:
        return None
    slope = rect[1][0]/rect[1][1]
    return slope


def get_area(rect):
    if rect is None:
        return None
    area = rect[1][0]*rect[1][1]
    return area


def check_center_tendency(rect):
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    x_values = [box[0][0],box[1][0],box[2][0],box[3][0]
    maximum = max(x_values)
    minimum = min(x_values)
    if maximum < 195 and minimum > 5:
        return True
    else:
        return False

#capture workspace image
def take_img(client):
    a, mtx, dist = client.get_calibration_object()
    # while 1:
    a, img_compressed = client.get_img_compressed()
    img_raw = uncompress_image(img_compressed)
    img_work = undistort_image(img_raw, mtx, dist)
    img_work, _ = extract_img_workspace(img_work,img_work, workspace_ratio=1)
    if img_work is not None:
        cv2.imshow('Object', img_work)
        cv2.waitKey(1)
    return True, img_work

#Uncompress and Undistort image
def uncompress_image(compressed_image):
    """
    Take a compressed img and return an OpenCV image
    :param compressed_image: compressed image
    :return: OpenCV image
    """
    np_arr = np.fromstring(compressed_image, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


def undistort_image(img, mtx, dist, newcameramtx=None):
    """
    Use camera intrinsics to undistort raw image
    :param img: Raw Image
    :param mtx: Camera Intrinsics matrix
    :param dist: Distortion Coefficient
    :param newcameramtx: Camera Intrinsics matrix after correction
    :return: Undistorted image
    """
    return cv2.undistort(src=img, cameraMatrix=mtx,
                         distCoeffs=dist, newCameraMatrix=newcameramtx)

drop_pose = PoseObject(  # position for the robot to place object
    x=0.038, y=-0.107, z=0.136,
    roll=-0.084, pitch=1.428, yaw=-1.194,
)
observation_pose = PoseObject(  # position for the robot to watch the workspace
    x=0.111, y=0.007, z=0.225,
    roll=0.255, pitch=0.992, yaw=0.070,
)

if __name__ == "__main__":
    # Setting up Niryo One
    client = NiryoOneClient()
    client.connect("10.10.10.10")
    client.calibrate(CalibrateMode.AUTO)
    client.change_tool(RobotTool.VACUUM_PUMP_1)
    workspace = "workspace1"

    # initialising arrays for various data required
    key_box = []
    key_img_arr = []
    key_frame = []
    bounding_box = []
    prev_area = 0
    prev_slope = 0
    new_slope = 0
    new_area = 0
    a = False
    client.move_pose(*observation_pose.to_list())

    while "User presses esc":
        # take image of workspace
        a, frame = take_img(client)
        if (frame is None):
            continue
        # print(frame.dtype)
        # print(frame.shape)
        frame = utils_cnt.standardize_img(frame)
        image = Image.fromarray(frame)
        im_width, im_height = image.size

        # calculate mask for frame (black and white image)
        mask = utils_cnt.objs_mask(frame)
        # apply mask to image
        img_masked = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow('mask', img_masked)

        # detect objects using contours and draw box around them
        # extract objects from frame
        obj_found = True
        obj = utils_cnt.extract_objs(frame, mask)
        if obj is None:
            obj_found = False
        # box co-ordinates : x1,y1,x2,y2
        bounding_box,rect = utils_cnt.bounding_box(frame, mask)

        key = True
        new_area = get_area(rect)
        new_slope = get_slope(rect)

        #check if box is inside the camera workspace (near the center): this function will be more useful
        #during start stop when object is initially moving

        center_tendency = check_center_tendency(rect)
        #if(center_tendency):
        print("center object")
        if not ((new_area >= 0.95*prev_area and new_area <= 1.05*prev_area)):
            if(new_slope != prev_slope):
                #use realsense data to get object distance
                distance = depth_calculate.distance()
                print("Distance to object is: {} m from the camera".format(distance))
            
                key_box.append(bounding_box)
                key_frame.append((frame))
                print("Area of Detected object: ", new_area)
                print("Slope of Detected object: ", new_slope)
                #height = distance of camera to workspace - distance of camera to object top
                height = 0.45 - distance
                print(height)
                prev_area = new_area
                prev_slope = new_slope
                if obj_found:
                    obj_found, obj = client.get_target_pose_from_rel(
                        workspace, height , obj.x/im_width, obj.y/im_height, obj.angle)
                    client.pick_from_pose(*obj.to_list())
                    client.place_from_pose(*drop_pose.to_list())
                client.move_pose(*observation_pose.to_list())
        key = cv2.waitKey(1)
        if key == 27:  # Esc key
            break

    print("Number of key images: ", len(frame))
    client.set_learning_mode(True)