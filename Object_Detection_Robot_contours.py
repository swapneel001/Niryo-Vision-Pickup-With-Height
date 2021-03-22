from __future__ import division
import os
import cv2
import re
import numpy as np
import sys
from threading import Thread
import importlib.util
import time
from PIL import Image, ImageDraw, ImageFont
import io

# utils_cnt has image processing functions, depth_calculate is code for RealSense
import utils_cnt_robot
import depth_calculate

from niryo_one_tcp_client import *
from niryo_one_camera import *
from robot_poses import observation_pose, drop_pose, pose1, pose2, pose3, pose4

# CAMERA_WIDTH = 640
# CAMERA_HEIGHT = 480

# functions to get slope, area, and center tendency of bounding box


def get_slope(rect):
    try:
        if rect is None:
            return None
        slope = rect[1][0]/rect[1][1]
        return slope
    except:
        return None


def get_area(rect):
    if rect is None:
        return None
    area = rect[1][0]*rect[1][1]
    return area


def check_center_tendency(rect):
    """Check if object is inside the workspace completely"""
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    x_values = [box[0][0], box[1][0], box[2][0], box[3][0]]
    maximum = max(x_values)
    minimum = min(x_values)
    if maximum < 197 and minimum > 3:
        return True
    else:
        return False

# capture workspace image


def take_img(client):
    img_work = depth_calculate.get_frames()
    try:
        img_work, _ = extract_img_workspace(
            img_work, img_work, workspace_ratio=0.37)
        print("Workspace shape at robot side", img_work.shape)
    except:
        print("No workspace detected")
    return True, img_work


if __name__ == "__main__":
    # Setting up Niryo One
    client = NiryoOneClient()
    client.connect("10.10.10.10")
    client.calibrate(CalibrateMode.AUTO)
    client.change_tool(RobotTool.VACUUM_PUMP_1)
    client.create_workspace("workspace", pose1, pose2, pose3, pose4)

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
        frame = utils_cnt_robot.standardize_img(frame)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(frame)
        im_width, im_height = image.size
        # calculate mask for frame (black and white image)
        mask = utils_cnt_robot.objs_mask(frame)
        cv2.imshow('mask', mask)
        # drawing region of interest on the color image
        cv2.line(frame, (0, 220), (200, 220), (255, 0, 0), thickness_big)
        cv2.line(frame, (0, 520), (200, 520), (0, 0, 255), thickness_big)
        cv2.imshow("Robot Camera frame", frame)
        # detect objects using contours and draw box around them
        obj_found = True
        try:
            bounding_box, rect = utils_cnt_robot.bounding_box(frame, mask)
        except TypeError:
            print("No object detected")
            obj_found = False
            continue

        key = True
        new_area = get_area(rect)
        new_slope = get_slope(rect)
        # centre and angle of rotation of contour
        centre, angle = rect[0], rect[2]
        print("Centre of contour is", centre)

        center_tendency = check_center_tendency(rect)
        if(center_tendency):
            print("center object")
        if not ((new_area >= 0.95*prev_area and new_area <= 1.05*prev_area)):
            if(new_slope != prev_slope):
                # use realsense data to get object distance
                distance = depth_calculate.distance()
                print("Distance to object is: {} m from the camera".format(distance))

                key_box.append(bounding_box)
                key_frame.append((frame))
                print("Area of Detected object: ", new_area)
                print("Slope of Detected object: ", new_slope)
                # height = distance of camera to workspace - distance of camera to object top
                height = 0.45 - distance
                print(height)
                prev_area = new_area
                prev_slope = new_slope
                if obj_found:
                    obj_found, obj = client.get_target_pose_from_rel(
                        "workspace", height, centre[0]/im_width, centre[1]/im_height, angle)
                    client.pick_from_pose(*obj.to_list())
                    client.place_from_pose(*drop_pose.to_list())
                client.move_pose(*observation_pose.to_list())
        key = cv2.waitKey(1)
        if key == 27:  # Esc key
            break

    print("Number of key images: ", len(frame))
    client.set_learning_mode(True)
