from __future__ import division
import os
import cv2
import re
import math
import numpy as np
import sys
from threading import Thread
import importlib.util
import time
from PIL import Image, ImageDraw, ImageFont
import io
from A3x3 import *
import conveyor_belt

# utils_cnt has image processing functions, depth_calculate is code for RealSens
import utils
import depth_calculate

from niryo_one_tcp_client import *
from niryo_one_camera import *
from robot_poses import observation_pose, drop_pose1, pose1, pose2, pose3, pose4


if __name__ == "__main__":

    # Setting up Niryo One
    client = NiryoOneClient()
    client.connect("10.10.10.81")
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
    key = True
    client.move_pose(*observation_pose.to_list())
    conveyor_belt.turnOn()
    while "User presses esc":
        # take image of workspace
        a, frame = utils.take_img(client)
        if (frame is None):
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        shape = frame.shape
        # calculate mask for frame (black and white image)
        mask = utils.objs_mask(frame)
        #cv2.imshow('mask', mask)
        maskCopy = mask.copy()
        maskCopy = cv2.rotate(maskCopy, cv2.ROTATE_90_COUNTERCLOCKWISE)
        #Display3x3("mask", maskCopy, 1)

        frame_copy = frame.copy()
        # drawing region of interest on the color image
        cv2.line(frame_copy, (0, 220), (200, 220),
                 (255, 0, 0), thickness_small)
        cv2.line(frame_copy, (0, 525), (200, 525),
                 (0, 0, 255), thickness_small)
        #cv2.imshow("Robot Camera frame", frame_copy)
        # detect objects using contours and draw box around them
        obj_found = True
        try:
            bounding_box, rect, centre = utils.bounding_box(frame, mask)
        except TypeError:
            print("No object detected")
            obj_found = False
            conveyor_belt.turnOn()
            cv2.waitKey(1)
            if key == 27:  # Esc key
                client.set_learning_mode(True)
                print("Number of key images: ", len(frame))
                quit()
            continue

        new_area = utils.get_area(rect)
        new_slope = utils.get_slope(rect)
        # centre and angle of rotation of contour
        #centre = rect[0]
        angle = utils.get_angle(rect)

        center_tendency = utils.check_center_tendency(rect)
        if(center_tendency):
            print("Center Object")
            if ((new_area != prev_area) or (new_slope != prev_slope)):
                print("Area or Slope condition satisfied")
                conveyor_belt.turnOff()

                # relocalise the object
                a, frame = utils.take_img(client)
                if (frame is None):
                    continue
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                shape = frame.shape
                # calculate mask for frame (black and white image)
                mask = utils.objs_mask(frame)
                #cv2.imshow('mask', mask)
                maskCopy = mask.copy()
                maskCopy = cv2.rotate(maskCopy, cv2.ROTATE_90_COUNTERCLOCKWISE)
                Display3x3("mask", maskCopy, 1)

                frame_copy = frame.copy()
                # drawing region of interest on the color image
                cv2.line(frame_copy, (0, 220), (200, 220),
                         (255, 0, 0), thickness_small)
                cv2.line(frame_copy, (0, 525), (200, 525),
                         (0, 0, 255), thickness_small)
                #cv2.imshow("Robot Camera frame", frame_copy)
                # detect objects using contours and draw box around them
                obj_found = True

                bounding_box, rect, centre = utils.bounding_box(frame, mask)
                new_area = utils.get_area(rect)
                new_slope = utils.get_slope(rect)

                # use realsense data to get object distance
                distance = depth_calculate.distance()
                print("Centre of contour is", centre)
                print("Distance to object is: {} m from the camera".format(distance))

                print("Area of Detected object: ", new_area)
                print("Slope of Detected object: ", new_slope)
                # height = distance of camera to workspace - distance of camera to object top
                height = 0.495 - distance
                print("Height of object is : {} m".format(height))
                prev_area = new_area
                prev_slope = new_slope
                if obj_found:
                    key_box.append(bounding_box)
                    key_frame.append((frame))
                    obj_found, obj = client.get_target_pose_from_rel(
                        "workspace", height,  centre[0]/shape[1], centre[1]/shape[0], 0)
                    print("Relative positions are {},{}".format(
                        centre[0]/shape[1], (centre[1]/shape[0])))
                    client.pick_from_pose(*obj.to_list())
                    print("Object Pick Up Pose")
                    print(obj)
                    client.place_from_pose(*drop_pose1.to_list())
                client.move_pose(*observation_pose.to_list())
                cv2.destroyAllWindows()
        cv2.waitKey(1)
        if key == 27:  # Esc key
            client.set_learning_mode(True)
            print("Number of key images: ", len(frame))
            quit()
