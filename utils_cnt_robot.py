from niryo_one_tcp_client import *
from niryo_one_camera import *
import numpy as np
import cv2
import time

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# take an image of the workspace


def take_workspace_img(client):
    a, mtx, dist = client.get_calibration_object()
    while 1:
        a, img_compressed = client.get_img_compressed()
        img_raw = uncompress_image(img_compressed)
        img = undistort_image(img_raw, mtx, dist)
        img_work = extract_img_workspace(img, workspace_ratio=1)
        if img_work is not None:
            break
        print("take_workspace_img failed")
        return False, img
    return True, img_work

# calculate a mask
def objs_mask(img):
    boundaries = [([0, 50, 0], [100, 255, 100])]
    for (lower, upper) in boundaries:
        lower = np.array(lower, dtype="uint8")
        upper = np.array(upper, dtype="uint8")
        mask = cv2.inRange(img, lower, upper)
    thresh = cv2.threshold(
        mask, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)[1]
    return thresh

def bounding_box(frame, mask):
    """Get bounding box of object"""
    cnts, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    selected_cnts = []
    if cnts is not None:
        for cnt in cnts:
            bigrect= cv2.boundingRect(cnt)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            y_values = [box[0][1],box[1][1],box[2][1],box[3][1]]
            if min(y_values) > 220 and max(y_values)<520:
                selected_cnts.append(cnt)
        cnt = max(selected_cnts, key = cv2.contourArea)
        bigrect = cv2.boundingRect(cnt)
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame,[box],0,(0,255),2)
        cv2.imshow('Bounding Box',frame)
        return box,rect
        
    return None

def standardize_img(img):
    array_type = img.dtype

    # color balance normalizing
    color_mean = np.mean(img, axis=(0, 1))
    mean_color_mean = np.mean(color_mean)
    img = img[:][:]*mean_color_mean/color_mean

    # color range normalizing
    min, max = np.quantile(img, [0.001, 0.95])
    img = (img - min) * 256 / (max - min)
    img = np.clip(img, 0, 255)
    img = img.astype(array_type)
    return img

# Uncompress and Undistort image
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
