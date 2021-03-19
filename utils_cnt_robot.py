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


# rotate a numpy img
def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(
        image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result


class CameraObject(object):
    def __init__(self, img, x=None, y=None, angle=None, cnt=None, box=None, square=None):
        self.img = img
        self.angle = angle
        self.x = x
        self.y = y
        self.cnt = cnt
        self.box = box
        self.square = square
        self.type = None

# get bounding box of object
def bounding_box(frame, mask):
    # box co-ordinates : x1,y1,x2,y2
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

# take an img and a mask / return an array of CameraObject
def extract_objs(img, mask):
    cnts, hierarchy = cv2.findContours(
        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]

    objs = []
    initial_shape = img.shape
    selected_cnts = []
    
    if cnts is not None:
        for cnt in cnts:
            bigrect= cv2.boundingRect(cnt)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            y_values = [box[0][1],box[1][1],box[2][1],box[3][1]]
            #selecting contours in region of interest
            if min(y_values) > 220 and max(y_values)<520:
                selected_cnts.append(cnt)
        #cnt = max(selected_cnts,key = cv2.contourArea)

    # for all the contour in the image, copy the corresponding object
    if cnt is not None:
        cx, cy = get_contour_barycenter(cnt)
        try:
            angle = get_contour_angle(cnt)
        except:
            angle = 0

        # get the minimal Area Rectangle around the contour
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)
        box = np.int0(box)

        up = int(box[0][1])
        down = int(box[2][1])
        left = int(box[1][0])
        right = int(box[3][0])

        size_x = right - left
        size_y = up - down

        # # verify that our objects is not just a point or a line
        # if size_x <= 0 or size_y <= 0:
        #     continue

        # transform our rectangle into a square
        if size_x > size_y:
            down -= int((size_x - size_y) / 2)
            size = size_x
        else:
            left -= int((size_y - size_x) / 2)
            size = size_y

        # if the square is to small, skip it
        # if size < 64:
        #     continue

        square = [[down, left], [left + size, down + size]]

        # copy the pixels of our rectangle in a new image
        down += initial_shape[0]
        left += initial_shape[1]
        img_cut = np.zeros((size, size, 3), np.uint8)
        img_cut[:, :] = img[down: down + size, left: left + size]

        # rotate the image so the object is in a vertical orientation
        img_cut = rotate_image(img_cut, angle * 180 / math.pi)

        # append the data and the image of our object
        objs.append(CameraObject(img_cut, cx, cy, angle, cnt, box, square))

        for i in range(len(objs)):
            print(len(objs))
            print("Countour centre: x,y", objs[i].x, objs[i].y)
        return objs[i]