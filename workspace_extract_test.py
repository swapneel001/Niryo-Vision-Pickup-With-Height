import pyrealsense2 as rs
import utils_cnt
import cv2
import numpy as np
import imutils
from niryo_one_camera import *

#set up camera
pipe = rs.pipeline()
config = rs.config()
# config.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
# config.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
config.enable_device_from_file("./recording1.bag")

#start reading frames
profile = pipe.start(config)
for x in range(5):
    try:
        pipe.wait_for_frames()
    except:
        x = x-1

while True:
    #read and align colour and depth frame
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
    
    #convert frames into numpy arrays readable by opencv
    colorizer = rs.colorizer()
    depth_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
    color_image = np.asanyarray(color_frame.get_data())

    gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

    img_thresh = cv2.adaptiveThreshold(gray, maxValue=255, adaptiveMethod=cv2.ADAPTIVE_THRESH_MEAN_C,
                                       thresholdType=cv2.THRESH_BINARY, blockSize=15, C=32)
    cv2.imshow("image", img_thresh)
    

    #extract image workspace
    try:
        color_image,depth_image = extract_img_workspace(color_image,depth_image ,workspace_ratio = 0.37)
        
        print("workspace_found")
        color_image = cv2.rotate(color_image,cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_image = cv2.rotate(depth_image,cv2.ROTATE_90_COUNTERCLOCKWISE)
        depth_image_dim = depth_image.shape
        color_colormap_dim = color_image.shape
        if depth_image_dim != color_colormap_dim:
            color_image = cv2.resize(color_image, dsize=(
                depth_image_dim[1], depth_image_dim[0]), interpolation=cv2.INTER_AREA)
        color_image = utils_cnt.standardize_img(color_image)
        cv2.imshow("workspace",color_image)
    except:
        continue
    cv2.waitKey(0)