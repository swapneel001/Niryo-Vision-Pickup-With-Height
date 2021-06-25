import cv2
import pyrealsense2 as rs
import numpy as np

# set up camera
pipe = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth,1280,720,rs.format.z16,30)
config.enable_stream(rs.stream.color,1280,720,rs.format.bgr8,30)
# config.enable_device_from_file(
#     "test.bag")

# start reading frames
# skip first 10 frames to allow auto exposure to adjust
profile = pipe.start(config)
for x in range(10):
    try:
        pipe.wait_for_frames()
    except:
        x = x-1

while True:
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
    print("Image shape", depth_image.shape)
    distance = depth_frame.get_distance(600,500)
    array = np.empty([720,1280])
    for i in range(720):
        for j in range(1280):
            array[i][j] = depth_frame.get_distance(j,i)
    print("array shape : ", array.shape)
    print("distance: ",distance)
    #cv2.imshow("depth image",depth_image)
    cv2.waitKey(1)
    break
print("Test distance at a point", array[500][600])
