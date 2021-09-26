# A vision based pick and place application for the Niryo One Robot utilising an Intel RealSense Depth Camera

This repository contains the code that can be be used with a Niryo One Robot to pick up objects placed in the Niryo one Workspace, by estimating their height using the Intel RealSense depth camera, and object contouring to localise the objects in the Niryo Vision Workspace

> Code was tested with Niryo One, Intel Realsense D415, and the niryo vision workspace
> 
> Localisation for the camera, workspace and robot has to be adjusted in the code based on the user's setup
> 
> Denkovi Drivers are used to connect to the conveyor belt shown in the image, for a non conveyor belt application please remove the conveyor belt code.
> 
> Workspace dimensions can be modified based on user's operation. For this operation, the dimensions were fit to the conveyor belt used
>
> The Niryo One TCP client api is utilized to send commands to the robot. 
>
> This code was run on Windows, and uses Python3. It can work on any OS that supports Denkovi Drivers, or on any OS regardless if conveyor belt isn't used.

### Objective of this code:
To scan objects passing through on a conveyor belt, and stop the conveyor belt to allow the robot to pick them up, and then restart the conveyor belt.
At the current state, the camera can only process one object in the frame at a time, however further work has been done to deal with multiple objects, but that code is intellectual property of Robert Bosch SEA. 

## Summary of files in this repository: 

1. Object_Detection_Robot_contours.py : Used to connect to the robot, read RGB camera data to locate centre of the object, call supporting programs, and send robot                                         move commands to the Niryo One
2. conveyor_belt.py : used to turn conveyor belt on and off
3. depth_calcualte.py : used to read Intel RealSense data and calculate height at the centre of the object
4. utils_cnt_robot.py : contains image processing functions for the robot

### Mission Launch Sequence:

1. Turn the Niryo One on, and set it to hotspot mode
2. Connect to the robot wifi. 
3. Check the connection through Niryo One Studio
4. Connect the conveyor belt driver, and Intel RealSense camera through USB Ports
5. Run ``` python3 Object_Detection_Robot_contours.py```

### An image of the robot and the conveyor belt and a sample object:
![niryo_one_workspace](https://user-images.githubusercontent.com/57823333/134794157-3d768b3c-219e-4e04-be94-ac91205e2fb0.png)
