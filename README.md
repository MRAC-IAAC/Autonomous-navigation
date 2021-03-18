## Autonomous-navigation
To explore more potentials of ROS and how ROS can play important role in converting  a normal drone to a mapping drone. the same is applicable to UGV and other robots. ORB_SLAM was used to generate point clouds  in real time using a monocular camera. This has capability of loop closure, map saving and using same map to improve the quality of point cloud/model. The final goal of the project is to create a autnomous photogrammetry with ROS.
Please note that this still a WIP. 

![alt text](Images\main.jpg "Autonomous Navigation")

## Hardware and softwares used : PARROT AR DRONE
Parrot Bebop2 drone was used for this project along with ROS, Bebop_autonomy & ORB_SLAM2 
![alt text](Images\HardwareUsed.jpg "Hardware Used")

## Software Diagram and algorithm
The process involves using ORB_SLAM2 with ROS in order to create the point clouds in real time. We created a python script which reads the point clouds in real time. 
ROS is connected with grasshopper and the python scripts, which can subscribe to the topics published by python scripts and ROS/ORB_SLAM2. As a reault of which we can visualize the point cloud in grasshopper in real time. 
A small part of the area is manually scanned and trajectory is generated using grasshopper, so that the process can start. Once the drone takes off on the trajectory, the algorithm can process on itself in real time and the trajectory keep on generating automatically(ROS, ORB_SLAM2 & Grasshopper communicated continuously)

![alt text](Images\Softwareprocess.png "Software Proces")

### How does the point cloud grabber works? 
There are 2 python scripts, one reads the point clouds in real time and saves them to a csv files. Another scripts can read the point cloud data saved by the first scripts in order to visualize them in 3d

![alt text](Images\Softwareprocess2.jpg "Software Proces")

### The first Manual point cloud extraction process? 
rosbag is saved from ROS+bebop2 in the computer and point clouds are extracted using the process mentioned above(using the point grabber scripts)

![alt text](Images\orbpoints.jpg "Software Proces")

Further to above the point clouds are extracted in grasshopper and cleaned with the target area where the trajectory has to be initiated. 
The trajectory is then generataed using the points cloud locations (reference image below)

![alt text](Images\orbpoints3.jpg "Software Proces")

Once the trajectory is genrated, the drone can be connected through ROS and grasshopper and the projeccs is ready to be initiated. 

Following steps to be followed:
Note: you need to have 2 pc or a virtual machine to run ubuntu (ROS) and grasshopper in windows. 
1. Connect the bebop to laptop wifi.
2. Start the Bebop_driver to connect the bebop to ROS.
3. Initiate the ORB_SLAM2_ROS
4. Initiate the pointgrabber.py, visualizeptcloud.py & pointcloudcompiler.py in seperate terminals
5. Initiate the bebop_position_controller.py. This files has multiple publishers and subscribers which sends/receive info such as position of bebop to ROS, point clouds to ROS, etc. 
6. Run the ROS bridge in ubuntu PC
7. start grasshopper and connect it to ROS pc using the websocket component from Bengesht plugin (https://www.food4rhino.com/app/bengesht). follow the rosbridge and bengesht guide from the link provided. 
8. when you are ready with all the setup. select the bebop_position_controller.py and you can take off with "T". once the drone takes off it can be set to navigation mode using "N". when navigation mode is activated, the drone will start the trajectory and will return to its starts positiion. 
9. to inturrupt the process at anytime, you can press "L" to land. 


