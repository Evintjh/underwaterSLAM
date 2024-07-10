# Sensor overview

Hardware needed:
- Occulus M750d imaging sonar
- DVL
- IMU
- pressure sensor


# Python Dependencies, note python-3

```
cv_bridge
gtsam
matplotlib
message_filters
numpy
opencv_python
rosbag
rospy
scipy
scikit_learn
sensor_msgs
Shapely
tf
tqdm
pyyaml
```

# ROS Dependencies
```
ROS-noetic
catkin-pybind11
catkin-tools
```

# Installation
- Ensure all python dependencies are installed
- Check ros distro
- clone this repo into your catkin workspace
- clone git clone https://github.com/ethz-asl/libnabo.git into your catkin workspace
- clone https://github.com/ethz-asl/libpointmatcher.git into your catkin workspace
- Make sure to use the specific tested version of libpointmatcher by using the following command in the libpointmatcher folder
```
git checkout d478ef2eb33894d5f1fe84d8c62cec2fc6da818f
```
- clone https://github.com/jake3991/Argonaut.git](https://github.com/Evintjh/underwaterSLAM.git into your catkin workspace
- build your workspace with catkin build NOT catkin_make

# Sample data
A rosbag data file is provided to test and run the SLAM system. Available here: https://drive.google.com/file/d/1nmiFfyk8mVssLqgac7BOe4_RPBP6Wnc9/view?usp=sharing

# Running "Online"
This will launch the SLAM system, then we will playback the data as if it is happening now. 
- source catkin_ws/devel/setup.bash
- roslaunch bruce_slam slam.launch
- rosbag play your_data.bag

# Running Offline
This runs our offline mode, great for quick testing/tuning of parameters. 
- source catkin_ws/devel/setup.bash
- roslaunch bruce_slam slam.launch file:=path_to_data/your_data.bag

# Configuration
This SLAM system has many parameters, please read the wiki for an explanation of each parameter. However, we highly recommend using the default parameters in the config folder. If you are to tune anything it would be the feature extraction node in feature.yaml. 

# IMPORTANT
- For the SLAM system to work in real world or your personal simulation, you would require a base_link. This requires a urdf of the bot to be loaded with SLAM.
- A simple urdf file for turtleboi bot is provided under: bruce_slam/urdf/turtleboi.urdf
- Customise the urdf file to your own needs
- in slam.launch, these lines have been added to initialise the urdf:
```
<param name="robot_description" command="$(find xacro)/xacro --inorder '$(find bruce_slam)/urdf/turtleboi.urdf'" />

<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="True"/>
</node>

<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
```

# Citation
```
@inproceedings{
  title={Virtual Maps for Autonomous Exploration of Cluttered Underwater Environments},
  author={Jinkun Wang, Fanfei Chen, Yewei Huang, John McConnell, Tixiao Shan, and Brendan Englot},
  booktitle={IEEE Journal of Oceanic Engineering,
  year={2022},
  organization={IEEE}
}
```






