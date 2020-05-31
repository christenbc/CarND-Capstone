# CarND-Capstone-Project
**Udacity Self-Driving Car Engineer Nanodegree** 

This project is the capstone project of final term of the Udacity Self-Driving Car Engineer Nanodegree. Applying ROS nodes to implement waypoint tracking, control and traffic light detection/classification, the final code will be implemented on a simulator. The car will be driven around a test track. 

[![Test video](/corona%20coders%20car.gif)](/corona%20coders%20car.mp4)

 ### Team: The Corona Coders

**Members:**
Full Name | E-mail|Student 
-----| ----- | ----- 
Christen Blom-Dahl (Team Lead)| chris10bdc@gmail.com | Christen B.
Yi Zhang | aaron6yi@gmail.com | Yi Z
Yi Lu | chrisyilu@outlook.com | Yi L
Ahmed Zahran | amzahran.guc@gmail.com | Ahmed Z.

### How to run the simulation
1. Navigate to `CarND-Capstone` folder 
2. Run the commands below in the terminal
```
pip install --upgrade pip
pip install -r requirements.txt
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
3. After launching the project, click "Go To Desktop" to open the workspace desktop.

4. Double click on the “Capstone Simulator” file available on the desktop. This will start the simulator.


 ### System Architecture

The following diagram shows the communication between the ROS nodes used in this project. ![img](https://lh5.googleusercontent.com/w9lr97P18pnYZ97xE6uR8EheNBfGv7L3HtosXvpCKGZDWhkkf6WvWsqkdeEubTUJG0CJLR18qDetgE7RyExs7M3RIaL6tSt9J9GVTDwqglTAhciHzhk4sm5Kx6vJ4Qgo98XUCZk0)**Waypoint Updater**

Waypoint Updater node is one of components in the path planning subsystem, which has been implemented in this project. Additionally, the Waypoint Loader node is already provided.

**Description**

The Waypoint Updater node aims to update the target velocity according to the traffic light and obstacle detection data. A waypoint has a data structure that consists of target position coordinates along the driving track. The car can then plan its trajectory based on these waypoints.

**Inputs and Outputs**![img](https://lh5.googleusercontent.com/r1Zug8zZGzMiuVacHmumuNzau32Cp00hIBCdKT0AOyRw4_r88OvLfMApTpsz1W9qQ3xNUM4iSJ-CXTtKYQbqC_Fb5mYe8yX5FpcH_jHkReBDyOQLOXAOrdatTNzR5laENpPbOyPU)

The Waypoint Updater node has following topics as inputs:·    

```
/base_waypoints
```

Base waypoints contain all the waypoints along the driving track, which are published to the car only once at the beginning. With base waypoints, the car can then figure out the target waypoints ahead of its current position. Waypoint Updater gets the base waypoints from the Waypoint Loader node.

```
 /traffic_waypointTraffic 
```

Waypoint gives the location, where the car should stop. This waypoint is published by the Traffic Light Detection node and received by the Waypoint Updater node. Also, the Waypoint Updater node calculates the target velocities for the traffic waypoint, so that the car can decelerate or stop at the traffic waypoint as smoothly as possible.·    

```
/current_pose
```

This topic reports the current position of the car and is published by the simulator. Waypoint Updater uses this information to estimate the car's current position with respect to the base waypoints.·    

```
/obstacle_waypoints
```

Waypoint updater subscribes to this waypoint from the Obstacle Detection node and then gives the obstacle positions respect to their waypoint positions.

In this project, this information is not available, since the testing is performed in an environment, where there are no obstacles.The final way point will be published as output from the Waypoint Updater node. 

The final way point gives a fixed number of waypoints ahead of the car under consideration of the target velocities depending on traffic lights. How many waypoints to publish ahead of a car's current position depends on various factors, e.g. driving speed of the car as well as the computational power of the single processing.

In the testing environment provided by Udacity's VM, we determined that 100 points led to an acceptable performance.

**Implementation**

Waypoint Updater node subscribe the following topics:

```
rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
```

Then waypoints will be updated with respected to the traffic lights and waypoints will be published using

```self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)```

Finally, the target waypoints will be generated related to base waypoints in the ```publish_waypoints``` method, where the closest waypoints are required to publish the final waypoints. Each waypoint is composed by pose and twist (linear & angular velocities) data which can be seen in Waypoint.msg:
* geometry_msgs/PoseStamped pose
* geometry_msgs/TwistStamped twist

The target velocities are initially provided by the Waypoint Uploader node and modified accordingly by Waypoint Updater node.

### DBW Node

DBW node is one of components in the control module, which has been implemented in this project. Additionally, the Waypoint Follower node is already provided.

**Description**

The car is under control of the Drive By Wire system, which takes responsibility for controlling throttle, brake and steering. DBW node subscribes target linear and angular velocities and then publishes throttle, brake, and steering commands to respective topics.In addition, DBW node outputs can also be turned off and the control can be transferred to the driver using a dedicated flag, i.a. ```dbw_enabled```.
**Inputs and outputs**

The DBW node subscribes to `/twist_cmd` message which provides the target linear and angular velocities.The inputs to the DBW node are following topics:![img](https://lh4.googleusercontent.com/IjfsgdxL6go2L46gj6kP02yi7aUXfX5nQ8vsNcCzUITcC0oZeTMemvyffSq8-vpMDDhiAwKFW0yPDw6MXmI-zki_2q55FzguU3y3jRNHivTb7zWAhVS2rfOMa0aLrnhFoGJQtxt-)

```
/twist_cmd
```

The twist command is published by the Waypoint Follower node. DBW node receives this topic and then generates the required output in terms of throttle, brake and steering commands.

```
/current_velocity
```

In the simulation environment, this topic is given by the simulator and serves as an input for the DBW node to determine the linear velocity, which is required by the controller.

```
/vehicle/dbw_enabled
```

This boolean topic describes the availability of the DBW system that is also published by the simulator. According to this topic, the DBW node will enable or disable the brake, throttle and steering.

As outputs, the throttle_cmd, brake_cmd and steering_cmd topics will be sended to the simulator. The outputs will be processed by a control system composed of 

**Yaw controller** 

This controller converts the target linear and angular velocities into a steering angle with respect to the car’s steering ratio and wheelbase length. To ensure the car drives smoothly as possible, the maximum linear and angular acceleration rates are bounded. 

**Throttle controller** 

This controller is a simple PID controller that adjusts the throttle based on comparing the current velocity with the target velocity. To avoid oscillation, the throttle gains were tuned by using trial and error for allowed acceleration.

**Brake controller**

This controller determines the deceleration force with respect to the car mass, the wheel radius, as well as the brake_deadband. 

**Implementation**

DBW node subscribes to the required topics, i.e. 

```twist_cmd``` and ```current_velocity```,  ```/vehicle/dbw_enabled``` from Waypoint Follower node and simulator, respectively:

```
rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)

rospy.Subscriber('/twist_cmd',TwistStamped, self.twist_cb)

rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb)
```

In the DBW node, the necessary motion parameters, e.g. linear and angular velocity, will be extracted from the twist command and will be published to respective topic by using

```
self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',SteeringCmd, queue_size=1)

self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',ThrottleCmd, queue_size=1)

self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',BrakeCmd, queue_size=1)
```
**Important note:**
For the simulation case scenario, if there is problem with synchronization with the system integration, try to set DBW node publishing rate to 30 Hz.

### Traffic Light Detection and Classification
The perception module consists of Obstacle Detection and Traffic Light Detection node. In this project, the latter was only taken into account.

**Description**

The Traffic Light Detection node receives data from the topics called ```/image_color```, ```/current_pose```, and ```/base_waypoints``` and publishes the ```/traffic_waypoint``` topic, which tells the locations to stop for red traffic lights. Additionally , the Waypoint Updater node takes this information as input to determine motion velocity for given waypoints.**![img](https://lh4.googleusercontent.com/v10X1nsiepiODh-x3WtmqpGNJJP7YBp8zoFmWjuQDCqGcu1V5XfZS-cDnkK7zN0la08hnz4KPl0Eh2Mh_KPXaQY5TQSzShKviPVWfuVLePXDJQ5yhIdrCgLVe-pJ62SKa84QsJ_5)**

The Traffic Light Classifier is implemented as an independent component in the Traffic light Detection node, which is responsible to publish information regarding where the car should come to a stop.
**Inputs and outputs**

The inputs for Traffic Light Detection node consists of 

```
/base_waypoints
```

This is the same list of waypoints as the one used by Waypoint Updater node and is published only once at initialization.

```
/image_color
```

This input provides an image stream captured by the car's front camera. The traffic light classifier uses these images to predicate the status of the traffic light in terms of the color.

```
/current_pose
```

The Traffic Light Detection node receives this message from the simulator to determine the current position of the car.

The Traffic Light Detection node will publish the ```/traffic_waypoint``` as its output. This topic contains the index of the waypoint for the nearest upcoming red light's stop line, which is viewed as an input for the Waypoint Updater node.
**Implementation**

The Traffic Light Detection node has following missions: 

Find the nearest traffic light ahead of the car by using the car's location and the ```(x, y)``` coordinates for traffic lights. This will be processed in the ```process_traffic_lights``` method defined in the ```tl_detector.py``` file. Additionally, the ```get_closest_waypoint``` method is applied to find the closest waypoints to the vehicle and lights. The indices of these waypoints are useful to determine which light is ahead of the car.

2. Use the camera image data to classify the color of the traffic light. The core functionality of this step takes place in the ```get_light_state``` method of ```tl_detector.py```

### Traffic Light detection with SSD detector

 The traffic light classification model is based on the pre-trained on the COCO dataset model "ssd_mobilenet_V2_coco" from [Tensorflow detection model zoo] (https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md). For this project, a simulator data based model was trained by using the [Tensorflow Object Detection API]

(https://github.com/tensorflow/models/tree/master/research/object_detection).
The models are available at `CarND-Capstone/ros/src/tl_detector/light_classification/site_classifier_model` and the simulator dataset was from `CarND-Capstone/ros/src/tl_detector/light_classification/sim_classifier_model`.
The classification output has four categories: Red, Green, Yellow and off. To simplify, the final output will be Red or Non-Red, that is only the Red will be classified as `TrafficLight.RED`, and the other cases will be classified as `TrafficLight.GREEN`.

We really appreciate the training and testing datasets openly provided by https://github.com/marcomarasca/SDCND-Traffic-Light-Detection. They are composed of manually annotated images from the simulator and from the real case scenario which is provided as a video contained in an udacity rosbag.

### Original Setup and Installation Instructions

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction here.
Please use one of the two installation options, either native or docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).

* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:

  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS

  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.

* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)

  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)

* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation

[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container

```bash
docker build . -t capstone
```

Run the docker file

```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding

To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository

```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies

```bash
cd CarND-Capstone
pip install -r requirements.txt
```

3. Make and run styx

```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```

4. Run the simulator

### Real world testing

1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file

```bash
unzip traffic_light_bag_file.zip
```

3. Play the bag file

```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```

4. Launch your project in site mode

```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

5. Confirm that traffic light detection works on real life images
