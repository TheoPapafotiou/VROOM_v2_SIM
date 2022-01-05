# BFMC Simulator Project

The project contains the entire Gazebo simulator developed by the organizing team as well as a ROS nod network example for gathering data from the simulator and controlling the car. 

By following the guide below you can get started with the simulator and start developing your project!


## 1. Installing the tools
- Install [Ubuntu 20.04.2.0 LTS (Focal Fossa) ](https://releases.ubuntu.com/20.04/) - Desktop Image. For best performances, avoid using a virtual machine.
- Install [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu) - Desktop-Full Install.
     

## 2. Cloning the repository

Clone the repository inside the `Documents` folder (or perform a commit on the repository)


## 3. setup of the repository

- Open a terminal in `~/Documents/Simulator` directory and run the following command:

```sh
catkin_make --pkg utils
catkin_make
```

- Two folders – `devel` and `build` – will be generated inside the `repository` directory. 

- Gazebo needs to know where the workspace's packages and models are, so you will need to add them to the gazebo environment variable list. The easiest and cleanest way is to add them to the source file of the simulation, so it won't interfear with other projects:

```sh
gedit devel/setup.bash
```

and add those two lines at the begining of the file (don't forget to change {YOUR_USER} with your actual user name).

```sh
export GAZEBO_MODEL_PATH="/home/{YOUR_USER}/Documents/Simulator/src/models_pkg:$GAZEBO_MODEL_PATH"
export ROS_PACKAGE_PATH="/home/{YOUR_USER}/Documents/Simulator/src:$ROS_PACKAGE_PATH"
```

## 4. Running of the simulation

For each terminal where you plan to use the ROS, you need to source the repository itself:

```sh
source devel/setup.bash
```
- Finally, run the following command in order to start the entire simulation (the simulaiton is heavy, it may take many resources):

```sh
roslaunch sim_pkg map_with_all_objects.launch
```


## 5. Interaction with the simulation

The simulation can interact directly with the car or it can be used as a stand-alone environment on your PC.


### 5.1. Interaction with the rcCar

Both the car and the PC running the simulation have to be on the same network and the ssh communication has to be enabled on the RPi.
In order for the nodes to be able to communicate between them, some environmental variables have to be set. A script was created for this purpose:
On the RPI, you have to run the script as follows:
```sh
source src/utils/network.sh PC_IP
```
On the PC running the simulation, you have to run the script as follows:
```sh
source src/utils/network.sh
```
This setup will make the nodes on your car interact with the roscore from the PC, and you will be able to see the entire setup of the network. 
The file have to be soruced for each terminal. 
The core can be swapped to run on the phisical car as well.

Run the simulation on the PC:
```sh
roslaunch sim_pkg map_with_car.launch
```
Run the control nodes of the car on the RPI:
```sh
roslaunch utils start_car_virtual.launch
```

Now the simulator will publish some info on the topics and you can subscribe to them from your car (automobile/image_raw, automobile/localisation, automobile/IMU, automobile/feedback, automobile/semaphores/*)

Now the simulator will subscribe to some info on the topics and you can publish on them from your car (automobile/command)

### 5.2. Interaction with the PC

To check an example of publishing & subscribing, after the simulaiton was started, you can open two other terminals and source the devel/setup.bash inside.
on one terminal, execute
```sh
rosrun example camera.py
```
and on another one you can execute
```sh
rosrun example controlRemote.py
```

### 5.3 Check ROSTOPICS

- If you want to check all the published topics, you need to run:
```sh
rostopic list
```
- If you want to check what each topic publishes, you need to run:
```sh
rostopic echo <topic>
```
- If you want to find the `msg type` of each topic, you need to run:
```sh
rostopic type <topic> | rosmsg show
```

If the setup for interacting with the car is done, the physical car will also interact with the two scripts, as well as the virtual one. 
We suggest, if you plan to develop on the PC, to create another workspace for the project itself. 

## 7. Interaction only with the PC

Exactly as the examples above, you can open one terminal (after opening the simulation) and run inside:
```sh
source devel/setup.bash
rosrun example controlAutonomous.py
```

This script has been created to help you test initially the function you have created without the need of the actual car. 

You only need to:
- Call the function from your class inside the `_test_function()`
- Update the `self.speed` and `self.angle`

And the you can see the car moving on the track! 

### Use sensor metrics

All sensor metrics are initiated inside the `__init__()`. You only need to use the corresponding parameter:
- Ranging Sensors ---> `self.<ray_name>.range`
- Cameras --> `self.<cam_name>.cv_image` 

### Stop the Car & The process

In order to stop the car and the process you need to do:
- Press `q` on the Frame Preview
- `CTRL+C` to stop the process

For now, you must use __both commands__ to stop the simulation properly!! 

## 8. Other tips

- If you want to start the testing from scratch, instead of restarting gazebo, you can only restart the simulation with the help of the ROS services:
```sh
$ rosservice call /gazebo/reset_simulatio
```

- Deactivate GUI from the launch in order to save some computational power;

- Play with rqt tool in order to check the images and other topic messages, frequency, etc. 

- You can change the resolution of the map from `src/models_pkg/track/materials/scripts/bfmc_track.material`. The available maps are under the textures directory. 
Bigger the resolution, bigger the requirements of the simulation.

- Inside the `src/sim_pkg/launch/sublaunchers` directory, there is a launch file for each type of elements on the map or groups of elements. You can also spawn some of them separately while the simulator is running, based on the requirements at the moment. 

You can also configure your own launching files, you can create a launch file that includes multiple launch files. See the `map_with_all_objects.launch` file inside the `src/sim_pkg/launch` as an example.

In order to use the launch files, you need to source the devel/setup.bash

# Hardware Setup

In order to test properly the modifications made on the car, it is needed to check the performance of the sensors under different **positions** and **fields of view**.
- Angles in ***rads***
- Distances in ***m***
- For constants of double data type, "." (dot) is used as decimal seperator
- After each change in `<something>.sdf` files, you need to kill and reload the simulation.
- The X, Y, Z are calculated **relatively to the center of car**, since all sensors are connected in the `rcCar_assembly`.
- [The SDF format specification](http://sdformat.org/spec?ver=1.6&elem=model#include_pose) FYI

## Depth Camera

### Change Resolution/FOV

Inside `src/models_pkg/depth_camera/model.sdf`, you will find the following tags:
- `<width>`
- `<height>`
- `<horizontal_fov>`
- `<near>`, `<far>` (MIN/MAX range)

Modifying them will allow you to change the camera resolution and/or FOV.

### Change Position

Inside `src/models_pkg/rcCar_assembly/model.sdf`, you will find the inclusion of `<uri>model://depth_camera</uri>`. Inside that same `<include>` tag, there is also a `<pose>` tag, which needs to be modified in order to change the camera's position.  

```
<pose> X Y Z ROLL PITCH YAW </pose> 
```

### Sensor Topics 
- `/automobile/color_image_raw`
- `/automobile/depth_image_raw`
- `/automobile/color_camera_info`
- `/automobile/depth_camera_info`

## Ranging Sensors (IR)

For each IR sensor around the car, a different model has been created. The following instructions are the same for each one of the models. 

### Change FOV/Noise

Inside `src/models_pkg/ray_<orientation>/model.sdf`, you will find the following tags:
- `<horizontal>`
    - `<min_angle>`
    - `<max_angle>` 
- `<vertical>`
    - `<min_angle>`
    - `<max_angle>`
- `<range>`
    - `<min>`
    - `<max>`
- `<plugin>`
    - `<gaussianNoise>` (min == 0)
    - `<fov>` (= 2 * horizontal_max_angle)

Modifying them will allow you to change the sensor FOV and/or the Noise. 

*You may notice that the `min_angle` should be the negative of the `max_angle`.*

### Change Position

Inside `src/models_pkg/rcCar_assembly/model.sdf`, you will find the inclusion of `<uri>model://ray_<orientation></uri>`. Inside that same `<include>` tag, there is also a `<pose>` tag, which needs to be modified in order to change the camera's position.  

```
<pose> X Y Z ROLL PITCH YAW </pose> 
```

### Sensor Topics 
- `ir_<orientation>`

## Lane Camera

### Change Resolution/FOV/Noise

Inside `src/models_pkg/camera/model.sdf`, you will find the following tags:
- `<width>`
- `<height>`
- `<horizontal_fov>`
- `<near>`, `<far>` (MIN/MAX range)
- `<noise>`
    - `<stddev>`

Modifying them will allow you to change the camera **resolution** and/or **FOV** and/or **Noise**.

### Change Position

Inside `src/models_pkg/rcCar_assembly/model.sdf`, you will find the inclusion of `<uri>model://camera</uri>`. Inside that same `<include>` tag, there is also a `<pose>` tag, which needs to be modified in order to change the camera's position.  

```
<pose> X Y Z ROLL PITCH YAW </pose> 
```

### Sensor Topics 
- `/automobile/lane_image_raw`
- `/automobile/lane_camera_info`

# Subscribe to any sensor topics

In order to use the sensor metrics (Images and ranges) in the algorithms, you need to subscribe in the corresponding topics of each sensor:

## In case of a Camera Sensor topic (Image)
```sh

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class SensorHandler:

    def __init__(self):
        self.bridge = CvBridge()
        self.cv_image = np.zeros((<width>, <height>))
        rospy.init_node('SensorNOD', anonymous=True)
        self.sensor_sub = rospy.Subscriber("<Sensor Topic>", Image, self.callback)

    def callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        cv2.imshow("Frame preview", self.cv_image)

if __name__ == '__main__':
    try:
        nod = SensorHandler()
    except rospy.ROSInterruptException:
        pass
```

## In case of a Ranging Sensor topic (Range)
```sh

import rospy
from sensor_msgs.msg import Range

class SensorHandler:

    def __init__(self):
        self.range = 0.0
        rospy.init_node('SensorNOD', anonymous=True)
        self.sensor_sub = rospy.Subscriber("<Sensor Topic>", Range, self.callback)

    def callback(self, data):
        self.range = round(data.range, 4)
        print("Distance: ", self.range)

if __name__ == '__main__':
    try:
        nod = SensorHandler()
    except rospy.ROSInterruptException:
        pass
```