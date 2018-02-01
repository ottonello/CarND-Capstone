This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Examples

Simulator run:
https://www.youtube.com/watch?v=0fd9f3lCNRU

Traffic Light detection samples(parking lot data):
https://www.youtube.com/watch?v=ZhWebjFoCd8
https://www.youtube.com/watch?v=PV1dIz3q17M

### Waypoint updates

The code for waypoint updates is quite simple but it works fine in general, the closest waypoint ahead(within 45 degrees) of the car and the next 200 waypoints are published to the `final_waypoints` topic to be read by the waypoint follower.

Some more logic is used for braking when a red light is detected, simple math is applied to determine the target velocity for each waypoint starting from the final one with speed=0 and using the target acceleration limit. Waypoint speed is updated and then published as usual.

### Drive by Wire control

Drive by wire code is also very simple, in the DBW node the proposed and current velocities are extracted from the received messages and then passed to the twist controller which returns the car commands to publish.

The Twist controller handles receiving target parameters and, according to physical parameters of the car, calculates the required inputs the car should do.

Acceleration and deceleration are implemented with a PID and a Low Pass Filter. The PID is updated with the error on the linear velocity on each step, and then corrected to brake whenever throttle is negative, using the maximum brake torque as determined by the mass and wheel radius of the car.

Yaw is determined by adjusting the requested angular velocity according to the ratio of the current velocity and the target linear velocity, and then converting that value to a steering angle by using the wheel base and steer ratio of the car.

### Traffic Light Detection

Traffic Light detection is implemented using the TensorFlow Object Detection API.
Required code to train different models is under the `trafficsign_training` folder, including also
a Jupyter notebook to test inference on some samples.

For more informacion on how to train a model look at the README.md file in the same folder.

I tried several models from the model zoo but ended up using SSD inception for a balance between good detection
capabilities and processing speed.

I tested the average inference time on the simulator images, these are the results:
* ssd_mobilenet: 11.7727288136 ms
* ssd_inception: 16.8356101695 ms
* faster_rcnn: 77.3651694915 ms

The training used to generate the final inference graph consists in retraining the `ssd_inception_v2_coco_2017_11_17` pretrained model for 6000 steps using a combination of real world data and sim data. 

https://github.com/ottonello/CarND-Capstone/blob/master/trafficlight_training/models/ssd_inception-trafficsign.config

Training data was based on the ones released by Anthony Sarkis', I added some samples to the sets to improve some areas where detection wasn't as good. Here's Anthony's blog post on how he implemented detection also using the object detection API: https://codeburst.io/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58


Some SSD Inception final training examples:

![Simulator](https://raw.githubusercontent.com/ottonello/CarND-Capstone/master/trafficlight_training/inference_results_ssd_inception/4.jpg)

![Parking lot sample](https://raw.githubusercontent.com/ottonello/CarND-Capstone/master/trafficlight_training/inference_results_ssd_inception/left0000.jpg)

More examples are located under the [samples folder](https://github.com/ottonello/CarND-Capstone/tree/master/trafficlight_training/inference_results_ssd_inception)


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
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

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
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images


### Credits

I took the idea of using Tensorflow Object Detection API from the following post by Anthony Sarkis:
https://codeburst.io/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58

I also based my training dataset on his, which saved some time. I did add some more samples using Sloth.
