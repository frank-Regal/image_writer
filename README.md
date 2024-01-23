# Image Writer
OpenCV-based ROS package to create video files from raw sensor image messages.

## Getting Started
### Install Denpendencies
1. [ROS Melodic](https://wiki.ros.org/melodic/Installation)
2. [cv-bridge](https://wiki.ros.org/cv_bridge)
```
sudo apt install ros-melodic-cv-bridge
```
3. [Open-CV](https://opencv.org/)
```
pip3 install opencv-python3
```

### Install Package
1. Clone to a caktin workspace
```
cd catkin_ws/src && git clone git@github.com:frank-Regal/image_writer.git
```
2. Build workspace
```
cd catkin_ws && catkin build
```
3. Source
```
source catkin_ws/devel/setup.bash
```

### Run Package
1. Edit params in ```launch/save_raw_image_stream.launch``` (one stream) *or* ```launch/save_multi_raw_image_stream.launch``` (multiple streams)
2. Launch file you edited above (i.e. ```roslaunch image_writer save_multi_raw_image_stream.launch```.
3. Publish ```sensor_msgs/Image``` msg on topic configured in step one.
4. Publish ```std_msgs/Empty``` msg on topic configured in step one to stop and reset the recorder. Then proceed to stream on the image topic again. The node is setup to create videos multiple times without needing to startup and shutdown
