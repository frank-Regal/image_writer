# Image Writer
OpenCV-based ROS package to create video files from raw sensor image messages.


* [1. Installation](#1-installation)
* [2. Running](#2-running)

## 1. Installation
### Install Dependencies
  * <details>
    <summary><a href="https://wiki.ros.org/melodic/Installation">ROS Melodic</a></summary>
    <br>
    See link for ROS Melodic installation instructions.
    </details>

  * <details>
    <summary><a href="https://wiki.ros.org/cv_bridge">cv-bridge</a></summary>
    <br>
    
    ```shell
    sudo apt install ros-melodic-cv-bridge
    ```
    </details>

  * <details>
    <summary><a href="https://opencv.org/">open-cv</a></summary>
    <br>
    
    ```shell
    pip3 install opencv-python3
    ```
    </details>

### Clone Repo
  ```shell
  # Clone to a caktin workspace
  cd catkin_ws/src && git clone git@github.com:frank-Regal/image_writer.git
  
  # build workspace
  cd catkin_ws && catkin build
    
  # source workspace
  source catkin_ws/devel/setup.bash
  ```

## 2. Running
1. Edit Params

    ```shell
    # for one video edit the following file
    vi launch/save_raw_image_stream.launch
  
    # for multiple videos, edit the following file
    vi launch/save_multi_raw_image_stream.launch
    ```
2. Launch file edited above
   ```shell
   # example
   roslaunch image_writer save_multi_raw_image_stream.launch
   ```
3. Publish ```sensor_msgs/Image``` ROS msgs on the topic assigned to the ```topicname_image``` param in launch file.
4. Publish a ```std_msgs/Empty``` ROS msg on the topic assigned to the ```topicname_empty``` param in the launch file to stop and reset the recorder.
    > **Info:** You can proceed to stream on the image topic again without shutting down the node. The node is setup to create videos multiple times without needing to startup and shutdown
