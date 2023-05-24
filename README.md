# Image Writer
OpenCV-based ROS package to create video files from raw sensor image messages.

# Running
1. Edit params in ```params/params.yaml```.
2. Launch main node ```roslaunch image_writer save_raw_image_stream```.
3. Publish an image stream on a ```sensor_msgs/Image``` topic configured in ```params/params.yaml```.
4. If trying to record multiple videos from one single stream of images, make sure to set ```save_multi_stream_in_sequence``` param to ```true```. After each video is streamed on the Image topic message, pass a ```std_msgs/Empty``` message to stop and reset the recorder. Then proceed to stream on the Image topic again.
