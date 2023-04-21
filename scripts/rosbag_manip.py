#!/usr/bin/env python

import rosbag
from sensor_msgs.msg import Image
import numpy as np
import cv2
import rospy

bag_file = "2023-04-06-16-03-27.bag"

# Open the bag file
bag = rosbag.Bag(bag_file)
out_bag = rosbag.Bag("new_bag.bag", 'w')

# Loop through the bag file and extract each message of type "sensor_msgs/Image"
for topic, msg, t in bag.read_messages(topics=["/hololens/RIGHT_RIGHT/image"]):

    # Convert the image message to a OpenCV image
    #cv_image = cv2.imdecode(np.fromstring(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    # Do some manipulation with the image
    # For example, resize the image
    #resized_image = cv2.resize(cv_image, (640, 480))

    # Convert the OpenCV image back to a ROS image message
    new_image_msg = Image()
    new_image_msg = msg
    new_image_msg.step = 480
    
    # Write the modified image message back to the bag file
    out_bag.write(topic, new_image_msg, t)

# Close the bag file
bag.close()
out_bag.close()