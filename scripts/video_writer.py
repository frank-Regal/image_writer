#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Empty
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os
import numpy as np

class VideoWriter:
    def __init__(self):
        # Initialize the node
        rospy.init_node('video_writer', anonymous=True)
        
        # Get the namespace of the current node
        self.node_ns = rospy.get_namespace()

        # Create a CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Define the video writer
        #fourcc = cv2.CV_FOURCC(*'MJPG')
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        #cv2.VideoWriter_fourcc('m','p','4','v')
        self.outvideo = cv2.VideoWriter('/project/ws_dev/src/hri_cacti_xr/image_writer/scripts/out2.avi',  
                        fourcc, 
                        10, (640, 480)) 


        self.video_writer_created = False
        image_topic = rospy.get_param(self.node_ns + '/topicname_image', '/image')
        stop_topic = rospy.get_param(self.node_ns + '/topicname_empty', '/empty')

        print(image_topic)
        print(stop_topic)

        # Subscribe to the image topic
        self.image_sub = rospy.Subscriber(image_topic, Image, self.write_video)
        self.stop_sub = rospy.Subscriber(stop_topic, Empty, self.stop)

    def setup_writer(self, h, w):
        # get params
        filename = rospy.get_param(self.node_ns + '/filename', 'na')
        filepath = rospy.get_param(self.node_ns + '/filepath', './')
        fps = rospy.get_param('fps', 10.0)

        # Check if the directory already exists
        if not os.path.exists(filepath):
            # Create the directory
            os.makedirs(filepath)
            print(f"Directory '{filepath}' was created successfully.")
        else:
            print(f"Directory '{filepath}' already exists.")

        # Define the video writer
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        self.outvideo = cv2.VideoWriter(filepath + filename, fourcc, fps, (h, w))

        self.video_writer_created = True

    
    def write_video(self, ros_image):
        
        # if(self.video_writer_created == False):
        #     rospy.logwarn('Setting up video writer')
        #     self.setup_writer(data.height, data.width)
        if(self.outvideo == None) :
            rospy.logwarn('Setting up the video writer')
            self.setup_writer(ros_image.height, ros_image.width)

        try:
            # Convert the ROS Image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'mono8')
            blank_image = np.zeros((ros_image.height, ros_image.width, 3), dtype=np.uint8)
            
            # Write the frame to the video file
            self.outvideo.write(blank_image)

            rospy.loginfo("wrote a frame")

        except CvBridgeError as e:
            rospy.logerr(e)



    def run(self):
        # Keep the program alive
        rospy.spin()

    def stop(self, msg):
        # Release the video writer when the object is destroyed
        self.outvideo.release()
        self.video_writer_created = False
        rospy.logwarn("Video writer released")

    def __del__(self):
        self.outvideo.release()

if __name__ == '__main__':
    node = VideoWriter()
    node.run()
