#!/usr/bin/env python
import rospy
import cv2
import os
from std_msgs.msg import String

def split_video_to_frames(vid_path, out_path):
    video = cv2.VideoCapture(vid_path)

    vid_name = vid_path.split('/')[-1].split('.')[0]
    rospy.loginfo(vid_name)

    out_full_path = os.path.join(out_path, vid_name)
    rospy.loginfo(out_full_path)

    if not os.path.exists(out_full_path):
        os.makedirs(out_full_path, exist_ok=True)

    fcount = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    rospy.loginfo(fcount)

    for i in range(0, fcount):
        ret, frame = video.read()
        if not ret:
            break
        cv2.imwrite('{}/img_{:05d}.jpg'.format(out_full_path, i), frame)

    video.release()
    rospy.loginfo('{} done'.format(vid_name))

def video_frame_splitter():
    rospy.init_node('video_frame_splitter', anonymous=True)
    vid_path = rospy.get_param('~vid_path')
    out_path = rospy.get_param('~out_path')
    split_video_to_frames(vid_path, out_path)

if __name__ == '__main__':
    try:
        video_frame_splitter()
    except rospy.ROSInterruptException:
        pass
