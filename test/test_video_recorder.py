#!/usr/bin/env python2

# System Imports #
import unittest

# ROS Imports #
import os
import time
import rospkg
import rospy
import rostest
import subprocess
import os 
import re
import cv2
from cv_bridge import CvBridge
import robot_video_recorder
from sensor_msgs.msg import Image
from robot_video_recorder.video_recorder import VideoRecorder


class TestVideoRecorder(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestVideoRecorder, self).__init__(*args, **kwargs)
    
    @classmethod
    def setUpClass(cls):
        camera_topic = "/camera_input"
        cls.test_video_folder = os.path.join(rospkg.RosPack().get_path('robot_video_recorder'), "videos")
        cls.frame_publisher = rospy.Publisher(camera_topic, Image, queue_size=10)
        pkg_path = rospkg.RosPack().get_path('robot_video_recorder')
        cls.frame_image = cv2.imread(os.path.join(pkg_path, 'images', 'test_image.png'))
        cls.bridge = CvBridge()
        cls.file_prefix = 'test'
        cls.file_postfix = ''
        cls.file_type = 'mp4'
        cls.recorder = VideoRecorder(camera_topic=camera_topic, folder_path= cls.test_video_folder, image_height=cls.frame_image.shape[0], image_width=cls.frame_image.shape[1], add_time_stamps=True, video_length=60, file_prefix=cls.file_prefix, file_postfix = cls.file_postfix)
        if not os.path.exists(cls.test_video_folder):
            os.makedirs(cls.test_video_folder)       

    def test_create_file_name(self):
        timestamp = time.strftime(self.recorder.timestamp_format)
        filename = self.recorder.create_file_name(timestamp)
        delimeter = "/"
        self.assertEqual(filename, "{0}{1}{2}_{3}_{4}.{5}".format(self.test_video_folder, delimeter, self.file_prefix, timestamp, self.file_postfix, self.file_type))
        
    def publish_frame(self, frames):
        self.frame_publisher.publish(self.bridge.cv2_to_imgmsg(self.frame_image, encoding='bgr8'))
        
if __name__ == "__main__":
    rospy.init_node("video_recorder_test")
    rostest.rosrun("robot_video_recorder", 'test_video_recorder', TestVideoRecorder)
