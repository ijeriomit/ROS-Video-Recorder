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
from robot_video_recorder.video_recorder import VideoRecorder


class TestVideoRecorder(unittest.TestCase):


    def __init__(self, *args, **kwargs):
        super(TestVideoRecorder, self).__init__(*args, **kwargs)
    
    @classmethod
    def setUpClass(cls):
        camera_topic = ""
        cls.test_video_folder = os.path.join(rospkg.RosPack().get_path('robot_video_recorder'), "videos")
        cls.frame_publisher = rospy.Publisher(camera_topic, Image, queue_size=10)
        cls.frame_image = cv2.imread(os.path.join(pkg_path, 'images', 'test.png'))
        cls.file_prefix = "test"
        cls.file_postfix = ''
        self.file_type = 'mp4'
        cls.recorder = VideoRecorder(camera_topic=camera_topic, folder_path= cls.test_video_folder, image_height=cls.frame_image.shape[0], image_width=cls.frame_image.shape[1], add_time_stamps=true, video_length=60, file_prefix=file_prefix, file_postfix = file_postfix)
         if not os.path.exists(self.test_video_folder):
            os.makedirs(self.test_video_folder)       

    def test_create_file_name(self):
        timestamp = time.strftime(self.recorder.timestamp_format)
        filename = self.recorder.create_file_name()
        delimeter = "/"
        self.assertEqual(filename, "{0}{1}{2}_{3}_{4}.{5}".format(self.test_video_folder, delimeter, self.file_prefix, timestamp, self.file_postfix, self.file_type))
        


    def tearDown(self):
        self.clean_videos_folder()
    
    def publish_frame(self, frames):
        self.frame_publisher.publish(self.bridge.cv2_to_imgmsg(self.frame_image, encoding='bgr8'))

    def check_in_string(self, str, sub_string, start_index, end_index ):
        return str[start_index : end_index] == sub_string

 
if __name__ == "__main__":
    rospy.init_node("video_recorder_test")
    rostest.rosrun("observe", 'test_video_recorder', TestVideoRecorder)
