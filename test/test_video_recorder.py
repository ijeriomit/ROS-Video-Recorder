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
import shutil
from random import seed
from random import random
import cv2
from cv_bridge import CvBridge
import threading
from sensor_msgs.msg import Image
from robot_video_recorder.video_recorder import VideoRecorder
from robot_video_recorder.image_manipulator import *
from robot_video_recorder.mock_camera_publisher import MockCamera



class TestVideoRecorder(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestVideoRecorder, self).__init__(*args, **kwargs)
    
    @classmethod
    def setUpClass(cls):
        camera_topic = "/camera_input"
        cls.test_video_folder = os.path.join(rospkg.RosPack().get_path('robot_video_recorder'), "videos")
        rospy.init_node("test_node")
        pkg_path = rospkg.RosPack().get_path('robot_video_recorder')
        frame_image = cv2.imread(os.path.join(pkg_path, 'images', 'test_image.png'))
        cls.file_prefix = 'test'
        cls.file_postfix = ''
        cls.file_type = 'mp4'
        cls.codec = 'MP4V'
        cls.fps = 4
        cls.max_delay = 0.1
        cls.sent_images = 0
        cls.recorder = VideoRecorder(camera_topic=camera_topic, folder_path= cls.test_video_folder, image_height=frame_image.shape[0], image_width=frame_image.shape[1], fps=cls.fps, add_time_stamps=True, video_length=60, file_prefix=cls.file_prefix, file_postfix = cls.file_postfix, file_type = cls.file_type, video_codec= cls.codec)
        if not os.path.exists(cls.test_video_folder):
            os.makedirs(cls.test_video_folder)
        clean_folder(cls.test_video_folder)
        
        cls.mock_camera = MockCamera(fps=cls.fps, topic=camera_topic, image_path=os.path.join(pkg_path, 'images', 'test_image.png'))
        cls.mock_camera.start()

    @classmethod
    def tearDownClass(cls): 
        # time.sleep(5)
        rospy.loginfo("shutting down ros HEHEHEHEHEH")
        rospy.signal_shutdown("test over")
        cls.mock_camera.stop_camera()
        cls.recorder.stop_recording()
    
    def setUp(self):
        self.recorder.record()
    
    def tearDown(self):
        self.recorder.stop_recording()
    
    def test_pad_images(self):
        self.recorder.stop_recording()
        num_pad_images = 4
        frame_number = self.recorder.get_real_frame_number()
        self.recorder.pad_video(num_pad_images)
        self.assertEqual(frame_number + num_pad_images, self.recorder.get_real_frame_number())
    
    def test_image_size_correction(self):
        test_image = np.zeros((900, 1200, 3))
        self.assertEqual((768, 1024, 3), image_size_correction(test_image, 1024, 768).shape)

    def test_image_recieved(self):
        rospy.sleep(2)
        self.assertGreater(len(self.recorder.get_frame_buffer()), 1)
    
    def test_num_of_images_recieved_equals_num_of_images_sent(self):
        num_images_sent = self.sent_images
        rospy.sleep(1)
        self.assertEqual(self.recorder.get_real_frame_number(), self.fps)

    def test_create_file_name(self):
        timestamp = time.strftime(self.recorder.timestamp_format)
        filename = self.recorder.create_file_name(timestamp)
        delimeter = "\\"
        self.assertEqual(filename, "{0}{1}{2}_{3}_{4}.{5}".format(self.test_video_folder, delimeter, self.file_prefix, timestamp, self.file_postfix, self.file_type))
    
    def test_create_directory(self):
        pass

    def test_video_recorded(self):
        pass

    def test_recorded_video_has_min_number_of_frames(self):
        pass

def clean_folder(folder):
    """ Delete the files in the directory """
    if(os.path.exists(folder) and os.path.isdir(folder)):
        for f in os.listdir(folder):
            f = os.path.join(folder, f)
            if os.path.isfile(f):
                rospy.loginfo("Cleaning file {}".format(f))
                os.remove(f)
            elif os.path.isdir(f):
                rospy.loginfo("Cleaning folder {}".format(f))
                shutil.rmtree(f)

if __name__ == "__main__":
    rostest.rosrun("robot_video_recorder", 'test_video_recorder', TestVideoRecorder)
   