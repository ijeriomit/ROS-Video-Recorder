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
from random import seed
from random import random
import cv2
from cv_bridge import CvBridge
import threading
from sensor_msgs.msg import Image
from robot_video_recorder.video_recorder import VideoRecorder
from robot_video_recorder.image_manipulator import *



class TestVideoRecorder(unittest.TestCase):

    def __init__(self, *args, **kwargs):
        super(TestVideoRecorder, self).__init__(*args, **kwargs)
    
    @classmethod
    def setUpClass(cls):
        camera_topic = "/camera_input"
        cls.test_video_folder = os.path.join(rospkg.RosPack().get_path('robot_video_recorder'), "videos")
       
        pkg_path = rospkg.RosPack().get_path('robot_video_recorder')
        cls.frame_image = cv2.imread(os.path.join(pkg_path, 'images', 'test_image.png'))
        cls.file_prefix = 'test'
        cls.file_postfix = ''
        cls.file_type = 'mp4'
        cls.fps = 4
        cls.max_delay = 0.1
        cls.sent_images = 0
        cls.recorder = VideoRecorder(camera_topic=camera_topic, folder_path= cls.test_video_folder, image_height=cls.frame_image.shape[0], image_width=cls.frame_image.shape[1], fps=cls.fps, add_time_stamps=True, video_length=60, file_prefix=cls.file_prefix, file_postfix = cls.file_postfix)
        if not os.path.exists(cls.test_video_folder):
            os.makedirs(cls.test_video_folder)
        
        rospy.init_node("frame_publisher")
        cls.publisher_thread = threading.Thread(target=cls.run_camera, args=(camera_topic,))
        cls.publisher_thread.start()

    @classmethod
    def tearDownClass(cls): 
        # time.sleep(5)
        rospy.loginfo("shutting down ros HEHEHEHEHEH")
        rospy.signal_shutdown("test over")
        cls.publisher_thread.join()
    
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
        self.assertEqual(self.recorder.get_real_frame_number(), self.recorder.fps)

    def test_create_file_name(self):
        timestamp = time.strftime(self.recorder.timestamp_format)
        filename = self.recorder.create_file_name(timestamp)
        delimeter = "/"
        self.assertEqual(filename, "{0}{1}{2}_{3}_{4}.{5}".format(self.test_video_folder, delimeter, self.file_prefix, timestamp, self.file_postfix, self.file_type))
    
    @classmethod
    def run_camera(cls, topic):
        if cls.fps == 0:
            return
        seed(cls.max_delay)
        sleep_rate = 1/abs(cls.fps) + random()
        frame_publisher = rospy.Publisher(topic, Image, latch=True, queue_size=10) 
        bridge = CvBridge()
        while not rospy.is_shutdown():
            frame_publisher.publish(bridge.cv2_to_imgmsg(cls.frame_image, encoding='bgr8'))
            cls.sent_images += 1
            rospy.sleep(sleep_rate)

def publish_frame(publisher, image):
    rospy.loginfo("publisher: ", self.frame_publisher)
    
if __name__ == "__main__":
    rostest.rosrun("robot_video_recorder", 'test_video_recorder', TestVideoRecorder)
