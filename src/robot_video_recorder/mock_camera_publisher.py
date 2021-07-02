from __future__ import division
import rospy 
from threading import Thread
from random import seed
from random import random
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class MockCamera(Thread):

    def __init__(self, fps, topic, image_path, max_delay=0.1):
        Thread.__init__(self)
        self.bridge = CvBridge()
        self.fps = fps
        self.topic = topic
        self._running = False
        self.max_delay = max_delay
        self.images_sent = 0
        self.timer = None
        self.image_publisher = rospy.Publisher(topic, Image, latch=True, queue_size=10)
        try:
            self.mock_image = cv2.imread(image_path)
        except cv2.error as e:
            rospy.logerr("Problem opening file. {} Default image will be used instead".format(e))
            self.mock_image = np.zeros((900, 1200, 3))
        
        self.mock_image = CvBridge().cv2_to_imgmsg(self.mock_image, encoding='bgr8')

    def run_camera(self):
        if self.fps == 0:
            return
        self._running = True
        seed(self.max_delay)
        while not rospy.is_shutdown() and self._running:
            sleep_rate = 1/abs(self.fps) + random()
            self.publish_image()
            rospy.sleep(sleep_rate)

    def publish_image(self):
        print("Sent image")
        self.mock_image.header.stamp = rospy.Time.now()
        self.image_publisher.publish(self.mock_image)
        self.images_sent += 1
        
    def stop_camera(self):
        self._running = False
        self.join()
    
    def run(self):
        print("started")
        self.run_camera()
