import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy
from observe.image_manipulator import *
from cv_bridge import CvBridge


class Recorder(object):
    '''Records Video files using a ROS topic as a source for the video stream'''

    def __init__(self, camera_topic, folder_path="", image_height, image_width, add_time_stamps=false, recording_interval=60, fps="10" video_codec, file_prefix="", file_postfix="", timestamp_format="%Y-%m-%d_%H-%M-%S", file_type="mp4")
        self._camera_topic = camera_topic
        self._folder_path = folder_path
        self._recording_interval = recording_interval
        self._file_prefix = file_prefix
        self._file_postfix = file_postfix
        self._timestamp_format = timestamp_format
        self._file_type = file_type
        self._video_codec = video_codec
        self._fps = fps
        self._video_stream_sub = None
        self._trigger_interval_sub = None
        self._image_height = image_height
        self._image_width = image_width
        self.default_image = None
        self._video_writer = None
        self._current_filename = ""
        self._add_time_stamps = add_time_stamps
        self.bridge = CvBridge()
        self._target_frame_number = 0
        self._real_frame_number = 0
        self._image_buffer = []
        self._temp_buffer = []
        self._writing = False

    def __setup(self):
        '''Setups up Subcriber, Creates Directories, loads default image'''
        self._video_stream_sub = rospy.Subscriber(self._camera_topic,
                         Image, self.new_image_callback, queue_size=10)
        
        self._trigger_interval_sub = rospy.Subcriber("trigger_interval", Bool, self.)
        self._target_frame_number = self._fps * self._recording_interval * 60
        self.default_image = numpy.zeros((self._image_width, self._image_height, 3))
        filename = self.create_file_name()
        
        try:
            create_directory(self.folder_path)
        except os.error as e: 
            raise RuntimeError("Cannot create folder an path {}. [Error]: {}".format(self.folder_path), e)
        
        try:
            self._start_new_video(self, filename)
        except cv2.error as e:
            raise RuntimeError("{}".format(e))


    def create_file_name(self, timestamp):
        filename = "~/{0}/{1}/{2}/{3}.{4}".format(self._folder_path, self._file_prefix, timestamp, self.file_postfix, self.file_type)
        return filename

    def start_new_video(self, filename):
        try: 
            if(self._video_writer.is_opened()):
                self._video_writer.release()
            self._video_writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*self._codec), self._fps, (self._image_width, self._image_height), True)
        except cv2.error as e:
            raise cv2.error("An error occured with video writer {}".format(e))

    def new_image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        timestamp = time.strftime(self.timestamp_format, time.gmtime(msg.header.stamp.secs))
        if msg.height != (self._image_height or self._image_width): 
            image = image_size_correction(image, self._image_width, self._image_height)
        if self._add_time_stamps: 
            image = add_text_to_image(image, timestamp)
        self._image_buffer.append(image)
        # self.write_image_to_video(image)
        self._real_frame_number += 1 
    
    def pad_video(self, num_pad_frames):
        for i in range(0, num_pad_frames):
            self.write_image_to_video(self.default_image)

    def interval_callback(self, data):
        if(self._real_frame_number < self._target_frame_number):
            self.pad_video(self._target_frame_number - self._real_frame_number)
        self._video_writer.release()
        timestamp = time.strftime(self.timestamp_format)
        filename = self.create_file_name(timestamp)
        self.start_new_video(filename)
    
    def write_image_to_video(self, event):
        images = copy.deepcopy(self._image_buffer)
        del self._image_buffer[0:len(images)-1]
        num_of_images = len(images)
        try:
            if(self._video_writer.is_opened()):
                for i in range(0, self._fps - num_of_images):
                    images.append(self.default_image)
                for i in range(0, num_of_images):
                    self._video_writer.write(image)
        except cv2.error as e:
            print("An error occured when writing a video. {}".format(e))
    
def create_directory(path):
    '''creates a directory if it doesnt already exist'''
    if not os.path.exists(path):
        os.makedirs(path)
