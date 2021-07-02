import rospy
import cv2
import os
import time
import copy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import numpy
from robot_video_recorder.image_manipulator import *
from cv_bridge import CvBridge


class VideoRecorder(object):
    '''Records Video files using a ROS topic as a source for the video stream'''

    def __init__(self, camera_topic,image_height, image_width, add_time_stamps=False,  folder_path="", video_length=60, fps="10", video_codec="MP4V", file_prefix=None, file_postfix=None, timestamp_format="%Y-%m-%d_%H-%M-%S", file_type="mp4"):
        self._camera_topic = camera_topic
        self._folder_path = folder_path
        self._video_length = video_length
        self._file_prefix = file_prefix
        self._file_postfix = file_postfix
        self.timestamp_format = timestamp_format
        self._file_type = file_type
        self._video_codec = video_codec
        self._fps = fps
        self._video_stream_sub = None
        self._target_image_height = image_height
        self._target_image_width= image_width
        self.default_image = None
        self._video_writer = None
        self._current_filename = ""
        self._add_time_stamps = add_time_stamps
        self.bridge = CvBridge()
        self._target_frame_number = 0
        self._real_frame_number = 0
        self._frame_buffer = []
        self.path_delimeter = "\\"
        self._add_footage_timer = None
        self._end_video_timer = None
        self._recording = False
        self.__setup()

    def __setup(self):
        '''Setups up Subscriber, Creates Directories, loads default image'''
        self._target_frame_number = self._fps * self._video_length * 60
        self.default_image = numpy.zeros((self._target_image_width, self._target_image_height, 3))
        try:
            create_directory(self._folder_path)
        except os.error as e: 
            raise RuntimeError("Cannot create folder an path {}. [Error]: {}".format(self._folder_path), e)
        
      
    def record(self):
        self._recording = True
        try:
            self.start_new_video()
        except cv2.error as e:
            raise RuntimeError("{}".format(e))
        
        self._video_stream_sub = rospy.Subscriber(self._camera_topic,
                         Image, self.new_image_callback, queue_size=10)
        self._add_footage_timer = rospy.Timer(rospy.Duration(1), self.add_second_of_footage)
        self._end_video_timer = rospy.Timer(rospy.Duration(self._video_length), self.end_video)

    def stop_recording(self):
        self._add_footage_timer.shutdown()
        self._end_video_timer.shutdown()
        self._video_stream_sub.unregister()
        self._recording = False
        self.end_video(None)
        self._frame_buffer[:] = []

    def create_file_name(self, timestamp):
        filename = ""
        if not self._file_prefix and self._file_postfix:
            filename = "{0}{1}{2}_{3}.{4}".format(self._folder_path, self.path_delimeter, timestamp, self._file_postfix, self._file_type)
        elif self._file_prefix and not self._file_postfix:
            filename = "{0}{1}{2}_{3}.{4}".format(self._folder_path, self.path_delimeter, self._file_prefix, timestamp, self._file_type)
        elif not self._file_prefix and not self._file_postfix:
            filename = "{0}{1}{2}.{3}".format(self._folder_path, self.path_delimeter, self._file_prefix, timestamp, self._file_postfix, self._file_type)
        elif self._file_prefix and self._file_postfix:
            filename = "{0}{1}{2}_{3}_{4}.{5}".format(self._folder_path, self.path_delimeter, self._file_prefix, timestamp, self._file_postfix, self._file_type)
        return filename

    def end_video(self, event):
        print("end video")
        if(self._real_frame_number < self._target_frame_number):
            self.pad_video(self._target_frame_number - self._real_frame_number)
        self.reset_frame_number()
        if self._recording is True:
            self.start_new_video()
        else:
            self._video_writer.release()

    def reset_frame_number(self):
        self._real_frame_number = 0

    def start_new_video(self):
        print("start new video")
        timestamp = time.strftime(self.timestamp_format, time.gmtime(rospy.Time.now().secs))
        filename = self.create_file_name(timestamp)
        try: 
            if self._video_writer is not None and self._video_writer.isOpened():
                self._video_writer.release()
            self._video_writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*self._video_codec), self._fps, (self._target_image_width, self._target_image_height), True)
        except cv2.error as e:
            raise cv2.error("An error occured with video writer {}".format(e))

    def add_second_of_footage(self, event):
        frames = self.get_frame_buffer()
        num_pad_frames = self._fps - len(frames)
        self._frame_buffer[:] = []
        leftover_frames = self.write_frames_to_video(frames)
        self.pad_video(num_pad_frames)
        self._frame_buffer = leftover_frames + self._frame_buffer
    

    def write_frames_to_video(self, frames):
        written_frames = 0
        try:
            if(self._video_writer.isOpened()):
                for i in range(0, len(frames)):
                    self._video_writer.write(frames[i])
                    print("Wrote image to video")
                    written_frames += 1
                    self._real_frame_number += 1
                del frames[0:written_frames]
                    
        except cv2.error as e:
            rospy.logerr("An error occured when writing a video. {}".format(e))
        
        return frames
    
    def new_image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        timestamp = time.strftime(self.timestamp_format, time.gmtime(msg.header.stamp.secs))
        if msg.height != self._target_image_height or msg.width != self._target_image_width: 
            image = image_size_correction(image, self._target_image_width, self._target_image_height)
        if self._add_time_stamps: 
            image = add_text_to_image(image, timestamp)
        self._frame_buffer.append(image)
        self._real_frame_number += 1 
    
    def pad_video(self, num_pad_frames):
        pad_frames = []
        for i in range(0, num_pad_frames):
            pad_frames.append(self.default_image)
        self.write_frames_to_video(pad_frames)

    def get_frame_buffer(self):
        return copy.deepcopy(self._frame_buffer)
    
    def get_real_frame_number(self):
        return self._real_frame_number
    
def create_directory(path):
    '''creates a directory if it doesnt already exist'''
    if not os.path.exists(path):
        os.makedirs(path)
