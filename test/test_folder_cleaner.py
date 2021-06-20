#!/usr/bin/env python2

# System Imports #
import unittest

# ROS Imports #
import os
import sys
import time
import rospy
import rostest
import observe.folder_cleaner as FC
from observe.folder_cleaner import FolderCleaner
from observe.video_recorder import VideoRecorder
from theia_log import *

class TestFolderCleaner(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestFolderCleaner, self).__init__(*args, **kwargs)
        self.package_path = os.path.dirname(os.getcwd())
        self.temp_path  = os.path.join(self.package_path , "videos", "temporary")
        self.event_path = os.path.join(self.package_path , "videos", "events")
        self.archive_path = os.path.join(self.package_path , "videos", "archives")
        self.test_directory = os.path.join(self.package_path, "test", "FC_test_folder")
        if not os.path.exists(self.test_directory):
            os.makedirs(self.test_directory)
    
    def setUp(self):
        self.today_date = time.strftime(VideoRecorder.date_format)
        #create directories
        if not os.path.exists(os.path.join(self.temp_path, self.today_date)):
            os.makedirs(os.path.join(self.temp_path, self.today_date))
        if not os.path.exists(os.path.join(self.event_path, self.today_date)):
            os.makedirs(os.path.join(self.event_path, self.today_date))
        if not os.path.exists(os.path.join(self.archive_path, self.today_date)):
            os.makedirs(os.path.join(self.archive_path, self.today_date))

    def tearDown(self):
        FC.clean_folder(self.test_directory)
        FC.clean_folder(self.archive_path)
        FC.clean_folder(self.event_path)
        FC.clean_folder(self.temp_path)

    def test_find_empty_disk_space(self):
        self.assertNotEqual(-1, FC.find_empty_disk_space(self.archive_path))
    
    def test_get_oldest_file(self):
        for i in range(0, 3):
            try:
                f = open(os.path.join(self.test_directory, str(i) + ".txt"), 'w+')
                f.close()
            except:
                self.fail()
        os.utime(os.path.join(self.test_directory,"0.txt"), (1, 1))
        self.assertEqual(os.path.join(self.test_directory, "0.txt"), FC.get_oldest_file(self.test_directory))

    def test_get_oldest_directory(self):
        for i in range(0, 3):
            try:
                os.mkdir(os.path.join(self.test_directory, str(i)))
            except:
                self.fail()
        os.utime(os.path.join(self.test_directory,"0"), (1, 1))
        self.assertEqual(os.path.join(self.test_directory, "0"), FC.get_oldest_directory_in_folder(self.test_directory))
    
    def test_clean_folder_with_files_and_subfolders(self):
        for i in range(0, 3):
            open(os.path.join(self.test_directory, str(i) + ".txt"), 'w+')
            os.mkdir(os.path.join(self.test_directory, str(i)))
        FC.clean_folder(self.test_directory)
        self.assertEqual(0, len(os.listdir(self.test_directory)))

    def test_clean_older_archives(self):
        old_file = os.path.join(self.archive_path, self.today_date, "old.mp4")
        open(old_file, "w")
        # make it really old
        os.utime(old_file, (1, 1))
        # make disk threshold very big(sys.maxint) to trigger the folder cleaner every time
        folder_cleaner = FolderCleaner(self.temp_path, self.event_path, self.archive_path, 5, sys.maxint, 30)
        folder_cleaner.clean_older_archive(30, self.today_date)
        self.assertTrue("old.mp4" not in os.listdir(os.path.join(self.archive_path, self.today_date)))

    def test_clean_older_archives_with_folder(self):
        old_dir = os.path.join(self.archive_path, "old" )
        os.mkdir(old_dir)
        os.utime(old_dir, (1, 1))
        folder_cleaner = FolderCleaner(self.temp_path, self.event_path, self.archive_path, 5, sys.maxint, 30)
        folder_cleaner.clean_older_archive(30)
        self.assertTrue("old" not in os.listdir(self.archive_path))

    def test_clean_older_events(self):
        old_file = os.path.join(self.event_path, self.today_date, "old.mp4")
        open(old_file, "w")
        # make it really old
        os.utime(old_file, (1, 1))
        # make disk threshold very big(sys.maxint) to trigger the folder cleaner every time
        folder_cleaner = FolderCleaner(self.temp_path, self.event_path, self.archive_path, 5, sys.maxint, 30)
        folder_cleaner.clean_older_event(30, self.today_date)
        self.assertTrue("old.mp4" not in os.listdir(os.path.join(self.event_path, self.today_date)))

    def test_clean_older_events_with_folder(self):
        old_dir = os.path.join(self.event_path, "old" )
        os.mkdir(old_dir)
        os.utime(old_dir, (1, 1))
        folder_cleaner = FolderCleaner(self.temp_path, self.event_path, self.archive_path, 5, sys.maxint, 30)
        folder_cleaner.clean_older_event(30)
        self.assertTrue("old" not in os.listdir(self.archive_path))

    def test_check_folders(self):
        today_temp = os.path.join(self.temp_path, self.today_date)
        today_archive = os.path.join(self.archive_path, self.today_date)
        today_event = os.path.join(self.event_path, self.today_date)
        try:
            a = open(os.path.join(today_temp, "test.mp4"), "w")
            b = open(os.path.join(today_archive, "test.mp4"), "w")
            c = open(os.path.join(today_event, "test.mp4"), "w")
            a.close()
            b.close()
            c.close()
        except:
            self.fail()
        rospy.sleep(1)
        folder_cleaner = FolderCleaner(self.temp_path, self.event_path, self.archive_path, sys.maxint, sys.maxint, 30)
        # check if temp folder was cleaned
        # self.assertFalse(os.path.exists(today_temp))
        # self.assertEqual(0, len(os.listdir(self.temp_path)))
        # check if event and archive folders were not cleared
        self.assertIn("test.mp4", os.listdir(today_event))
        self.assertGreater(len(os.listdir(self.event_path)), 0)
        
        self.assertIn("test.mp4", os.listdir(today_archive))
        self.assertGreater(len(os.listdir(self.archive_path)), 0)

        folder_cleaner.check_folders(None)
        # # check if the files are deleted because threshold is too high
        self.assertIs(len(os.listdir(self.archive_path)), 0)
        # # event files are never deleted
        self.assertGreater(len(os.listdir(self.event_path)), 0)
    

if __name__ == "__main__":
    rospy.init_node("folder_cleaner_test")
    rostest.rosrun("observe", 'test_folder_cleaner', TestFolderCleaner)
