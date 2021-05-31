#!/usr/bin/env python2
"Grabs OpenCV images from Cameras over ROS Topic"
from threading import Thread
import rospy
from std_msgs import Bool

class TimerTrigger(Thread):
    ''' Monitorss time intervals '''

    def __init__(self, time_interval):
        """ """

        self._trigger_pub = rospy.Publisher("trigger_interval", Bool, latch=false)
        self._timer_on = False
    
    def start_timer(self):
        self._timer_on = True
        start_time = rospy.Time.now()

        while self.timer_on:
            if(rospy.Time.now() - start_time >= self.time_interval):
                self.interval_trigger()
                start_time = rospy.Time.now()
            rospy.sleep(rospy.Duration(1))
    
    def stop_timer(self):
        self._timer_on = False
    
    def destroy_timer(self):
        self.stop_timer()
        self._trigger_pub.shutdown()


    def interval_trigger(self):
        msg = Bool(data=True)
        self.trigger_pub.publish(msg)