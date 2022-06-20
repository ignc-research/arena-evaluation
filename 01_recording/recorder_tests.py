#!/usr/bin/env python3

# general packages
import time
import numpy as np
import csv
import os
import sys
import subprocess
import yaml

# ros packages
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Pose2D, Pose, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray

rospy.Subscriber("/flatland_server/debug/model/obstacle_dynamic_with_traj_00", MarkerArray, self.dynamic_obs_callback)

from tf.transformations import euler_from_quaternion

class recorder():
    def __init__(self) -> None:
        # create rawdata csv file
        self.local_planner = rospy.get_param("local_planner")
        self.start = time.time()
        self.dir_path = os.path.dirname(os.path.abspath(__file__)) # get path for current file, does not work if os.chdir() was used
        self.model = rospy.get_param("model","base_model")
        self.now = time.strftime("%y-%m-%d_%H-%M-%S")
        #'''
        self.waypoint_generator = rospy.get_param("waypoint_generator")
        self.record_only_planner = rospy.get_param("record_only_planner")
        self.scenario = rospy.get_param("scenario_file").replace(".json","").replace("eval/","")


        rospy.Subscriber("/flatland_server/debug/model/obstacle_dynamic_with_traj_00", MarkerArray, self.dynamic_obs_callback)

        # def dynamic_obs_callback(self, msg_dynamic: MarkerArray):
        #     print(msg_dynamic)

        def laserscan_callback(self, msg_laserscan: LaserScan):
            self.laserscan = [float("{:.3f}".format(min(msg_laserscan.ranges)))]


if __name__=="__main__":
    rospy.init_node("data_recorder")    
    data_recorder = recorder()
    rospy.spin()
