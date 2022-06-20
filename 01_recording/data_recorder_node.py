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
from std_msgs.msg import Int32, Float32MultiArray, String

# for transformations
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

        ''' #for debugging:
        self.waypoint_generator = True# rospy.get_param("waypoint_generator")
        self.record_only_planner = True#rospy.get_param("record_only_planner")
        self.scenario = 'test'# rospy.get_param("scenario_file").replace(".json","").replace("eval/","")
        #self.real-eval = rospy.get_param("real-eval")
        #'''

        if self.record_only_planner:
            with open(self.dir_path+"/{0}_{1}--{2}--{3}.csv".format(self.local_planner,self.model,self.scenario,self.now), "w+", newline = "") as file:
                writer = csv.writer(file, delimiter = ',')
                header = [["episode","time","laser_scan","robot_lin_vel_x","robot_lin_vel_y","robot_ang_vel","robot_orientation","robot_pos_x","robot_pos_y","action","model", "number_dynamic_obs", "form_dynamic_obs", "size_dynamic_obs", "speed_dynamic_obs", "number_static_obs", "form_static_obs", "size_static_obs"]]
                writer.writerows(header)
                file.close()
        else:
            with open(self.dir_path+"/{0}_{1}_{2}--{3}--{4}.csv".format(self.local_planner,self.waypoint_generator,self.model,self.scenario,self.now), "w+", newline = "") as file:
                writer = csv.writer(file, delimiter = ',')
                header = [["episode","time","laser_scan","robot_lin_vel_x","robot_lin_vel_y","robot_ang_vel","robot_orientation","robot_pos_x","robot_pos_y","action", "number_dynamic_obs", "form_dynamic_obs", "size_dynamic_obs", "speed_dynamic_obs", "number_static_obs", "form_static_obs", "size_static_obs"]]
                writer.writerows(header)
                file.close()

        # read config
        with open(self.dir_path+ "/data_recorder_config.yaml") as file:
            self.config = yaml.safe_load(file)

        # initialize variables to be recorded with default values, NOTE: time is recorded as well, but no need for seperate variable
        self.episode = 0
        self.laserscan = ["None"]
        self.robot_lin_vel_x = 0
        self.robot_lin_vel_y = 0
        self.robot_ang_vel = 0
        self.robot_orientation = 0
        self.robot_pos_x = 0
        self.robot_pos_y = 0
        self.action = ["None"]
        self.last_action_time = rospy.get_time()

        # Ricardo new line
        self.dynamic_obs_number = 0
        self.dynamic_obs_form = ["None"]
        self.dynamic_obs_size = ["None"]
        self.dynamic_obs_speed = ["None"]

        self.static_obs_number = 0
        self.static_obs_form = ["None"]
        self.static_obs_size = ["None"]
        #--------------------------
        

        # subscribe to topics
        rospy.Subscriber("/scenario_reset", Int16, self.episode_callback)
        rospy.Subscriber("/scan", LaserScan, self.laserscan_callback)
        rospy.Subscriber("/odom", Odometry, self.odometry_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.action_callback)

        # Ricardo new line
        rospy.Subscriber("/obstacles/dynamic/number", Int32, self.dynamic_number_callback)
        rospy.Subscriber("/obstacles/dynamic/form", String, self.dynamic_form_callback)
        rospy.Subscriber("/obstacles/dynamic/radius", String, self.dynamic_size_callback)
        rospy.Subscriber("/obstacles/dynamic/speed", String, self.dynamic_speed_callback)

        rospy.Subscriber("/obstacles/static/number", Int32, self.static_number_callback)
        rospy.Subscriber("/obstacles/static/form", String, self.static_form_callback)
        rospy.Subscriber("/obstacles/static/radius", String, self.static_size_callback)
        #--------------------------------


        #rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)


    # Ricardo new line
    def dynamic_number_callback(self, msg: Int32):
        self.dynamic_obs_number = msg.data

    def dynamic_form_callback(self, msg: String):
        self.dynamic_obs_form = msg.data.split(",")

    def dynamic_size_callback(self, msg: String):
        str_list = msg.data.split(",")
        self.dynamic_obs_size  = [float(x) for x in str_list]

    def dynamic_speed_callback(self, msg: String):
        str_list = msg.data.split(",")
        self.dynamic_obs_speed  = [float(x) for x in str_list]

    def static_number_callback(self, msg: Int32):
        self.static_obs_number = msg.data

    def static_form_callback(self, msg: String):
        self.static_obs_form = msg.data.split(",") 

    def static_size_callback(self, msg: String):
        str_list = msg.data.split(",")
        self.static_obs_size = [float(x) for x in str_list]
    #----------------------------------------


    def clear_costmaps(self):
        bashCommand = "rosservice call /move_base/clear_costmaps"
        process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
        output, error = process.communicate()
        return output, error

    # define callback function for all variables and their respective topics
    def episode_callback(self, msg_scenario_reset: Int16):
        self.episode = msg_scenario_reset.data
        self.clear_costmaps()

    def laserscan_callback(self, msg_laserscan: LaserScan):
        self.laserscan = [float("{:.3f}".format(min(msg_laserscan.ranges)))]

        #  check for termination criterion "max time"
        # if time.time()-self.start > self.config["max_time"]:
        #     subprocess.call(["killall","-9","rosmaster"]) # apt-get install psmisc necessary
        #     sys.exit()

    def odometry_callback(self, msg_Odometry: Odometry):
        pose3d = msg_Odometry.pose.pose
        twist = msg_Odometry.twist.twist
        pose2d = self.pose3D_to_pose2D(pose3d)
        self.robot_lin_vel_x = float("{:.3f}".format(twist.linear.x))
        self.robot_lin_vel_y = float("{:.3f}".format(twist.linear.y))
        self.robot_ang_vel = float("{:.3f}".format(twist.angular.z))
        #if rospy.get_param("/real-eval", default=True):
        self.robot_orientation = float("{:.3f}".format(pose2d.theta))
        self.robot_pos_x = float("{:.3f}".format(pose2d.x))
        self.robot_pos_y =  float("{:.3f}".format(pose2d.y))

    # def amcl_callback(self, msg_PoseWithCovarianceStamped: PoseWithCovarianceStamped):
    #     pose3d = msg_PoseWithCovarianceStamped.pose.pose
    #     pose2d = self.pose3D_to_pose2D(pose3d)
    #     #if rospy.get_param("/real-eval", default=False):
    #         self.robot_orientation = pose2d.theta
    #         self.robot_pos_x = pose2d.x
    #         self.robot_pos_y = pose2d.y

    def pose3D_to_pose2D(self, pose3d: Pose):
        pose2d = Pose2D()
        pose2d.x = pose3d.position.x
        pose2d.y = pose3d.position.y
        quaternion = (
            pose3d.orientation.x,
            pose3d.orientation.y,
            pose3d.orientation.z,
            pose3d.orientation.w,
        )
        euler = euler_from_quaternion(quaternion)
        yaw = euler[2]
        pose2d.theta = yaw
        return pose2d

    def action_callback(self, msg_action: Twist): # variables will be written to csv whenever an action is published
        self.action = [msg_action.linear.x,msg_action.linear.y,msg_action.angular.z]
        self.action = [float("{:.3f}".format(_)) for _ in self.action]
        current_simulation_action_time = rospy.get_time() 
        current_action_time = time.time()
        if current_simulation_action_time - self.last_action_time >= self.config["record_frequency"]:
            if self.laserscan != ["None"]:         
                self.last_action_time = current_simulation_action_time
                self.addData(np.array(
                    [self.episode,
                    float("{:.3f}".format(current_simulation_action_time)),
                    list(self.laserscan),
                    self.robot_lin_vel_x,
                    self.robot_lin_vel_y,
                    self.robot_ang_vel,
                    self.robot_orientation,
                    self.robot_pos_x,
                    self.robot_pos_y,
                    self.action,
                    self.model,
                    # Ricardo new line
                    self.dynamic_obs_number,
                    self.dynamic_obs_form,
                    self.dynamic_obs_size,
                    self.dynamic_obs_speed,

                    self.static_obs_number,
                    self.static_obs_form,
                    self.static_obs_size
                    #--------------------
                    ]
                ))

        # check for termination criterion "max episodes"
        # if self.episode == self.config["max_episodes"]-1:
        #     subprocess.call(["killall","-9","rosmaster"]) # apt-get install psmisc necessary
        #     sys.exit()

    def addData(self, data:np.array): #add new row to the csv file
        if self.record_only_planner:
            with open(self.dir_path+"/{0}_{1}--{2}--{3}.csv".format(self.local_planner,self.model,self.scenario,self.now), "a+", newline = "") as file:
                writer = csv.writer(file, delimiter = ',') # writer has to be defined again for the code to work
                writer.writerows(data.reshape(1,-1)) # reshape into line vector
                file.close()
        else:
            with open(self.dir_path+"/{0}_{1}_{2}--{3}--{4}.csv".format(self.local_planner,self.waypoint_generator,self.model,self.scenario,self.now), "w+", newline = "") as file:
                writer = csv.writer(file, delimiter = ',') # writer has to be defined again for the code to work
                writer.writerows(data.reshape(1,-1)) # reshape into line vector
                file.close()



if __name__=="__main__":
    rospy.init_node("data_recorder")    
    data_recorder = recorder()
    rospy.spin()