#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32
import tf
import geometry_msgs.msg
import math

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    # SCAN_TOPIC = rospy.get_param("wall_follower/scan_topic")
    # DRIVE_TOPIC = rospy.get_param("wall_follower/drive_topic")
    SIDE = -1 #rospy.get_param("wall_follower/side")
    VELOCITY = 10   #rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = 1#rospy.get_param("wall_follower/desired_distance")

    
    def __init__(self):
        # TODO:
        # Initialize your publishers and
        # subscribers here
        
        self.drive_pub = rospy.Publisher('/drive',AckermannDriveStamped, queue_size=10)
        self.rate = rospy.Rate(10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.bag_pub = rospy.Publisher('/bagger',Float32, queue_size=10)
        # drive_message = AckermannDriveStamped()
        # # drive_message.header.time = rospy.Time.now()
        # drive_message.drive.speed = 1
        # drive_message.drive.steering_angle = math.pi/10
        pass
    
    def callback(self,data):
        all_angles = []
        
        curAngle = data.angle_min
        for i in range(int((data.angle_max-data.angle_min)/data.angle_increment)):
            all_angles.append(curAngle)
            curAngle += data.angle_increment
        
        
        if self.SIDE == -1:
            #RIGHT SIDE rospy.loginfo('right side')
            r_vals = data.ranges[10:40]
            theta_vals = all_angles[10:40]
        else:
            # LEFT SIDE rospy.loginfo('left side')
            r_vals = data.ranges[50:70]
            theta_vals = all_angles[50:70]
        
        x_vals = []
        y_vals = []
        for i in range(len(r_vals)):
            x = r_vals[i]*math.cos(theta_vals[i])
            y = r_vals[i]*math.sin(theta_vals[i])
            
            if True:
                x_vals.append(x)
                y_vals.append(y)
                
        # rospy.loginfo("")      
        # rospy.loginfo(x_vals)
        # rospy.loginfo(y_vals)
        
        alpha, Beta = np.polyfit(x_vals,y_vals,1)
        # rospy.loginfo("")
        # rospy.loginfo(Beta)
        # rospy.loginfo(alpha)      
        
        err = self.DESIRED_DISTANCE - abs(Beta)
        command = -1 * self.SIDE * 10 * err + 5 * self.SIDE * self.VELOCITY * alpha * self.SIDE
        
        rospy.loginfo(err)
        self.bag_pub.publish(err)
        
        
        driveCommand = AckermannDriveStamped()
        driveCommand.header.frame_id = 'wall_follower'
        driveCommand.header.stamp = rospy.Time.now()
        
        # rospy.loginfo(Beta)
        
        if data.ranges[50] > 1.5:
            driveCommand.drive.steering_angle = command
        else:
            driveCommand.drive.steering_angle = -1 * self.SIDE * 2
        driveCommand.drive.steering_angle_velocity = 0
        
        driveCommand.drive.speed = self.VELOCITY
        
        self.drive_pub.publish(driveCommand)
        
        
        # rospy.loginfo(str(err) + ' ' + str(command))
    
        

if __name__ == "__main__":
    
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    # ex_data = np.array([1,3,5,7,9,11,13,15,17])
    # res = wall_follower.linearize_wall_data(ex_data)
    # print(res)
    rospy.spin()