#!/usr/bin/env python

import csv
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

def trajectory_publisher():
    # Initialize ROS node
    rospy.init_node('trajectory_publisher')
    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)
    signal = rospy.Publisher('/data_collection', String, queue_size=10)
    
    poses_to_send = 0
    poses_sent = 0

    # Read trajectory
    csv_file = '/home/gio/gazebo_ws/src/panda_control/trajectory.csv'
    with open(csv_file, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  
        trajectory = []
        for row in reader:
            time = float(row[0])
            pose = PoseStamped()
            pose.header.stamp = rospy.Time(time)
            pose.header.frame_id = "panda_link0"
            pose.pose.position.x = float(row[1])
            pose.pose.position.y = float(row[2])
            pose.pose.position.z = float(row[3])
            pose.pose.orientation.x = float(row[4])
            pose.pose.orientation.y = float(row[5])
            pose.pose.orientation.z = float(row[6])
            pose.pose.orientation.w = float(row[7])
            trajectory.append(pose)
            poses_to_send = poses_to_send + 1

    # Publish the trajectory
    rospy.sleep(3)
    signal.publish("start")
    rospy.loginfo("Starting publishing.")

    rate = rospy.Rate(20)  
    start_time = rospy.Time.now()
    for pose in trajectory:
        while rospy.Time.now() < start_time + rospy.Duration(pose.header.stamp.to_sec()):
            rate.sleep()
        pub.publish(pose)
        poses_sent = poses_sent + 1
    signal.publish("stop")
    
    final_message = "Published " + str(poses_sent) + " poses out of " + str(poses_to_send) + "."
    rospy.loginfo(final_message)               

if __name__ == '__main__':
    trajectory_publisher()

