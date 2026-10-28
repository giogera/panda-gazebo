#!/usr/bin/env python

import csv
import rospy
import tf
from sensor_msgs.msg import JointState
from std_msgs.msg import String


collect_data = False  # variable to keep track on data collection status

# Callback to save joint states (position, velocity)
def joint_state_callback(data):
    global collect_data
    if collect_data:
        try:
            timestamp = rospy.get_time()
            joint_position = data.position
            joint_velocity = data.velocity

            with open('joint_positions.csv', 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([timestamp] + list(joint_position[:7]))  # consider only the arm joints

            with open('joint_velocities.csv', 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([timestamp] + list(joint_velocity[:7]))

        except IOError as e:
            rospy.logerr(f"Error during writing: {e}")
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")

# Function to register end-effector pose
def record_end_effector_pose(listener):
    try:
        # Get the transformation between the base link and the end-effector
        (pos, ori) = listener.lookupTransform('/panda_link0', '/panda_hand', rospy.Time(0))
        
        timestamp = rospy.get_time()
        with open('end_effector_pose.csv', 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([timestamp] + list(pos) + list(ori))

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Trasformation not available.")

# Callback to start/stop data collection
def control_callback(msg):
    global collect_data
    if msg.data == "start":
        rospy.loginfo("Data collection started.")
        collect_data = True
    elif msg.data == "stop":
        rospy.sleep(3) # wait some time for the trajectory to be completed
        rospy.loginfo("Data collection stopped.")
        collect_data = False
    else:
        rospy.logwarn(f"Received unknown signal: {msg.data}")

if __name__ == '__main__':
    rospy.init_node('data_collector', anonymous=True)
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)
    rospy.Subscriber("/data_collection", String, control_callback)

    listener = tf.TransformListener()

    rate = rospy.Rate(20)  
    while not rospy.is_shutdown():
        if collect_data:
            record_end_effector_pose(listener)
        rate.sleep()
