#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

def feedback_callback(data):
    """
    Callback function for the /quadrotor_feedback topic.
    Logs whether the quadrotor is moving or stopped.
    """
    rospy.loginfo(f"Feedback - Moving: {data.data}")

def joint_state_callback(data):
    """
    Callback function for the /quadrotor/joint_states topic.
    Logs the joint names and positions.
    """
    if data.name and data.position:
        rospy.loginfo(f"Joint States - Names: {data.name}, Positions: {data.position}")
    else:
        rospy.logwarn("Received joint state message with missing data.")

def shutdown_callback():
    """
    This function will be called when the node shuts down.
    You can place cleanup code here if necessary.
    """
    rospy.loginfo("Shutting down quadrotor subscriber node.")

if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('quadrotor_subscriber', anonymous=True)
        
        # Register the callback functions
        rospy.Subscriber("/quadrotor_feedback", Bool, feedback_callback)
        rospy.Subscriber("/quadrotor/joint_states", JointState, joint_state_callback)  # Adjust topic name if needed
        
        # Register shutdown behavior
        rospy.on_shutdown(shutdown_callback)
        
        rospy.loginfo("Quadrotor Subscriber Node Initialized.")
        
        # Keep the node running until it's shutdown
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt Exception. Shutting down the node.")
