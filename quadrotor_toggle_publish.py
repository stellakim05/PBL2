#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys, select, termios, tty

def getKey():
    """Function to get a single key press."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn, z_speed):
    """Function to print the current speed, turn, and z speed values."""
    return f"currently:\tspeed {speed}\tturn {turn}\tz_speed {z_speed}"

def handleArrowKeys(key):
    """Handle arrow keys for movement control."""
    global x, th
    if key == '\x1b[A':  # Up Arrow
        x = speed  # Move forward
        th = 0     # No rotation
        print("Moving forward")
    elif key == '\x1b[B':  # Down Arrow
        x = -speed  # Move backward
        th = 0      # No rotation
        print("Moving backward")
    elif key == '\x1b[D':  # Left Arrow
        x = 0       # Stop moving forward/backward
        th = turn   # Rotate left
        print("Turning left")
    elif key == '\x1b[C':  # Right Arrow
        x = 0       # Stop moving forward/backward
        th = -turn  # Rotate right
        print("Turning right")

def handleVerticalKeys(key):
    """Handle 'w' and 's' keys for vertical movement control (up/down)."""
    global z
    if key == 'w':  # 'w' for moving up (increase z)
        z = speed   # Move up
        print("Moving up")
    elif key == 's':  # 's' for moving down (decrease z)
        z = -speed  # Move down
        print("Moving down")

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    # Initialize the ROS node
    rospy.init_node('quadrotor_toggle_publisher')

    # Publisher to send cmd_vel commands to Unity or the quadrotor
    pub = rospy.Publisher('/quadrotor/cmd_vel', Twist, queue_size=1)

    # Subscriber to get feedback from Unity or quadrotor (optional)
    unity_feedback_sub = rospy.Subscriber('/unity_movement_feedback', Bool,
                                          lambda msg: rospy.loginfo(f"Unity Feedback: {'Moving' if msg.data else 'Stopped'}"))

    # Default speed and turning parameters (can be overridden by ROS params)
    speed = rospy.get_param("~speed", 0.2)
    turn = rospy.get_param("~turn", 1.0)

    # Initial velocities
    x = 0  # Linear velocity in x (forward/backward)
    y = 0  # Linear velocity in y (left/right)
    z = 0  # Linear velocity in z (up/down)
    th = 0  # Angular velocity (rotation around z-axis)

    moving = False

    try:
        print(vels(speed, turn, z))
        print("Use arrow keys for movement (forward/backward/turn). Press 'w' to move up and 's' to move down.")
        print("Press Enter to toggle movement. Press Ctrl-C to exit.")
        rate = rospy.Rate(10)  # 10Hz loop

        while not rospy.is_shutdown():
            key = getKey()  # Get a key press from the user

            # Toggle movement with the Enter key
            if key == '\r':  # Enter key to toggle movement
                moving = not moving
                if moving:
                    x = speed  # Move forward
                    th = 0     # No rotation
                else:
                    x = 0      # Stop moving
                    th = 0     # No rotation
                print("ROS: Command sent -", "Moving" if moving else "Stopped")

            # Handle arrow keys for movement control
            handleArrowKeys(key)

            # Handle w/s keys for vertical movement control (up/down)
            handleVerticalKeys(key)

            # Publish the twist message
            rospy.loginfo(f"Publishing twist message: linear.x = {x}, linear.z = {z}, angular.z = {th}")

            # Create Twist message and set velocities
            twist = Twist()
            twist.linear.x = x
            twist.linear.y = y
            twist.linear.z = z
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th

            # Publish the twist message to the /quadrotor/cmd_vel topic
            pub.publish(twist)

            rate.sleep()  # Wait until the next iteration

    except Exception as e:
        rospy.logerr(f"Error: {e}")

    finally:
        # Send a stop command before exiting
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        rospy.loginfo("Quadrotor stopped.")

    # Restore terminal settings
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
