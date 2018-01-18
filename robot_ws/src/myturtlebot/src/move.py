#!/usr/bin/env python

# Import ROS python API
import rospy

# Import the message information we are going to use (more info here: http://wiki.ros.org/msg). To find out the data structure for this particular message, go to: http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html
from geometry_msgs.msg import Twist

# The queue_size=1 argument tells rospy to only buffer a single outbound message. In case the node sending the messages is transmitting at a higher rate than the receiving node(s) can receive them, rospy will simply drop any messages beyond the queue_size.
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
rospy.init_node('go_stop')

# The message constructors set all fields to zero. Therefore, the stop_twist message tells a robot to stop, since all of its velocity subcomponents are zero.
stop_twist = Twist()
go_twist = Twist()

# The x component of the linear velocity in a Twist message is, by convention, aligned in the direction the robot is facing, so this line means drive straight ahead at 0.5 meters per second.
go_twist.linear.x = 0.5

driving_forward = False

# Checkout how time works in ROS -- http://wiki.ros.org/rospy/Overview/Time
change_time = rospy.Time.now()
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    if driving_forward:
# We need to continually publish a stream of velocity command messages, since most mobile base drivers will time out and stop the robot if they do not receive at least several messages per second.
        rospy.loginfo("GO!")
        cmd_vel_pub.publish(go_twist)
    else:
        rospy.loginfo("STOP!")
        cmd_vel_pub.publish(stop_twist)

# This branch checks the system time and toggles it periodically.
    if change_time < rospy.Time.now():
        rospy.loginfo("Toogling behaviour!")
        driving_forward = not driving_forward
        change_time = rospy.Time.now() + rospy.Duration(3)
    
# Without this call to rospy.sleep() the code would still run, but it would send far too many messages, and take up an entire CPU core!
    rate.sleep()