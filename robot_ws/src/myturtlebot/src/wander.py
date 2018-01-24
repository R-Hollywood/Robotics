#!/usr/bin/env python
import rospy

# Data structure here: http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ReadScan:

    range_value = 1 # anything to start with

    def query_range(self):
        return self.range_value

    def scan_callback(self, msg):
        import math
        self.range_value = min(msg.ranges)
        # If the object is not within the sensor's working range, range_value will be NaN; so let's set it to 0
        if math.isnan(self.range_value):
            self.range_value =1
        rospy.loginfo(self.range_value)
           
rs = ReadScan()
# The queue_size=1 argument tells rospy to only buffer a single outbound message. In case the node sending the messages is transmitting at a higher rate than the receiving node(s) can receive them, rospy will simply drop any messages beyond the queue_size.
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
scan_sub=rospy.Subscriber('scan',LaserScan, rs.scan_callback)

rospy.init_node('wander')

# you now subscribe to the topic, this is similar to 'rostopic echo /scan' but now the messages are channelled to 'scan_callback' rather than the screen
# The message constructors set all fields to zero. Therefore, the stop_twist message tells a robot to stop, since all of its velocity subcomponents are zero.
stop_twist = Twist()
go_twist = Twist()


# The x component of the linear velocity in a Twist message is, by convention, aligned in the direction the robot is facing, so this line means drive straight ahead at 0.5 meters per second.
go_twist.linear.x = 0.5

stop_twist.angular.z=5
stop_twist.linear.x=0

driving_forward = False

# Checkout how time works in ROS -- http://wiki.ros.org/rospy/Overview/Time
change_time = rospy.Time.now()
rate = rospy.Rate(5)

while not rospy.is_shutdown():
    
    if driving_forward:
        
        cmd_vel_pub.publish(go_twist)
# We need to continually publish a stream of velocity command messages, since most mobile base drivers will time out and stop the robot if they do not receive at least several messages per second.
        if rs.query_range() < 0.8:
            print("Hello")
            cmd_vel_pub.publish(stop_twist)
            driving_forward=False

          
# This branch checks the system time and toggles it periodically.
    if change_time < rospy.Time.now():
        if not driving_forward:
            cmd_vel_pub.publish(stop_twist)
            
        rospy.loginfo("Toggling Behaviour")
        driving_forward=not driving_forward
        change_time = rospy.Time.now() + rospy.Duration(5)
    
# Without this call to rospy.sleep() the code would still run, but it would send far too many messages, and take up an entire CPU core!
    rate.sleep()
    
    # spin() keeps your node from exiting until the node is shut down. This is thread independent and does not affect the execution of the callbacks
