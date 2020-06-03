#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from ros_conveyor.msg import conveyor
import time
import math

sensor_1 = False
sensor_2 = False
sensor_3 = False
current_vel = 0.0


def state_callback(msg):
    global sensor_1, sensor_2, sensor_3, current_vel
    sensor_1 = msg.sensor_1
    sensor_2 = msg.sensor_2
    sensor_3 = msg.sensor_3
    current_vel = msg.current_vel


if __name__ == "__main__":
    rospy.init_node("example_node")
    rospy.loginfo("Starting example_node.")

    command_pub = rospy.Publisher("conveyorA/command", Float32, queue_size=1)
    command_msg = Float32()
    rospy.Subscriber("conveyorA/state", conveyor, state_callback)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if sensor_1:
            command_msg.data = 2.0 * math.sin(4*time.time())
        elif sensor_2:
            command_msg.data = current_vel
        elif sensor_3:
            command_msg.data = -1.0
        else:
            command_msg.data = 2.0 * \
                math.sin(2*time.time()) + 1.5 * math.sin(4*time.time())
        command_pub.publish(command_msg)
        rate.sleep()
