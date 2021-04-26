#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Vector3
import time



if __name__ == '__main__':
    startTime = time.time()
    rospy.init_node('resetOdom')
    resetControls0 = rospy.Publisher('/volta_0/cmd_vel', Twist, queue_size=10)
    resetControls1 = rospy.Publisher('/volta_0/cmd_vel', Twist, queue_size=10)
    resetControls2 = rospy.Publisher('/volta_0/cmd_vel', Twist, queue_size=10)
    resetControls3 = rospy.Publisher('/volta_0/cmd_vel', Twist, queue_size=10)
    # resetOdom0 = rospy.Publisher('volta_0/volta_base_controller/odom', Empty, queue_size=10)	

    while time.time() - startTime < 1.0:
        resetControls0.publish(Twist(Vector3(0, 0, 0),Vector3(0, 0, 0)))
        resetControls1.publish(Twist(Vector3(0, 0, 0),Vector3(0, 0, 0)))
        resetControls2.publish(Twist(Vector3(0, 0, 0),Vector3(0, 0, 0)))
        resetControls3.publish(Twist(Vector3(0, 0, 0),Vector3(0, 0, 0)))
	print "__Reset__"