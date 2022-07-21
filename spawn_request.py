#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
    spawn_pub = rospy.Publisher("change_prim", Bool)
    rospy.init_node("spawner_request")
    
    spawn_pub.publish(True)