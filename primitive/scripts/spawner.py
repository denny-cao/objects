#!/usr/bin/env python

from spawn_helper import sim_control_handler
from primitives import Box, Sphere, Cylinder
from random import choice
import rospy
from primitive_msgs.srv import Spawn

def rand_shape():
    # Generate random primitive
    shape = choice([Box(), Sphere(), Cylinder()])
    
    shape.rand_dim()
    shape.rand_pos()
    shape.show()

    return shape


def callback(req):
    if spawner.primitive_spawned:
            spawner.delete_models()
    
    for number in range(1, req.amount + 1):    
        shape = rand_shape()
        spawner.update_shape(shape, number)
        
        spawner.spawn_model()


if __name__ == "__main__":
    spawner = sim_control_handler()

    rospy.init_node("spawner")

    service = rospy.Service("spawn_amount", Spawn, callback)

    rospy.spin()
    