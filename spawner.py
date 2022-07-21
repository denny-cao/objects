#!/usr/bin/env python

from spawn_helper import sim_control_handler
from primitives import Box, Sphere, Cylinder
from random import choice, randint
import rospy
from std_msgs.msg import Bool

def rand_shape():
    # Generate random primitive
    shape = choice([Box(), Sphere(), Cylinder()])
    
    shape.rand_dim()
    shape.rand_pos()
    shape.show()

    return shape


def callback(msg):
    if spawner.primitive_spawned:
            spawner.delete_models()
    
    shape_amount = randint(2, 5)
    
    for number in range(1, shape_amount):
        shape = rand_shape()

        spawner.update_shape(shape, number)
        spawner.spawn_model()


if __name__ == "__main__":
    spawner = sim_control_handler()

    # subscribe to node that publishes when the objects are to be changed
    change_prim_sub = rospy.Subscriber("/change_prim", Bool, callback)
    
    rospy.init_node("spawner")

    rospy.spin()
    
