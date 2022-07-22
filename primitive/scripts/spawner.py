#!/usr/bin/env python

from spawn_helper import sim_control_handler
from primitives import Box, Sphere, Cylinder
from random import choice
import rospy
from primitive_msgs.srv import Spawn, SpawnRequest, SpawnResponse
from gazebo_msgs.msg import LinkStates

class SpawnShape:
    def __init__(self):
        self.shape = None
        self.spawner = sim_control_handler()
        self.number = 0

    def rand_shape(self, msg):
        self.spawner.pause_sim()

        # Generate random primitive
        shape = choice([Box(), Sphere(), Cylinder()])
        
        shape.rand_pos(msg)
        shape.rand_dim()
        shape.show()

        self.shape = shape

        self.spawner.update_shape(self.shape, self.number)
            
        self.spawner.spawn_model()

    def spawn_cb(self, req):

        if self.spawner.primitive_spawned:
            self.spawner.delete_models()
            self.number = 0
            
        for number in range(1, req.amount + 1):    
            number += 1
            link_states = rospy.Subscriber("/gazebo/link_states", LinkStates, self.rand_shape)
            link_states.unregister()
        self.spawner.unpause_sim()        

if __name__ == "__main__":
    spawn = SpawnShape()
    rospy.init_node("spawner")

    service = rospy.Service("spawn_amount", Spawn, spawn.spawn_cb)

    rospy.spin()
    


