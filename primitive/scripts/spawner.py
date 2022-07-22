#!/usr/bin/env python

from spawn_helper import sim_control_handler
from primitives import Box, Sphere, Cylinder
from random import choice
import rospy
from primitive_msgs.srv import Spawn, SpawnRequest, SpawnResponse
from gazebo_msgs.msg import LinkStates

class Spawn:
    def __init__(self):
        self.shape = None
        self.spawner = sim_control_handler()


    def rand_shape(self, msg):
        # Generate random primitive
        shape = choice([Box(), Sphere(), Cylinder()])
        
        shape.rand_pos()
        shape.rand_dim()
        shape.show()

        self.shape = shape

    def spawn_cb(self, req):
        self.spawner.pause_sim()

        if self.spawner.primitive_spawned:
            self.spawner.delete_models()
        
        for number in range(1, req.amount + 1):    
            link_states_sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.rand_shape) 
            
            self.spawner.update_shape(self.shape, number)
            
            self.spawner.spawn_model()

        self.spawner.unpause_sim()        

if __name__ == "__main__":
    spawn = Spawn()
    rospy.init_node("spawner")

    service = rospy.Service("spawn_amount", Spawn, spawn.spawn_cb)

    rospy.spin()
    


