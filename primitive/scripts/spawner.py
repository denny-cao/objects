#!/usr/bin/env python

from spawn_helper import sim_control_handler
from primitives import Box, Sphere, Cylinder
from random import choice
import rospy
from primitive_msgs.srv import Spawn, SpawnRequest, SpawnResponse
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Bool

class SpawnShape:
    def __init__(self):
        self.shape = None
        self.spawner = sim_control_handler()
        self.number = None
        self.ef = None

    def rand_shape(self, msg, static):
        # Generate random primitive
        self.shape = choice([Box(static=static), Sphere(static=static), Cylinder(static=static)])
        
        self.shape.rand_pos(msg)
        self.shape.rand_dim()
        self.shape.rand_mass()
        self.shape.rand_friction()
        self.shape.show()

        self.spawner.update_shape(self.shape, self.number)
            
        self.spawner.spawn_model()

    def swap(self):
        self.ef = choice([Box(static=True, ef=True), Sphere(static=True, ef=True), Cylinder(static=True, ef=True)])

        self.spawner.update_ef(self.shape)

        self.spawner.swap_ef()
        
    def spawn_cb(self, req):     
        if self.spawner.primitive_spawned:  
            self.spawner.pause_sim()  
            
            self.spawner.delete_models()
            self.number = 0

        if self.spawner.ef:
            self.spawner.pause_sim()

            self.spawner.delete_ef()
            self.ef = None

        self.spawner.unpause_sim()

        link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)

        self.spawner.pause_sim()  

        for number in range(1, req.amount + 1):    
            self.number = number

            self.rand_shape(link_states, req.static) 

        self.spawner.update_prim_spawned(True)

        if req.swap:
            self.swap
            self.spawner.update_ef_spawned(True)
        
        response = SpawnResponse()
        response.success = True

        self.spawner.unpause_sim()        

        return response


if __name__ == "__main__":
    spawn = SpawnShape()
    rospy.init_node("spawner")

    spawn_service = rospy.Service("spawn_amount", Spawn, spawn.spawn_cb)

    rospy.spin()
    


