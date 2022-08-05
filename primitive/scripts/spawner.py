#!/usr/bin/env python

from spawn_helper import sim_control_handler
from primitives import Box, Sphere, Cylinder
from random import choice
import rospy
from primitive_msgs.srv import Spawn, SpawnRequest, SpawnResponse
from gazebo_msgs.msg import LinkStates

# import rospkg
# import os 

class SpawnShape:
    def __init__(self):
        self.shape = None
        self.spawner = sim_control_handler()
        self.number = None

    def rand_shape(self, msg, static):
        # Generate random primitive
        self.shape = choice([Box(static=static), Sphere(static=static), Cylinder(static=static)])
        
        self.shape.sample_pose(msg)
        self.shape.rand_dim()
        self.shape.rand_mass()
        self.shape.rand_friction()
        self.shape.show()

        self.spawner.update_shape(self.shape, self.number)
            
        self.spawner.spawn_model()


    def spawn_cb(self, req):     
        if self.spawner.primitive_spawned:  
            self.spawner.pause_sim()  
            
            self.spawner.delete_models()
            self.number = 0

            self.spawner.unpause_sim()

        link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)

        self.spawner.pause_sim()  

        for number in range(1, req.amount + 1):    
            self.number = number

            self.rand_shape(link_states, req.static) 

        self.spawner.update_prim_spawned(True)

        response = SpawnResponse()
        response.success = True

        self.spawner.unpause_sim()        

        return response

if __name__ == "__main__":
    spawn = SpawnShape()
    rospy.init_node("spawner")

    service = rospy.Service("spawn_amount", Spawn, spawn.spawn_cb)

    # rospack = rospkg.RosPack()
    # base_shape_urdf_path = os.path.join(rospack.get_path('primitive'), 'description', 'shape.urdf.xacro')
    # print(base_shape_urdf_path)
    rospy.spin()
    


