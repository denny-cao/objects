#!/usr/bin/env python

from spawn_helper import sim_control_handler
from primitives import Box, Sphere, Cylinder
from random import choice
import rospy
from primitive_msgs.srv import Spawn, SpawnRequest, SpawnResponse
from gazebo_msgs.msg import LinkStates

from franka_msgs.msg import FrankaStateCustom
import numpy as np

# import rospkg
# import os 

class SpawnShape:
    def __init__(self):
        self.shape = None
        self.spawner = sim_control_handler()
        self.number = None
        self.O_T_EE = np.zeros((4,4))  # dq, O_Jac_EE, O_dP_EE
        self.state_sub = rospy.Subscriber('/panda/franka_state_controller_custom/franka_states', FrankaStateCustom, self.franka_state_callback, queue_size=1) # want latest state (drop if stale)

    def rand_shape(self, static):
        # Generate random primitive
        self.shape = choice([Box(static=static), Sphere(static=static), Cylinder(static=static)])
        
        self.shape.sample_pose(self.O_T_EE)
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

        # link_states = rospy.wait_for_message("/gazebo/link_states", LinkStates)

        self.spawner.pause_sim()  

        for number in range(1, req.amount + 1):    
            self.number = number

            self.rand_shape(req.static) 

        self.spawner.update_prim_spawned(True)

        response = SpawnResponse()
        response.success = True

        self.spawner.unpause_sim()        

        return response
    

    def franka_state_callback(self, msg):
        '''
        keep track of franka current EE pose and EE velocity
        EE velocity is filtered
        '''

        self.O_T_EE = np.transpose(np.reshape(msg.O_T_EE, (4, 4)))

if __name__ == "__main__":
    spawn = SpawnShape()
    rospy.init_node("spawner")

    service = rospy.Service("spawn_amount", Spawn, spawn.spawn_cb)

    # rospack = rospkg.RosPack()
    # base_shape_urdf_path = os.path.join(rospack.get_path('primitive'), 'description', 'shape.urdf.xacro')
    # print(base_shape_urdf_path)
    rospy.spin()
    


