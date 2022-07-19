from numpy import random
from random import choice
import rospy
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Point
from abc import ABCMeta, abstractmethod
import xml.etree.ElementTree as ET

namespaces = {"xacro": "http://www.ros.org/wiki/xacro"}
ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")

max_coord = 0.5
min_coord = 0.1

max_dim = 0.4
min_dim = 0.005

class Shape(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, mass=10):
        self.mass = mass
    
    @abstractmethod
    def rand_dim(self):
        pass
    
    @abstractmethod
    def rand_pos(self):
        pass

    def same_len(self):
        # Restrict 50/50 axis if length and width are the same
        coord_1 = choice([random.uniform(min_coord, max_coord),
                          random.uniform(-max_coord, -min_coord)])
        coord_2 = random.uniform(-max_coord, max_coord)

        if choice([0, 1]):
            return coord_1, coord_2
        else:
            return coord_2, coord_1
    
    def diff_len(self): 
        # Resrict shortest side
        coord_1 = choice([random.uniform(min_coord, max_coord),
                          random.uniform(-max_coord, -min_coord)])
        coord_2 = random.uniform(-max_coord, max_coord)

        return coord_1, coord_2

    @abstractmethod
    def show(self, truth_table):
        tree = ET.parse("shape.urdf.xacro")
        tree.find('xacro:property[@name="mass"]', namespaces).set("value", str(self.mass))

        position = self.rand_pos()

        tree.find('xacro:property[@name="xyz"]', namespaces).set("value",
                                                     str(position.x) + " " + str(position.y) + " " + str(position.z))

        tree.find('xacro:property[@name="use_box"]', namespaces).set("value", truth_table["box"])
        tree.find('xacro:property[@name="use_cylinder"]', namespaces).set("value", truth_table["cylinder"])
        tree.find('xacro:property[@name="use_sphere"]', namespaces).set("value", truth_table["sphere"])

        return tree


class Box(Shape):
    def __init__(self, length=0, width=0, height=0, mass=10):
        super(Box, self).__init__()
        self.length = length
        self.width = width
        self.height = height

    def rand_dim(self):
        self.length = random.uniform(min_dim, max_dim)
        self.width = random.uniform(min_dim, max_dim)
        self.height = random.uniform(min_dim, max_dim)

    def rand_pos(self):
        position = Point()

        if self.length == self.width:
            position.x, position.y = super(Box, self).same_len()
        
        else:
            if self.length > self.width:
                position.x, position.y = super(Box, self).diff_len()
            else:
                position.y, position.x = super(Box, self).diff_len()
        
        position.z = self.height / 2
        
        return position
    
    def show(self):
        tree = super(Box, self).show(truth_table={
            "box": "true",
            "sphere": "false",
            "cylinder": "false"
        })

        tree.find(
            'xacro:property[@name="length"]', namespaces).set("value", str(self.length))
        tree.find(
            'xacro:property[@name="width"]', namespaces).set("value", str(self.width))
        tree.find(
            'xacro:property[@name="height"]', namespaces).set("value", str(self.height))

        tree.write("shape.urdf.xacro")


class Sphere(Shape):
    def __init__(self, radius=0, mass=10):
        super(Sphere, self).__init__()
        self.radius = radius

    def rand_dim(self):
        self.radius = random.uniform(min_dim / 2, max_dim / 2)

    def rand_pos(self):
        position = Point()

        position.x, position.y = super(Sphere, self).same_len()
        position.z = self.radius        
      
        return position

    def show(self):
        tree = super(Sphere, self).show(truth_table={
            "box": "false",
            "sphere": "true",
            "cylinder": "false"
        })

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))

        tree.write("shape.urdf.xacro")


class Cylinder(Shape):
    def __init__(self, radius=0, length=0, mass=10):
        super(Cylinder, self).__init__()
        self.radius = radius
        self.length = length

    def diameter(self):
        return self.radius * 2

    def rand_dim(self):
        self.radius = random.uniform(min_dim / 2, max_dim / 2)
        self.length = random.uniform(min_dim / 2, max_dim / 2)

    def rand_pos(self):
        position = Point()

        position.x, position.y = super(Cylinder, self).same_len()
        position.z = self.length / 2
        
        return position


    def show(self):
        tree = super(Cylinder, self).show(truth_table={
            "box": "false",
            "sphere": "false",
            "cylinder": "true"
        })

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))
        tree.find(
            'xacro:property[@name="length"]', namespaces).set("value", str(self.length))

        tree.write("shape.urdf.xacro")


# Spawn and Delete Model
class sim_control_handler():
    def __init__(self):
        self.shape_model_name = "object"
        self.delete_model_proxy = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        self.spawn_model_proxy = rospy.ServiceProxy("/gazebo/spawn_urdf_model", SpawnModel)


# Generate random primitive
def rand_shape():
    shape = choice([Box(), Sphere(), Cylinder()])
    
    shape.rand_dim()

    shape.show()


if __name__ == "__main__":
    rand_shape()
    
