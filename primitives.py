from numpy import random
from random import choice
import rospy as rs
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from abc import ABCMeta, abstractmethod
import xml.etree.ElementTree as ET

namespaces = {"xacro": "http://www.ros.org/wiki/xacro"}


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
        return Point(), Pose(orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=0.0))

    def same_len(self):
        # Restrict 50/50 axis if length and width are the same
        coord_1 = choice([random.uniform(0.2, 0.85),
                          random.uniform(-0.85, -0.2)])
        coord_2 = random.uniform(-0.85, 0.85)

        if choice([0, 1]):
            return coord_1, coord_2
        else:
            return coord_2, coord_1
    
    def diff_len(self): 
        # Resrict shortest side
        coord_1 = choice([random.uniform(0.2, 0.85),
                          random.uniform(-0.85, -0.2)])
        coord_2 = random.uniform(-0.85, 0.85)

        return coord_1, coord_2

    @abstractmethod
    def show(self, truth_table):
        tree = ET.parse("shape.urdf.xacro")
        tree.find('xacro:property[@name="mass"]', namespaces).set("value", str(self.mass))

        pose = self.rand_pos()
        position = pose.position
        orientation = pose.orientation

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
        self.length = random.uniform(0.005, 0.5)
        self.width = random.uniform(0.005, 0.5)
        self.height = random.uniform(0.005, 0.5)

    def rand_pos(self):
        position, pose = super(Box, self).rand_pos()
        if self.length == self.width:
            position.x, position.y = super(Box, self).same_len()
        
        else:
            if self.length > self.width:
                position.x, position.y = super(Box, self).diff_len()
            else:
                position.y, position.x = super(Box, self).diff_len()
        
        position.z = self.height / 2

        pose.position = position
        
        return pose
    
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
    def __init__(self, radius=0, height=0, mass=10):
        super(Sphere, self).__init__()
        self.radius = radius
        self.height = height

    def rand_dim(self):
        self.radius = random.uniform(0.0025, 0.25)
        self.height = random.uniform(0.0025, 0.25)

    def rand_pos(self):
        position, pose = super(Sphere, self).rand_pos()

        position.x, position.y = super(Sphere, self).same_len()
        position.z = self.radius

        pose.position = position
        
        return pose

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
    def __init__(self, radius=0, height=0, mass=10):
        super(Cylinder, self).__init__()
        self.radius = radius
        self.height = height

    def diameter(self):
        return self.radius * 2

    def rand_dim(self):
        self.radius = random.uniform(0.0025, 0.25)
        self.height = random.uniform(0.0025, 0.25)

    def rand_pos(self):
        position, pose = super(Cylinder, self).rand_pos()

        if self.diameter() == self.height:
            position.x, position.y = super(Cylinder, self).same_len()
        
        else:
            if  self.height > self.diameter():
                position.x, position.y = super(Cylinder, self).diff_len()
            else:
                position.y, position.x = super(Cylinder, self).diff_len()

        pose.position = position
        
        return pose


    def show(self):
        tree = super(Cylinder, self).show(truth_table={
            "box": "true",
            "sphere": "false",
            "cylinder": "true"
        })

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))
        tree.find(
            'xacro:property[@name="height"]', namespaces).set("value", str(self.height))

        tree.write("shape.urdf.xacro")


# Generate random primitive
def rand_shape():
    shape = choice([Box(), Sphere(), Cylinder()])
    
    shape.rand_dim()

    shape.show()


if __name__ == "__main__":
    rand_shape()
    
