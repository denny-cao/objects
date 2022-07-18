from numpy import random
from random import choice
import rospy as rs
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, DeleteModel, DeleteModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from abc import ABCMeta, abstractmethod
import xml.etree.ElementTree as ET


class Shape():
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, mass=10):
        self.mass = mass

    @abstractmethod
    def rand_dim(self):
        pass

    @abstractmethod
    def rand_pos(self):
        coord_1 = coord_2 = 0
        position = Point()

        # Restrict both axes
        if self.length == self.width:
            coord_1 = choice([random.uniform(0.2, 0.85),
                             random.uniform(-0.85, -0.2)])
            coord_2 = choice([random.uniform(0.2, 0.85),
                             random.uniform(-0.85, -0.2)])

            position.x, position.y = coord_1, coord_2

        # Restrict one axis
        else:
            coord_1 = choice([random.uniform(0.2, 0.85),
                             random.uniform(-0.85, -0.2)])
            coord_2 = random.uniform(-0.85, 0.85)

            # Restrict x-values
            if self.length > self.width:
                position.x, position.y = coord_1, coord_2

            # Restrict y-values
            else:
                position.x, position.y = coord_2, coord_1

        orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        return Pose(position=position, orientation=orientation)

    @abstractmethod
    def show(self):
        tree = ET.parse("shape.urdf.xacro")
        tree.find('xacro:property[@name="mass"]').set("value", str(self.mass))

        position = self.rand_pos()

        tree.find('xacro:property[@name="xyz"]').set("value",
                                                     str(position.x) + " " + str(position.y) + " " + str(position.z))

        return tree


class Box(Shape):
    def __init__(self, length=0, width=0, height=0, mass=10):
        super(Shape, self).__init__()
        self.length = length
        self.width = width
        self.height = height

    def rand_dim(self):
        self.length = random.uniform(0.005, 0.5)
        self.width = random.uniform(0.005, 0.5)
        self.height = random.uniform(0.005, 0.5)

    def rand_pos(self):
        position = super(Shape, self).rand_pos()
        position.z = self.height / 2

        return position

    def show(self):
        tree = super(Shape, self).show()

        tree.find('xacro:property[@name="use_box"]').set("value", "true")
        tree.find('xacro:property[@name="use_cylinder"]').set("value", "false")
        tree.find('xacro:property[@name="use_sphere"]').set("value", "false")
        tree.find(
            'xacro:property[@name="length"]').set("value", str(self.length))
        tree.find(
            'xacro:property[@name="width"]').set("value", str(self.width))
        tree.find(
            'xacro:property[@name="height"]').set("value", str(self.height))

        tree.write("shape.urdf.xacro")


class Sphere(Shape):
    def __init__(self, radius=0, height=0, mass=10):
        super(Shape, self).__init__()
        self.radius = radius
        self.height = height

    def rand_dim(self):
        self.radius = random.uniform(0.0025, 0.25)
        self.height = random.uniform(0.0025, 0.25)

    def rand_pos(self):
        position = super(Shape, self).rand_pos()
        position.z = self.radius

        return position

    def show(self):
        tree = super(Shape, self).show()

        tree.find('xacro:property[@name="use_sphere"]').set("value", "true")
        tree.find('xacro:property[@name="use_box"]').set("value", "false")
        tree.find('xacro:property[@name="use_cylinder"]').set("value", "false")
        tree.find(
            'xacro:property[@name="radius"]').set("value", str(self.radius))

        tree.write("shape.urdf.xacro")


class Cylinder(Shape):
    def __init__(self, radius=0, mass=10):
        super(Shape, self).__init__()
        self.radius = radius

    def rand_dim(self):
        self.radius = random.uniform(0.0025, 0.25)

    def rand_pos(self):
        position = super(Shape, self).rand_pos()
        position.z = self.height / 2

        return position

    def show(self):
        tree = super(Shape, self).show()

        tree.find('xacro:property[@name="use_cylinder"]').set("value", "true")
        tree.find('xacro:property[@name="use_sphere"]').set("value", "false")
        tree.find('xacro:property[@name="use_box"]').set("value", "false")
        tree.find(
            'xacro:property[@name="radius"]').set("value", str(self.radius))
        tree.find(
            'xacro:property[@name="height"]').set("value", str(self.height))

        tree.write("shape.urdf.xacro")


# Generate random primitive
def rand_shape():
    shape = choice([Box(), Sphere(), Cylinder()])
    
    shape.rand_dim()
    shape.rand_pos()

    shape.show()


if __name__ == "__main__":
    rand_shape()
    
