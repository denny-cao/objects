from numpy import random
from random import choice
from abc import ABCMeta, abstractmethod
import xml.etree.ElementTree as ET
import rospy
from gazebo_msgs.msg import LinkStates

namespaces = {"xacro": "http://www.ros.org/wiki/xacro"}
ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")

MAX_COORD= 0.5
SAFETY_DIST = 0.1
MAX_DIM = 0.4
MIN_DIM = 0.05

class Shape(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def __init__(self, mass=10, x=None, y=None, z=None):
        self.mass = mass
        self.x = self.y = self.z = None

        self.largest_z = None
        self.lower_bound_x = self.upper_bound_x = self.lower_bound_y = self.upper_bound_y = None

    @abstractmethod
    def rand_pos(self, msg):
        # Get end link position
        link_position = msg.pose[-1].position

        largest_x = max([0, link_position.x])
        largest_y = max([0, link_position.y])
        # Get z-value of end link
        self.largest_z = msg.pose[-1].position.z
        # Limit z-value to MAX_DIM
        if self.largest_z > MAX_DIM:
            self.largest_z = MAX_DIM

        # Generate random x-value
        self.x = round(random.uniform(-MAX_COORD, MAX_COORD), ndigits=4)

        # Boundaries
        self.lower_bound_x = link_position.x - SAFETY_DIST if not largest_x else -SAFETY_DIST
        self.upper_bound_x = link_position.x + SAFETY_DIST if largest_x else SAFETY_DIST

        self.lower_bound_y = link_position.y - SAFETY_DIST if not largest_y else -SAFETY_DIST
        self.upper_bound_y = link_position.y + SAFETY_DIST if largest_y else SAFETY_DIST

        # Check if y-value needs to be bounded
        if self.lower_bound_x <= self.x <= self.upper_bound_x:
            # 50/50 sample from above or below boundary
            if choice([0, 1]) and self.upper_bound_y < MAX_COORD - MIN_DIM:
                self.y = random.uniform(self.upper_bound_y, MAX_COORD)
            
            elif self.lower_bound_y > -MAX_COORD + MIN_DIM:
                self.y = random.uniform(-MAX_COORD, self.lower_bound_y)
        else:
            self.y = round(random.uniform(-MAX_COORD, MAX_COORD), ndigits=4)

    @abstractmethod
    def rand_dim(self):
        # Max_dim is a function of distance from boundary from center of shape
        max_dim_x = round((self.x - self.upper_bound_x) * 2 if abs(self.x - self.upper_bound_x) <= abs(self.x - self.lower_bound_x) else (self.lower_bound_x - self.x) * 2, ndigits=4)
        max_dim_y = round((self.y - self.upper_bound_y) * 2 if abs(self.y - self.upper_bound_y) <= abs(self.y - self.lower_bound_y) else (self.lower_bound_y - self.y) * 2, ndigits=4)
        
        return max_dim_x, max_dim_y

    @abstractmethod
    def show(self):
        tree = ET.parse("shape.urdf.xacro")

        tree.find('xacro:property[@name="mass"]', namespaces).set("value", str(self.mass))
        tree.find('xacro:property[@name="xyz"]', namespaces).set("value",
                                                     str(self.x) + " " + str(self.y) + " " + str(self.z))
        tree.find('xacro:property[@name="shape_name"]', namespaces).set("value", self.__class__.__name__)
        return tree
    
class Box(Shape):
    def __init__(self, length=0, width=0, height=0, mass=10):
        super(Box, self).__init__()
        self.length = length
        self.width = width
        self.height = height

    def rand_pos(self, msg):
        super(Box, self).rand_pos(msg)

    def rand_dim(self):
        max_dim_x, max_dim_y = super(Box, self).rand_dim()

        self.width = round(random.uniform(MIN_DIM, max_dim_x), ndigits=4)
        self.length = round(random.uniform(MIN_DIM, max_dim_y), ndigits=4)
        self.height = round(random.uniform(MIN_DIM, self.largest_z), ndigits=4)

        self.z = self.height / 2

    def show(self):
        tree = super(Box, self).show()
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

    def rand_pos(self, msg):
        super(Sphere, self).rand_pos(msg)

    def rand_dim(self):
        max_dim_x, max_dim_y = super(Sphere, self).rand_dim()

        max_rad = abs(max_dim_x / 2) if abs(max_dim_x) < abs(max_dim_y) else abs(max_dim_y / 2)
        max_rad = abs(self.largest_z) if abs(self.largest_z) < max_rad else max_rad

        self.radius = round(random.uniform(MIN_DIM, max_rad), ndigits=4)

        self.z = self.radius

    def show(self):
        tree = super(Sphere, self).show()

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))

        tree.write("shape.urdf.xacro")


class Cylinder(Shape):
    def __init__(self, radius=0, length=0, mass=10):
        super(Cylinder, self).__init__()
        self.radius = radius
        self.length = length

    def rand_pos(self, msg):
        super(Cylinder, self).rand_pos(msg)

    def rand_dim(self):
        max_dim_x, max_dim_y = super(Cylinder, self).rand_dim()

        max_rad = max_dim_x / 2 if max_dim_x < max_dim_y else max_dim_y / 2

        self.radius = round(random.uniform(MIN_DIM, max_rad), ndigits=4)
        self.length = round(random.uniform(MIN_DIM, self.largest_z), ndigits=4)

        self.z = self.length / 2

    def show(self):
        tree = super(Cylinder, self).show()

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))
        tree.find(
            'xacro:property[@name="length"]', namespaces).set("value", str(self.length))

        tree.write("shape.urdf.xacro")

'''
[ERROR] [1658525793.910571, 79.608000]: Error processing request: unsupported operand type(s) for -: 'NoneType' and 'float'
['Traceback (most recent call last):\n', '  File "/opt/ros/melodic/lib/python2.7/dist-packages/rospy/impl/tcpros_service.py", line 633, in _handle_request\n    response = convert_return_to_response(self.handler(request), self.response_class)\n', '  File "spawner.py", line 43, in spawn_cb\n    self.rand_shape(link_states)\n', '  File "spawner.py", line 21, in rand_shape\n    shape.rand_dim()\n', '  File "/home/srlxprmntsleon/franka_ros_ws/src/objects/primitive/scripts/primitives.py", line 90, in rand_dim\n    max_dim_x, max_dim_y = super(Box, self).rand_dim()\n', '  File "/home/srlxprmntsleon/franka_ros_ws/src/objects/primitive/scripts/primitives.py", line 65, in rand_dim\n    max_dim_y = round((self.y - self.upper_bound_y) * 2 if abs(self.y - self.upper_bound_y) <= abs(self.y - self.lower_bound_y) else (self.lower_bound_y - self.y) * 2, ndigits=4)\n', "TypeError: unsupported operand type(s) for -: 'NoneType' and 'float'\n"]
'''