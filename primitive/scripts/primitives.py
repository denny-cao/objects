from math import pi
from numpy import random
import xml.etree.ElementTree as ET
from partition import Partition

namespaces = {"xacro": "http://www.ros.org/wiki/xacro"}
ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")

MAX_COORD = 0.5
MIN_DIM = 0.05
MAX_DIM = 0.2

class Shape(object):
    def __init__(self, mass=10, x=None, y=None, z=None, mu=1.0, mu2 = 1.0, r=0, p=0, ya=0, static=True):
        self.name = None

        # Position
        self.x = self.y = self.z = x, y, z
        
        self.largest_z = None
        self.lower_bound_x = self.upper_bound_x = self.lower_bound_y = self.upper_bound_y = None
        self.max_dim_x = self.max_dim_y = self.max_dim_z = None
        
        # Orientation
        self.r, self.p, self.ya = r, p, ya

        # Physics
        self.mass = mass
        self.mu = mu
        self.mu2 = mu2

        # Static or dynamic
        self.static = static

    def rand_pos(self, Partition):
        '''
        Place object in random location in a partition
        '''
        
        Partition.rand_pos(MIN_DIM)
        self.x, self.y, self.z = Partition.x, Partition.y, Partition.z

    def rand_dim(self, Partition):
        '''
        Generate maximum dimensions for implementations in subclasses 
        '''
        
        Partition.max_len(MAX_DIM)
        self.max_dim_x, self.max_dim_y, self.max_dim_z = Partition.max_dim_x, Partition.max_dim_y, Partition.max_dim_z 

    def show(self):
        '''
        Edit xacro file with new properties
        '''

        tree = ET.parse("shape.urdf.xacro")

        tree.find('xacro:property[@name="static"]', namespaces).set("value", str(self.static))
        tree.find('xacro:property[@name="xyz"]', namespaces).set("value",
                                                     str(self.x) + " " + str(self.y) + " " + str(self.z))
        tree.find('xacro:property[@name="rpy"]', namespaces).set("value",
                                                     str(self.r) + " " + str(self.p) + " " + str(self.ya))
        tree.find('xacro:property[@name="shape_name"]', namespaces).set("value", self.name)
        tree.find('xacro:property[@name="mass"]', namespaces).set("value", str(self.mass))
        tree.find('xacro:property[@name="mu"]', namespaces).set("value", str(self.mu))
        tree.find('xacro:property[@name="mu2"]', namespaces).set("value", str(self.mu2))
   
        return tree
    
    def rand_mass(self):
        self.mass = round(random.uniform(0.0, 2.0), ndigits=4)

    def rand_friction(self):
        self.mu = random.uniform(0, 2/(9.8*self.mass))
        self.mu2 = self.mu
    
class Box(Shape):
    def __init__(self, length=0, width=0, height=0, mass=10, x=0, y=0, z=0, r=0, p=0, ya=0, static=True, mu=1.0, mu2=1.0):
        super(Box, self).__init__(mass, x, y, z, mu, mu2, r, p, ya, static)
        self.name = "Box"

        self.length = length
        self.width = width
        self.height = height

    def rand_dim(self):
        super(Box, self).rand_dim()

        self.width = round(random.uniform(MIN_DIM, self.max_dim_x), ndigits=4)
        self.length = round(random.uniform(MIN_DIM, self.max_dim_y), ndigits=4)
        self.height = round(random.uniform(MIN_DIM, self.max_dim_z), ndigits=4)

        self.z = self.height / 2 if self.static else self.z
 
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
    def __init__(self, radius=0, mass=10, x=0, y=0, z=0, r=0, p=0, ya=0, static=True, mu=1.0, mu2=1.0):
        super(Sphere, self).__init__(mass, x, y, z, mu, mu2, r, p, ya, static)

        self.name = "Sphere"

        self.radius = radius
    
    def rand_dim(self):
        super(Sphere, self).rand_dim()

        max_rad = self.max_dim_x if self.max_dim_x < self.max_dim_z else self.max_dim_z
        max_rad /= 2

        self.radius = round(random.uniform(MIN_DIM, max_rad), ndigits=4)

        self.z = self.radius / 2 if self.static else self.z

    def show(self):
        tree = super(Sphere, self).show()

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))

        tree.write("shape.urdf.xacro")

class Cylinder(Shape):
    def __init__(self, radius=0, length=0, mass=10, x=0, y=0, z=0, r=0, p=0, ya=0, static=True, mu=1.0, mu2=1.0):
        super(Cylinder, self).__init__(mass, x, y, z, mu, mu2, r, p, ya, static)
        
        self.name = "Cylinder"
        
        self.radius = radius
        self.length = length

    def rand_dim(self):
        super(Cylinder, self).rand_dim()

        max_rad = self.max_dim_x if self.max_dim_x < self.max_dim_z else self.max_dim_z
        max_rad /= 2

        self.radius = round(random.uniform(MIN_DIM, max_rad), ndigits=4)
        self.length = round(random.uniform(MIN_DIM, self.max_dim_z), ndigits=4)
        
        self.z = self.length / 2 if self.static else self.z
    
    def show(self):
        tree = super(Cylinder, self).show()

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))
        tree.find(
            'xacro:property[@name="length"]', namespaces).set("value", str(self.length))

        tree.write("shape.urdf.xacro")
