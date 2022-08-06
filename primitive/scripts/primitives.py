from math import pi
from numpy import random
import xml.etree.ElementTree as ET
import rospkg
import os 


namespaces = {"xacro": "http://www.ros.org/wiki/xacro"}
ET.register_namespace("xacro", "http://www.ros.org/wiki/xacro")


MAX_COORD= 0.3
SAFETY_DIST = 0.05
MAX_DIM = 0.1
MIN_DIM = 0.05

class Shape(object):
    def __init__(self, mass=10, x=None, y=None, z=None, mu=1.0, mu2 = 1.0, r=0, p=0, ya=0, static=True):
        self.name = None

        # Position
        self.x = self.y = self.z = x, y, z
        
        self.largest_z = None
        self.lower_bound_x = self.upper_bound_x = self.lower_bound_y = self.upper_bound_y = None
        self.max_dim_x = self.max_dim_y = None
        
        # Orientation
        self.r, self.p, self.ya = r, p, ya

        # Physics
        self.mass = mass
        self.mu = mu
        self.mu2 = mu2

        # Static or dynamic
        self.static = static

        self.rospack = rospkg.RosPack()
        self.base_shape_urdf_path = os.path.join(self.rospack.get_path('primitive'), 'description', 'shape.urdf.xacro')
  
    def denny_sample_pos(self, O_p_EE, largest_x, largest_y):


        # Generate random x-value
        self.x = round(random.uniform(-MAX_COORD, MAX_COORD), ndigits=4)

        # Boundaries
        self.lower_bound_x = O_p_EE[0] - SAFETY_DIST if not largest_x else -SAFETY_DIST
        self.upper_bound_x = O_p_EE[0] + SAFETY_DIST if largest_x else SAFETY_DIST

        self.lower_bound_y = O_p_EE[1] - SAFETY_DIST if not largest_y else -SAFETY_DIST
        self.upper_bound_y = O_p_EE[1] + SAFETY_DIST if largest_y else SAFETY_DIST

        # Check if y-value needs to be bounded
        if self.lower_bound_x <= self.x <= self.upper_bound_x:
            # 50/50 sample from above or below boundary
            if random.choice([0, 1]) and self.upper_bound_y < MAX_COORD - MIN_DIM:
                self.y = round(random.uniform(self.upper_bound_y, MAX_COORD), ndigits=4)
            
            else:
                self.y = round(random.uniform(-MAX_COORD, self.lower_bound_y), ndigits=4)
        else:
            self.y = round(random.uniform(-MAX_COORD, MAX_COORD), ndigits=4)

    def sample_pose(self, O_T_EE):
        # Get end link position
        # link_position = msg.pose[-1].position
        O_p_EE = O_T_EE[:3, -1]
        
        largest_x = max([0, O_p_EE[0]])
        largest_y = max([0, O_p_EE[1]])
        # Get z-value of end link
        self.largest_z = O_p_EE[-1]
        # Limit z-value to MAX_DIM
        if self.largest_z > MAX_DIM:
            self.largest_z = MAX_DIM

        # sample position
        self.denny_sample_pos(O_p_EE, largest_x, largest_y)
        # Change orientation for dynamic objects
        # if not self.static:
        self.r = round(random.uniform(-pi, pi), ndigits=4)
        self.p = round(random.uniform(-pi, pi), ndigits=4)
        self.ya = round(random.uniform(-pi, pi), ndigits=4)
    
    def unif_sample_circle(self, radius):
        pass
        #TODO

    def rand_dim(self):
        # Max_dim is a function of distance from boundary from center of shape
        self.max_dim_x = round(abs((self.x - self.upper_bound_x)) * 2 if abs(self.x - self.upper_bound_x) <= abs(self.x - self.lower_bound_x) else (abs(self.lower_bound_x - self.x)) * 2, ndigits=4)
        self.max_dim_y = round(abs((self.y - self.upper_bound_y)) * 2 if abs(self.y - self.upper_bound_y) <= abs(self.y - self.lower_bound_y) else (abs(self.lower_bound_y - self.y)) * 2, ndigits=4)
        
    def show(self):
        tree = ET.parse(self.base_shape_urdf_path)

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
        self.mass = round(random.uniform(0.0, 5.0), ndigits=4)

    def rand_friction(self):
        self.mu = random.uniform(0, 2.0)
        self.mu2 = self.mu

    # def rand_pos_denny(self, msg):
    #     # Get end link position
    #     link_position = msg.pose[-1].position

    #     largest_x = max([0, link_position.x])
    #     largest_y = max([0, link_position.y])
    #     # Get z-value of end link
    #     self.largest_z = msg.pose[-1].position.z
    #     # Limit z-value to MAX_DIM
    #     if self.largest_z > MAX_DIM:
    #         self.largest_z = MAX_DIM

    #     # Generate random x-value
    #     self.x = round(random.uniform(-MAX_COORD, MAX_COORD), ndigits=4)

    #     # Boundaries
    #     self.lower_bound_x = link_position.x - SAFETY_DIST if not largest_x else -SAFETY_DIST
    #     self.upper_bound_x = link_position.x + SAFETY_DIST if largest_x else SAFETY_DIST

    #     self.lower_bound_y = link_position.y - SAFETY_DIST if not largest_y else -SAFETY_DIST
    #     self.upper_bound_y = link_position.y + SAFETY_DIST if largest_y else SAFETY_DIST

    #     # Check if y-value needs to be bounded
    #     if self.lower_bound_x <= self.x <= self.upper_bound_x:
    #         # 50/50 sample from above or below boundary
    #         if random.choice([0, 1]) and self.upper_bound_y < MAX_COORD - MIN_DIM:
    #             self.y = round(random.uniform(self.upper_bound_y, MAX_COORD), ndigits=4)
            
    #         else:
    #             self.y = round(random.uniform(-MAX_COORD, self.lower_bound_y), ndigits=4)
    #     else:
    #         self.y = round(random.uniform(-MAX_COORD, MAX_COORD), ndigits=4)
        
    #     # Change orientation for dynamic objects
    #     # if not self.static:
    #     self.r = round(random.uniform(-pi, pi), ndigits=4)
    #     self.p = round(random.uniform(-pi, pi), ndigits=4)
    #     self.ya = round(random.uniform(-pi, pi), ndigits=4)
    
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
        self.height = round(random.uniform(MIN_DIM, self.largest_z), ndigits=4)

        self.z = self.height / 2 if self.static else random.uniform(2.0, 3.0)

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

        max_rad = self.max_dim_x / 2 if self.max_dim_x < self.max_dim_y else self.max_dim_y / 2
        max_rad = self.largest_z if self.largest_z < max_rad else max_rad
        max_rad /= 2

        self.radius = round(random.uniform(MIN_DIM, max_rad), ndigits=4)

        self.z = self.radius if self.static else random.uniform(2, 3)

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

        max_rad = self.max_dim_x / 2 if self.max_dim_x < self.max_dim_y else self.max_dim_y / 2
        max_rad /= 2

        self.radius = round(random.uniform(MIN_DIM, max_rad), ndigits=4)
        self.length = round(random.uniform(MIN_DIM, self.largest_z), ndigits=4)
        
        self.z = self.length / 2 if self.static else random.uniform(2, 3)
    
    def show(self):
        tree = super(Cylinder, self).show()

        tree.find(
            'xacro:property[@name="radius"]', namespaces).set("value", str(self.radius))
        tree.find(
            'xacro:property[@name="length"]', namespaces).set("value", str(self.length))

        tree.write("shape.urdf.xacro")
