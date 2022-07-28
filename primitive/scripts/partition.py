from numpy import random

class Partition:
    def __init__(self, number, x_bounds, y_bounds):
        # Partition Number
        self.number = number

        # Boundaries of quadrant
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds

        # Coordinates of object's center
        self.x = self.y = self.z = None

        # Maximum side lengths
        self.max_dim_x = self.max_dim_y = self.max_dim_z = None

    def rand_pos(self, min_dim):
        '''
        Get random coordinates within partition
        '''
        
        self.x = round(random.uniform(self.x_bounds[0] + min_dim, self.x_bounds[1] - min_dim), ndigits=4)
        self.y = round(random.uniform(self.y_bounds[0] + min_dim, self.y_bounds[1] - min_dim), ndigits=4)
        self.z = round(random.uniform(min_dim, 2.0), ndigits=4)

    def max_len(self, max_dim):
        '''
        Get  maximum lengths in x, y, and z dimensions
        '''
        
        # Get distances from the boundaries
        x_dists = (bound - self.x for bound in self.x_bounds)
        y_dists = (bound - self.y for bound in self.y_bounds)

        # Find minimum of both to determine max size
        max_dim_x = 2 * min(x_dists) if 2 * min(x_dists) < max_dim else max_dim
        max_dim_y = 2 * min(y_dists) if 2 * min(y_dists) < max_dim else max_dim
        max_dim_z = 2 * self.z if 2 * self.z < max_dim else max_dim 

        self.max_dim_x, self.max_dim_y, self.max_dim_z = max_dim_x, max_dim_y, max_dim_z


        