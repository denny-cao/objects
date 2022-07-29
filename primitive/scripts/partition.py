from numpy import random
from math import sqrt

from objects.primitive.scripts.primitives import MIN_DIM, MAX_DIM, SAFETY_DIST, PARTITION_COUNT, MAX_COORD
class Partition:
    def __init__(self, number, column, row=None):
        # Partition Number
        self.number = number

        # Partition column and row
        self.column = column
        self.row = row

        # Boundaries
        self.min_x, self.max_x = None
        self.min_y, self.max_y = None

        # Max lengths of objects
        self.max_dim_x = self.max_dim_y = None

        # Partition info 
        self.partition_axis_count = sqrt(PARTITION_COUNT) 
        self.partition_len = MAX_COORD - (SAFETY_DIST * (self.partition_axis_count - 1)) / self.partition_axis_count

        # Panda information
        self.in_panda_column = False
        self.min_panda_y = self.max_panda_y = None
        self.num_rows_below = self.num_rows_above = None

    def bounds(self, msg): 
        '''
        Get min and max for x values
        '''
    
        self.min_x = (self.partition_len + self.safety_dist) * (self.column - 1)
        self.max_x = self.partition_len * self.column + self.safety_dist * (self.column - 1)

        self.panda_info(msg)
        
        # Restrict row
        if self.in_panda_column:
            if random.choice([0, 1]) and (self.num_rows_above != 0):
                self.row = random.choice(range(self.partition_axis_count - self.num_rows_above, self.partition_axis_count + 1))
            else:
                self.row = random.choice(range(1, self.partition_axis_count - self.num_rows_below + 1))
        # Unrestrict row
        else:
            self.row = random.choice(1, self.partition_axis_count + 1)
    
    def panda_info(self, msg):
        '''
        Find if the partition currently being created exists within the panda's boundaries
        '''

        # Get link positions of panda (x, y, z)
        link_positions = [link.position for link in msg.pose]

        # Does the partition currently being created exist within these bounds?
        min_panda_x = min(link_positions, key=lambda a:a.x) 
        max_panda_x = max(link_positions, key=lambda a:a.x) 

        self.in_panda_column = True if min_panda_x <= self.min_x and max_panda_x >= self.max_x else False

        if self.in_panda_column:
            # Set panda y bounds
            self.min_panda_y = min(link_positions, key=lambda a:a.y)
            self.max_panda_y = max(link_positions, key=lambda a:a.y)

            # Get row count
            self.num_rows_below = self.panda_rows(lower=-MAX_COORD, upper=self.min_panda_y)
            self.num_rows_above = self.panda_rows(lower=self.max_panda_y, upper=MAX_COORD)

    def panda_rows(self, lower, upper):
        '''
        How many rows does the panda take?
        '''

        panda_rows = abs(upper - lower) / (self.partition_len + SAFETY_DIST)
        if panda_rows % 1 >= (self.partition_len / (self.partition_len + SAFETY_DIST)):
            panda_rows = int(panda_rows + 0.5)
        else:
            panda_rows = int(panda_rows)

        return panda_rows

    def rand_pos(self):
        '''
        Get random coordinates within partition
        '''
        
        self.x = round(random.uniform(self.min_x + MIN_DIM, self.max_x - MIN_DIM), ndigits=4)
        self.y = round(random.uniform(self.min_y + MIN_DIM, self.max_y - MIN_DIM), ndigits=4)

    def max_len(self):
        '''
        Get  maximum lengths in x, y, and z dimensions
        '''

        # Get distances from boundaries
        x_dists = (abs(self.x - self.min_x), abs(self.x - self.max_x))
        y_dists = (abs(self.y - self.min_y), abs(self.y - self.max_y))

        # Find minimum value to determine max length size in each dimension
        self.max_dim_x = 2 * min(x_dists) if 2 * min(x_dists) < MAX_DIM else MAX_DIM
        self.max_dim_y = 2 * min(y_dists) if 2 * min(y_dists) < MAX_DIM else MAX_DIM

    def partition_loc(self):
        return (self.column, self.row)