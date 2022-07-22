from objects.primitive.scripts.primitives import MIN_COORD
import rospy 
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point
from numpy import random


MAX_COORD = 0.4
MIN_COORD = 0.05

MAX_DIM = 0.4
MIN_DIM = 0.05

SAFETY_DIST = 0.05

def pos_cb(msg):
    # Get list of link positions
    link_positions = [link.position for link in msg.pos]
    
    # Find furthest x-values
    largest_x = max(link.x for link in link_positions)
    smallest_x = min(link.x for link in link_positions)

    # Generate random x-value
    x = random.uniform(-MAX_COORD, MAX_COORD)

    # Check if y-value needs to be bounded
    # if smallest_x - SAFETY_DIST <= x <= largest_x + SAFETY_DIST:
       # pass


if __name__ == "__main__":
    rospy.init_node("link positions")

    pos = rospy.subscriber("/gazebo/link_states", LinkStates, pos_cb)




