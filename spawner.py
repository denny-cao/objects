#!/usr/bin/env python

from spawn_helper import sim_control_handler
from primitives import Box, Sphere, Cylinder
from random import choice


def rand_shape():
    # Generate random primitive
    shape = choice([Box(), Sphere(), Cylinder()])
    
    shape.rand_dim()

    shape.rand_pos()

    shape.show()

    return shape


if __name__ == "__main__":
    shape = rand_shape()
    
    spawner = sim_control_handler(shape=shape)

    if spawner.primitive_spawned:
        spawner.delete_model()
    
    spawner.spawn_model()
    
