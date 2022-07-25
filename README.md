# Spawn Primitive Objects

## BUILD
```shell
catkin build primitive
catkin build primitive_msgs 
```

## RUN
Python2.7

```shell
python spawner.py
```
- Create new service, spawn_amount
- Arguments: Amount (How Many Objects to Spawn [INT], Static or Dynamic [BOOL])

```shell
rosservice call \spawn_amount "{amount: 3, static: False}"
```

## TODO
- Add pause and border around franka to stop objects from spawning in it when moving
- Add response to service