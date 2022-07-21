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
- Argument: Amount (How Many Objects to Spawn [INT])

```shell
rosservice call \spawn_amount 3
```