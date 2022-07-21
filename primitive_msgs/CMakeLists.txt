cmake_minimum_required(VERSION 2.8.3)

project(primitive_msgs)

find_package(catkin REQUIRED
        message_generation
        std_msgs
)

add_service_files( DIRECTORY srv
        FILES
        Spawn.srv
)


add_message_files( DIRECTORY msg
        FILES
        ControllerDiag.msg
)

generate_messages( DEPENDENCIES std_msgs )

generate_messages(DEPENDENCIES std_msgs geometry_msgs control_msgs sensor_msgs actionlib_msgs franka_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

