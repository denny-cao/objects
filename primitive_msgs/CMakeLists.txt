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

generate_messages( DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

