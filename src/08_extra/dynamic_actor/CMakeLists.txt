cmake_minimum_required(VERSION 3.8)
project(dynamic_actor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

# ── C++ Gazebo plugins ───────────────────────────────────────────────────────
function(add_gz_plugin target src)
  add_library(${target} SHARED ${src})
  target_link_libraries(${target}
    ignition-gazebo6::ignition-gazebo6
    ignition-plugin1::register
  )
  install(TARGETS ${target} LIBRARY DESTINATION lib)
endfunction()

# World plugin: publishes /actor/*/pose to ROS 2
add_gz_plugin(ActorPosePublisher src/ActorPosePublisher.cc)
ament_target_dependencies(ActorPosePublisher rclcpp geometry_msgs)

# Actor plugin: teleports a ghost collision model to the actor's pose each tick
add_gz_plugin(AttachModelPlugin src/AttachModelPlugin.cc)

# Actor plugin: drives actor movement along waypoints with obstacle avoidance
add_gz_plugin(TrajectoryActorPlugin src/TrajectoryActorPlugin.cc)

# ── Python package + scripts ─────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
  dynamic_actor/actor_pose_subscriber.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME actor_pose_subscriber
)

# ── Resources ─────────────────────────────────────────────────────────────────
install(DIRECTORY launch worlds meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
