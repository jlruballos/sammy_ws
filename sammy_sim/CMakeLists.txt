cmake_minimum_required(VERSION 3.8)
project(sammy_sim)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)
install(PROGRAMS
  scripts/json_joint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY motion/
  DESTINATION share/${PROJECT_NAME}/motion
)
ament_package()

