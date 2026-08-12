cmake_minimum_required(VERSION 2.8.3)
project(raspimouse_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs	
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

foreach(dir launch meshes urdf)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir}
  )
endforeach(dir)
