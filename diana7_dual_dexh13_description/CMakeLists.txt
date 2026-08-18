cmake_minimum_required(VERSION 3.0.2)
project(diana7_dual_dexh13_description)

find_package(catkin REQUIRED)
catkin_package()

foreach(dir config launch meshes urdf)
  install(DIRECTORY ${dir}/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
