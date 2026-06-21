cmake_minimum_required(VERSION 2.8.3)
project(lsdc_slam)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)|(ARM64)|(aarch64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  if(N GREATER 4)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=3)
    message("core for MP: 3")
  elseif(N GREATER 3)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=2)
    message("core for MP: 2")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")
find_package (GeographicLib REQUIRED)

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  mavros
  mavros_msgs
  livox_ros_driver
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  include)

add_message_files(
  FILES
  Pose6D.msg bywire_chassis_state.msg
)

generate_messages(
 DEPENDENCIES
 std_msgs
 geometry_msgs
 nav_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(rs_velodyne src/preprocess/rs_to_velodyne.cpp)
add_dependencies(rs_velodyne ${catkin_EXPORTED_TARGETS})
target_link_libraries(rs_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(lsdc_mapping src/LIO/laserMapping.cpp include/ikd-Tree/ikd_Tree.cpp src/LIO/preprocess.cpp)
add_dependencies(lsdc_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lsdc_mapping ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(lsdc_mapping PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(lsdc_ins_preprocess src/preprocess/ins_preprocess.cpp)
add_dependencies(lsdc_ins_preprocess ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lsdc_ins_preprocess ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})

add_executable(lsdc_flight_controller src/localization/flight_controller.cpp)
add_dependencies(lsdc_flight_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lsdc_flight_controller ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(lsdc_repub_gnss src/RTK/repub_gnss.cpp)
add_dependencies(lsdc_repub_gnss ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lsdc_repub_gnss ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})

add_executable(lsdc_rtk2pose src/RTK/rtk2pose.cpp)
add_dependencies(lsdc_rtk2pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lsdc_rtk2pose ${GeographicLib_LIBRARIES} ${catkin_LIBRARIES})

add_executable(lsdc_global_match src/localization/global_match.cpp)
add_dependencies(lsdc_global_match ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lsdc_global_match ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(lsdc_fusion_repub src/localization/fusion_repub.cpp)
add_dependencies(lsdc_fusion_repub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lsdc_fusion_repub ${PCL_LIBRARIES} ${catkin_LIBRARIES})

add_executable(lsdc_save_result src/LIO/save_result.cpp)
add_dependencies(lsdc_save_result ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lsdc_save_result ${catkin_LIBRARIES})
