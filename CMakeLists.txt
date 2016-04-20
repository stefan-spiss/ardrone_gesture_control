cmake_minimum_required(VERSION 2.8.3)
project(ardrone_gesture_control)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  ardrone_autonomy
  roscpp
  rospy
  std_msgs
  std_srvs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
	CATKIN_DEPENDS
    roscpp
    rospy
    geometry_msgs
    std_msgs
    std_srvs
    ardrone_autonomy
)

###########
## Build ##
###########
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a cpp library
#add_library(${PROJECT_NAME}
#  src/OculusObserver.cpp
#  src/MyoObserver.cpp
#)

set(SOURCES_OMC
  src/OculusObserver.cpp 
  src/MyoObserver.cpp
  src/oculus_myo_control.cpp
)

set(SOURCES_LMOC
  src/OculusObserver.cpp 
  src/MyoObserver.cpp
  src/LeapObserver.cpp
  src/leap_myo_oculus_control.cpp
)

set(SOURCES_LMOAC
  src/OculusObserver.cpp 
  src/MyoObserver.cpp
  src/LeapObserver.cpp
  src/LeapMyoOculusDroneControl.cpp
  src/leap_myo_oculus_advanced_control.cpp
)

set(SOURCES_JOC
  src/OculusObserver.cpp 
  src/GamepadObserver.cpp
  src/JoypadOculusControl.cpp
  src/joypad_oculus_control.cpp
)

## Declare a cpp executable
add_executable(myo_control src/myo_control.cpp)
add_executable(oculus_myo_control ${SOURCES_OMC})
add_executable(leap_myo_oculus_control ${SOURCES_LMOC})
add_executable(advanced_lmo_control ${SOURCES_LMOAC})
add_executable(joypad_oculus_control ${SOURCES_JOC})


## Specify libraries to link a library or executable target against
target_link_libraries(myo_control
   ${catkin_LIBRARIES}
)
target_link_libraries(oculus_myo_control
   ${catkin_LIBRARIES}
)
target_link_libraries(leap_myo_oculus_control
   ${catkin_LIBRARIES}
)
target_link_libraries(advanced_lmo_control
   ${catkin_LIBRARIES}
)
target_link_libraries(joypad_oculus_control
   ${catkin_LIBRARIES}
)