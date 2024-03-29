cmake_minimum_required(VERSION 2.8.3)
project(drone)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
                    roscpp rviz
                    message_runtime std_msgs sensor_msgs geometry_msgs
                    image_transport
                    message_generation
		    ardrone_autonomy
                    #crosbot_msgs
                    #crosbot
                    ## Uncomment these if required
                    cv_bridge
                    #crosbot_explore
                    )

## Uncomment if using OpenCV
##OpenCV
find_package(OpenCV)

# QT4 required for RViz plugins
find_package(Qt4 REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  beaconGeometry.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#   LIBRARIES
#   CATKIN_DEPENDS
#  DEPENDS system_lib
    CATKIN_DEPENDS message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
##    including locations and requirements for Qt
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

## Add generated MOC Qt header files
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} include/rviz/**.hpp)
QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

## Starter library with all common code
add_library(drone_library
    src/controlPanelTest.cpp
    src/bottomBeaconDetector.cpp
    src/altitudeControl.cpp
    #src/other.cpp
)

## GUI Specific Libraries
add_library(drone_rviz_library
    src/rviz/controlPanel.cpp
    ${QT_MOC_HPP}
)

## Declaring all Nodes
add_executable(drone_controlPanelTest
    src/nodes/controlPanelTest_node.cpp
)
add_executable(drone_bottomBeaconDetector
    src/nodes/bottomBeaconDetector_node.cpp
)
add_executable(drone_altitudeControl
    src/nodes/altitudeControl_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(drone_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
target_link_libraries(drone_controlPanelTest
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  drone_library
)
target_link_libraries(drone_bottomBeaconDetector
  ${CATKIN_LIBRARIES}
  ${OpenCV_LIBRARIES}
  drone_library
)
target_link_libraries(drone_altitudeControl
  ${CATKIN_LIBRARIES}
  ${OpenCV_LIBRARIES}
  drone_library
)
target_link_libraries(drone_rviz_library
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${QT_LIBRARIES}
  drone_library
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
	src/nodes/drone_controller.py
	src/nodes/autonom_controller.py
	#src/pattern_detection_node.py
#   scripts/my_python_script
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
#  PATTERN ".svn" EXCLUDE
#)

## Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

