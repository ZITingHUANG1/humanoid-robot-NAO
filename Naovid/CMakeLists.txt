cmake_minimum_required(VERSION 2.8.3)
project(Naovid)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  naoqi_bridge_msgs
  std_srvs
  actionlib_msgs
  geometry_msgs
  rospy
  message_generation
  sensor_msgs
  tf
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED COMPONENTS)
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(aruco REQUIRED)

add_action_files(
    FILES
    )



add_service_files(
    DIRECTORY srv
    FILES
    MoveJoints.srv
    MoveJoints2.srv
    CertificateCheck.srv
    Standup.srv
    MoveHead.srv
    AniSpeech.srv
    Aruco.srv
)

set(CMAKE_CXX_STANDARD 11)
generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    DEPENDENCIES std_msgs
    )


catkin_package(
  CATKIN_DEPENDS message_runtime)
catkin_package(CATKIN_DEPENDS actionlib_msgs)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(naovid_library src/covid_check.cpp src/teleoperation.cpp src/shirtdetection_visitor.cpp src/facedetection.cpp src/speech.cpp src/arucomarker.cpp)


catkin_install_python(PROGRAMS ./script/move_service.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    
)

add_executable(nao_1 src/client.cpp)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(nao_1 ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(nao_1 ${catkin_LIBRARIES} naovid_library ${catkin_LIBRARIES})
target_link_libraries(nao_1 sfml-graphics sfml-system sfml-window boost_system pthread ${ZBAR_LIBRARIES} zbar ${aruco_LIBRARIES} ${OpenCV_LIBRARIES})

#add_executable(shirtdetection src/shirtdetection_standalone.cpp)
#target_link_libraries(shirtdetection ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
