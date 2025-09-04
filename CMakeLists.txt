cmake_minimum_required(VERSION 3.0.2)
project(my_rviz_panel)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)
include(/usr/share/OGRE/cmake/modules/FindOGRE.cmake)
if(NOT Ogre_FOUND)
  message(FATAL_ERROR "Failed to find Ogre library using FindOGRE.cmake script.")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rviz
)

catkin_package()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
  ${Ogre_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/Control_Keyboard.cpp
  src/KOREATECH.cpp
  src/RealTimeClockPanel.cpp
  src/CooldownPanel.cpp
  src/compass_panel.cpp
  src/compass_widget.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  ${Ogre_LIBRARIES}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resources/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/resources
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
