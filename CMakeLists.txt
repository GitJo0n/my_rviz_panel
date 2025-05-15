cmake_minimum_required(VERSION 3.0.2)
project(my_rviz_panel)

## 반드시 Qt5 모듈들을 명시적으로 요청합니다.
find_package(Qt5 REQUIRED COMPONENTS Widgets Core Gui)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rviz
)

catkin_package()

## Qt 관련 설정
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

## include 디렉토리 추가
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

## 소스 추가
add_library(${PROJECT_NAME} src/my_rviz_panel.cpp)

## 링크에 Qt5 라이브러리 포함
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
  Qt5::Core
  Qt5::Gui
)

## plugin_description.xml 설치
install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 라이브러리 설치
install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
