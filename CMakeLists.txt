cmake_minimum_required(VERSION 3.0.2)
project(yerin)

## 필요한 ROS 패키지
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  morai_msgs
)

## catkin 패키지 정의
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs morai_msgs
)

## include 디렉토리 경로
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include        # ✅ GeographicLib 헤더 경로
)

## 실행 파일
add_executable(main2 src/main2.cpp src/coord_utils.cpp)

## 라이브러리 링크
target_link_libraries(main2
  ${catkin_LIBRARIES}
  Geographic          # ✅ libGeographic.so 링크
)

## 빌드 종속성
add_dependencies(main2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
