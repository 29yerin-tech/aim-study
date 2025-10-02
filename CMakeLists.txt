cmake_minimum_required(VERSION 3.0.2)
project(yerin)

## 필요한 ROS 패키지
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  morai_msgs
  geodesy
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

## 🔹 main2는 현재 사용하지 않으므로 주석 처리
# add_executable(main2 src/main2.cpp src/coord_utils.cpp)

# target_link_libraries(main2
#   ${catkin_LIBRARIES}
#   Geographic          # ✅ libGeographic.so 링크
# )

# add_dependencies(main2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 🔹 path_recorder 실행 파일 
add_executable(path_recorder src/path.cpp src/coord_utils.cpp)
target_link_libraries(path_recorder
  ${catkin_LIBRARIES}
  Geographic
)
add_dependencies(path_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
