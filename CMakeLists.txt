cmake_minimum_required(VERSION 2.8.3)
project(yerin)

## catkin + 필요한 ROS 패키지 찾기
find_package(catkin REQUIRED COMPONENTS 
  message_generation
  std_msgs
  roscpp
  sensor_msgs
  morai_msgs
  geodesy
  geographic_msgs
)

## 메시지를 쓰는 경우 (필요 시만 활성화)
# add_message_files(FILES yerin.msg)
# generate_messages(DEPENDENCIES std_msgs)

## catkin 패키지 설정
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS message_runtime std_msgs roscpp sensor_msgs morai_msgs geodesy geographic_msgs
)

## include 디렉토리
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include                # GeographicLib 헤더 포함
)

## 실행 파일 빌드
add_executable(main src/main.cpp)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## 라이브러리 연결
target_link_libraries(main
  ${catkin_LIBRARIES}
  Geographic                  # libGeographic.so 링크
)
