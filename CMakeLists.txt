### 1 - Required CMake Version
cmake_minimum_required(VERSION 2.8.3)
### 2- Package Name
project(rosxwiimote)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra") # add extra warnings
### 3 - Find other CMake/Catkin packages needed for build
set(DEPS roscpp roslib std_msgs)
find_package(catkin REQUIRED COMPONENTS ${DEPS})
### 4 - Message/Service/Action Generators (add_message_files(), add_service_files(), add_action_files())
# nothing to do
### 5 - Invoke message/service/action generation (generate_messages())
## Generate added messages and services with any dependencies listed here
### 6 - Specify package build info export (catkin_package())
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS ${DEPS}
)
### 7 - Libraries/Executables to build (add_library()/add_executable()/target_link_libraries())
include_directories(${catkin_INCLUDE_DIRS})
add_executable(xwiimote_node.exe         src/xwiimote_node.cpp)
target_link_libraries(xwiimote_node.exe  ${catkin_LIBRARIES}  xwiimote)
### 8 - Tests to build (catkin_add_gtest())
# nothing to do
### 9 - Install rules (install())
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)
