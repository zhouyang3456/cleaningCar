##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.0)
project(demo_smarco_robot)

##############################################################################
# New add
##############################################################################
set(CMAKE_INCLUDE_CURRENT_DIR ON)

##############################################################################
# Catkin
##############################################################################

# qt_build provides the qt cmake glue, roscpp the comms for a default talker
find_package(catkin REQUIRED COMPONENTS qt_build
                                        roscpp
                                        std_msgs
                                        message_generation)

include_directories(${catkin_INCLUDE_DIRS})

##############################################################################
# my messages
##############################################################################
## 配置 msg 源文件
add_message_files(
  FILES
  stringVector.msg
)

# 生成消息时依赖于 std_msgs
generate_messages(
  DEPENDENCIES
  std_msgs
)

# Use this to define what the package will export (e.g. libs, headers).
# Since the default here is to produce only a binary, we don't worry about
# exporting anything. 
catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs message_runtime)

##############################################################################
# Qt Environment
##############################################################################

# this comes from qt_build's qt-ros.cmake which is automatically 
# included via the dependency call in package.xml
# rosbuild_prepare_qt4(QtCore QtGui) # Add the appropriate components to the component list here
find_package(Qt5 REQUIRED COMPONENTS Core Widgets Network)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets Qt5::Network)

##############################################################################
# Sections
##############################################################################

file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/demo_smarco_robot/*.hpp)

# QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT4_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
# QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})


##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/*.cpp src/tcp/*.cpp)

##############################################################################
# Binaries
##############################################################################

add_executable(demo_smarco_robot ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})

target_link_libraries(demo_smarco_robot ${QT_LIBRARIES}
                                        ${catkin_LIBRARIES}
                                        yaml-cpp
                                        jsoncpp)

target_include_directories(demo_smarco_robot PUBLIC
                           ${PROJECT_SOURCE_DIR}/../../devel/include
                           ${PROJECT_SOURCE_DIR}/include/demo_smarco_robot)

add_dependencies(demo_smarco_robot ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS demo_smarco_robot RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


message(STATUS "PROJECT_SOURCE_DIR = ${PROJECT_SOURCE_DIR}")
message(STATUS "PROJECT_BINARY_DIR = ${PROJECT_BINARY_DIR}")
message(STATUS "CMAKE_RUNTIME_OUTPUT_DIRECTORY = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
message(STATUS "CMAKE_LIBRARY_OUTPUT_DIRECTORY = ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
message(STATUS "CMAKE_ARCHIVE_OUTPUT_DIRECTORY = ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
