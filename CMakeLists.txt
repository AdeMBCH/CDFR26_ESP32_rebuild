cmake_minimum_required(VERSION 3.16.0)

# micro-ROS (Humble) component path.
# The component is installed by cloning:
#   components/micro_ros_espidf_component
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/components/micro_ros_espidf_component
)

if(NOT EXISTS "${CMAKE_CURRENT_LIST_DIR}/components/micro_ros_espidf_component/CMakeLists.txt")
    message(WARNING "micro-ROS component not found. Run the install commands from README / assistant instructions.")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(CDFR26_ESP32_rebuild)
