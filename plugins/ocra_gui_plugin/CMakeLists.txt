cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(YARP REQUIRED)



include_directories(${GAZEBO_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    )

link_directories(${GAZEBO_LIBRARY_DIRS}
                ${YARP_LIBRARY_DIRS}
                )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(ocra_gui_plugin SHARED OcraGuiPlugin.cc)
target_link_libraries(ocra_gui_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${YARP_LIBRARIES})

install(TARGETS ocra_gui_plugin DESTINATION lib)
