cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
find_package(OcraIcub REQUIRED)
IF(${OcraIcub_FOUND})
    message("-- Found OcraIcub version ${OcraIcub_VERSION}")
ENDIF()


include_directories(${GAZEBO_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${OcraIcub_INCLUDE_DIRS})

link_directories(   ${GAZEBO_LIBRARY_DIRS}
                    ${YARP_LIBRARY_DIRS}
                    )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


file(GLOB_RECURSE source_files *.cc)
file(GLOB_RECURSE header_files *.hh )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt4)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(YARP REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

QT4_WRAP_CPP(headers_MOC OcraTaskWidget.hh)

add_library(ocra_task_widget SHARED ${headers_MOC} ${source_files} ${header_files})
target_link_libraries(ocra_task_widget ${GAZEBO_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES}
    ${Boost_LIBRARIES}
    ${YARP_LIBRARY_DIRS}
    ${OcraIcub_LIBRARIES}
)

install(TARGETS ocra_task_widget DESTINATION lib)
