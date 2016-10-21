cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(YARP REQUIRED)

include_directories(
                    ${YARP_INCLUDE_DIRS}
                    )

link_directories(
                ${YARP_LIBRARY_DIRS}
                )

add_executable(test_yarp_model_plugin YarpModelPluginTest.cc)
target_link_libraries(test_yarp_model_plugin ${YARP_LIBRARIES})

install(TARGETS test_yarp_model_plugin DESTINATION bin)
