cmake_minimum_required(VERSION 2.8 FATAL_ERROR)


add_subdirectory(yarp_model_move)
find_package(OcraIcub QUIET)
IF(${OcraIcub_FOUND})
    add_subdirectory(ocra_task_widget)
ENDIF()
add_subdirectory(ocra_gui_plugin)
add_subdirectory(yarp_camera_control)
