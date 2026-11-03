cmake_minimum_required(VERSION 3.16)
# Offizielle Displayquellen werden vom lokalen Installationsskript bereitgestellt.
set(display_deps "${CMAKE_CURRENT_LIST_DIR}/.local/display-dependencies")
if(EXISTS "${display_deps}/lvgl/CMakeLists.txt")
    list(APPEND EXTRA_COMPONENT_DIRS "${display_deps}/lvgl")
endif()
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(heizungssteuerung)
