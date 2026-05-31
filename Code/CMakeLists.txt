cmake_minimum_required(VERSION 3.16)

# ESP-IDF auto-discovers components in `components/` — no EXTRA_COMPONENT_DIRS needed.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(blocking_focus)
