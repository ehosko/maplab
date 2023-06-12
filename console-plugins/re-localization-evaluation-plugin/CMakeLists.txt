cmake_minimum_required (VERSION 2.8)
project(re_localization_evaluation_plugin)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-fPIC -shared)

cs_add_library(${PROJECT_NAME} src/re-localization-evaluation-plugin.cc)
create_console_plugin(${PROJECT_NAME})

##################
# CMAKE Indexing #
##################
# TODO: (michbaum) Not 100% sure if this is still needed
FILE(GLOB_RECURSE LibFiles "include/*")
add_custom_target(headers SOURCES ${LibFiles})

cs_install()
cs_export()
