add_library(Utils STATIC Utils/utils.hpp Utils/utils.cpp)

target_include_directories(Utils PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
