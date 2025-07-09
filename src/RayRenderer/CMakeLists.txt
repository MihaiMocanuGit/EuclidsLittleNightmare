add_library(RayRenderer STATIC RayRenderer/RayRenderer.hpp RayRenderer/RayRenderer.cpp)

target_include_directories(RayRenderer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
