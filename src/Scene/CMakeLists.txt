add_library(Scene STATIC Scene/Scene.hpp Scene/Scene.cpp)

target_include_directories(Scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
