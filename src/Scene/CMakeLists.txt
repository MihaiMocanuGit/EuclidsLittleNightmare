add_subdirectory(Scene/TreeBVH)

add_library(Scene STATIC Scene/Scene.hpp Scene/Scene.cpp)

target_link_libraries(Scene PUBLIC TreeBVH)

target_include_directories(Scene PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
