add_subdirectory(Boost)
add_subdirectory(Catch2)
add_subdirectory(Eigen)
add_subdirectory(imgui)
add_subdirectory(SDL2)

add_library(third_party INTERFACE)

target_link_libraries(third_party INTERFACE Catch2 imgui Boost::math Eigen3::Eigen)
target_include_directories(third_party INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")
