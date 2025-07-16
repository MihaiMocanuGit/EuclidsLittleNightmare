add_subdirectory(GUI)
add_subdirectory(Scene)
add_subdirectory(Utils)

add_library(src INTERFACE)

target_link_libraries(src INTERFACE GUI Scene Utils)

target_include_directories(src INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
