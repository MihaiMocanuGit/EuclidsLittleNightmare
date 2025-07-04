add_library(GUI STATIC GUI/GUI.hpp GUI/GUI.cpp)

target_link_libraries(GUI PRIVATE imgui)

target_include_directories(GUI PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
