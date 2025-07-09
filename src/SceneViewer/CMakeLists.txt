add_library(SceneViewer STATIC SceneViewer/SceneViewer.hpp SceneViewer/SceneViewer.cpp)

target_include_directories(SceneViewer PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
