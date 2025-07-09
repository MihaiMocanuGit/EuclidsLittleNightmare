add_library(SceneImporter STATIC SceneImporter/SceneImporter.hpp SceneImporter/SceneImporter.cpp)

target_include_directories(SceneImporter PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
