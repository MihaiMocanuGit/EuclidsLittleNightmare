add_library(stb_image STATIC header_src/stb_image.h stb_image.h)

set_target_properties(stb_image PROPERTIES LINKER_LANGUAGE C)

target_include_directories(stb_image PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
