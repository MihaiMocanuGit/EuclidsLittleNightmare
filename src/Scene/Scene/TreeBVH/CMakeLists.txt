add_library(TreeBVH STATIC
        TreeBVH.hpp TreeBVH.cpp
)

target_include_directories(TreeBVH
        PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../..")

target_link_options(TreeBVH PUBLIC -ltbb)
target_link_libraries(TreeBVH PUBLIC third_party Utils)

target_sources(unit_tests PUBLIC unit_test.cpp)
