set(BOOST_INCLUDE_LIBRARIES container lockfree) # for now, math is only included to check if linking with boost works.
set(BOOST_ENABLE_CMAKE ON)
include(FetchContent)
FetchContent_Declare(
  Boost
  GIT_REPOSITORY https://github.com/boostorg/boost.git
  GIT_TAG boost-1.88.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE

)
FetchContent_MakeAvailable(Boost)
