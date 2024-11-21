cmake_minimum_required(VERSION 3.10)
project(coinft_class)

# Include the Boost headers
include_directories(${Boost_INCLUDE_DIRS})

# Include your project's header files
include_directories(include)

# Add the source files
add_library(COIN_FT src/coin_ft.cpp)
target_link_libraries(COIN_FT ${Boost_LIBRARIES} Eigen3::Eigen)

add_executable(coinft_test src/coinft_test.cpp)
target_link_libraries(coinft_test COIN_FT)

# Install the library
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS COIN_FT
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

