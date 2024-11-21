cmake_minimum_required(VERSION 3.10)
project(coinft_class)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Set the CMP0167 policy
cmake_policy(SET CMP0167 NEW)

# Include the Boost headers
include_directories(${Boost_INCLUDE_DIRS})

# Include your project's header files
include_directories(include)

# Add the source files
add_library(coinft_class src/coin_ft.cpp)
target_link_libraries(coinft_class ${Boost_LIBRARIES} Eigen3::Eigen)

add_executable(coinft_test src/coinft_interface.cpp)
target_link_libraries(coinft_test coinft_class)

# Install the library
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS ATI_NETFT
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

