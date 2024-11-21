cmake_minimum_required(VERSION 3.10)
project(coinft_class)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)

# Set the CMP0167 policy
cmake_policy(SET CMP0167 NEW)

# Include FetchContent module
include(FetchContent)

# Declare and fetch Boost
FetchContent_Declare(
    boost
    URL https://boostorg.jfrog.io/artifactory/main/release/1.86.0/source/boost_1_86_0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(boost)

# Find Boost libraries
find_package(Boost REQUIRED COMPONENTS system thread)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# Include the Boost headers
include_directories(${Boost_INCLUDE_DIRS})

# Include your project's header files
include_directories(include)

# Add the source files
add_executable(coinft_test src/coinft_interface.cpp src/CoinFT.cpp)

# Link against Boost libraries
target_link_libraries(coinft_test ${Boost_LIBRARIES})
target_link_libraries(coinft_test Eigen3::Eigen)


