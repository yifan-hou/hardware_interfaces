cmake_minimum_required(VERSION 3.10)
project(coinft_class)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ONNXRUNTIME_ROOT "/opt/onnxruntime")

include_directories(${ONNXRUNTIME_ROOT}/include)
link_directories(${ONNXRUNTIME_ROOT}/lib)
find_library(ONNXRUNTIME_LIB onnxruntime HINTS ${ONNXRUNTIME_ROOT}/lib)

# Include the Boost headers
include_directories(${Boost_INCLUDE_DIRS})

# Include your project's header files
include_directories(include)

# Add the source files
add_library(COINFTBUS SHARED src/CoinFTBus.cpp)
target_link_libraries(COINFTBUS ${Boost_LIBRARIES} ${ONNXRUNTIME_LIB})


add_library(COIN_FT SHARED src/coin_ft.cpp)
target_link_libraries(COIN_FT ${RUT} ${YAMLLib} Eigen3::Eigen COINFTBUS)

add_executable(coinft_test src/coinft_test.cpp)
target_link_libraries(coinft_test COIN_FT ${RUT})

# Install the library
install(DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/
)
install(TARGETS COINFTBUS COIN_FT
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/
)

