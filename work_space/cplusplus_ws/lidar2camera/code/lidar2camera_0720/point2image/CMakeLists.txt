cmake_minimum_required(VERSION 2.8)

project(point2image)
# 它会引入两个变量 demo_BINARY_DIR 和 demo_SOURCE_DIR， 同时，cmake 自动定义了两个等价的变量
# PROJECT_BINARY_DIR 和 PROJECT_SOURCE_DIR

set(CMAKE_BUILD_TYPE "Debug") # Debug调试模式，运行会慢很多,使得生成的可执行文件可以调试  "Release": 编译运行，比较高效
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
set(CMAKE_CXX_FLAGS "-Wall") # 开启所有警告

# set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)    # 设置生成的库文件（动态库和静态库）的输出路径
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin) # 设置生成的可执行文件的输出路径
set(PROJECT_INCLUDE ./include)
include_directories(${PROJECT_INCLUDE}) # 添加头文件目录

# find_package(Boost REQUIRED)
# include_directories(${Boost_INCLUDE_DIRS})
# link_directories(${Boost_LIBRARY_DIRS})
# add_definitions(${Boost_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS}) 

find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
link_directories(${OpenCV_LIBRARY_DIRS})
add_definitions(${OpenCV_INCLUDE_DIRS})

find_package(roscpp REQUIRED)
include_directories( ${roscpp_INCLUDE_DIRS} )
link_directories(${roscpp_LIBRARY_DIRS})
add_definitions(${roscpp_INCLUDE_DIRS})

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
link_directories(${YAML_CPP_LIBRARY_DIRS})
add_definitions(${YAML_CPP_INCLUDE_DIR})


# set(point2image ./src/main.cpp)
# add_executable(point2image ${point2image})
file(GLOB SRC_FILE "./src/main.cpp" "./src/point2image.cpp")             # 建立变量SRC_FILE为目录下的所有.cpp文件列表
add_executable(point2image ${SRC_FILE})       # 生成可执行的文件
target_link_libraries(point2image ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
${OpenCV_LIBS} ${roscpp_LIBRARIES} ${YAML_CPP_LIBRARIES} ${Boost_LIBRARIES})