cmake_minimum_required(VERSION 3.10)
project(PointCloudManager)

# 1. 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. 寻找 PCL 库 (指定需要的模块)
find_package(PCL 1.8 REQUIRED COMPONENTS common io filters visualization)

# 3. 包含头文件目录
include_directories(
    include
    ${PCL_INCLUDE_DIRS}
)

# 4. 找到所有的源文件
file(GLOB SOURCES "src/*.cpp")

# 5. 生成可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 6. 链接 PCL 的库文件
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

# 7. 编译定义（PCL 有时需要）
add_definitions(${PCL_DEFINITIONS})