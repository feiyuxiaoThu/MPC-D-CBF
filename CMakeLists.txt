cmake_minimum_required(VERSION 3.10)
project(local_plannerc)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 对应Python casadi依赖 - 非线性优化库
find_package(casadi REQUIRED)

set(CASADI_INCLUDE_DIR /usr/local/include/casadi)

# find casadi library
find_library(CASADI_LIBRARY
    NAMES casadi
    HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

# 对应Python numpy依赖 - 线性代数库  
find_package(Eigen3 REQUIRED)

# 查找Python依赖以使用matplotlib-cpp
find_package(PythonLibs 3 REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})


# 包含目录
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${CASADI_INCLUDE_DIR}
)


# 添加编译器标志
add_compile_options(-O3 -Wall -Wextra)

# 创建库文件
add_library(${PROJECT_NAME}
  src/local_planner_linear.cpp
)

# 链接库依赖
target_link_libraries(${PROJECT_NAME}
  ${CASADI_LIBRARIES}
)

# 创建可执行文件 - 对应Python的main执行
add_executable(local_planner_mpc 
  src/main.cpp
)

# 链接可执行文件
target_link_libraries(local_planner_mpc
  ${PROJECT_NAME}
  ${CASADI_LIBRARIES}
  ${PYTHON_LIBRARIES}
)
