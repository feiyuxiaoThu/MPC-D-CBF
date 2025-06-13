cmake_minimum_required(VERSION 3.10)
project(local_plannerc)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找ROS包 - 对应Python的rospy等ROS依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs  
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_geometry_msgs
)

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

# catkin包配置
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs visualization_msgs tf2 tf2_geometry_msgs
  DEPENDS EIGEN3
)


# 包含目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  SYSTEM ${CASADI_INCLUDE_DIR}
)

catkin_package(
  INCLUDE_DIRS include
)

# 添加编译器标志
add_compile_options(-O3 -Wall -Wextra)

# 创建库文件
add_library(${PROJECT_NAME}
  src/local_planner.cpp
)

# 链接库依赖
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CASADI_LIBRARIES}
)

# 创建可执行文件 - 对应Python的main执行
add_executable(local_planner_node 
  src/main.cpp
)

# 链接可执行文件
target_link_libraries(local_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CASADI_LIBRARIES}
)

# 添加依赖关系
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(local_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 安装规则
install(TARGETS ${PROJECT_NAME} local_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# 安装launch文件和配置文件
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  PATTERN ".svn" EXCLUDE)


  # 添加Controller可执行文件
add_executable(controller_node
src/controller.cpp
src/controller_main.cpp
)

target_link_libraries(controller_node
${catkin_LIBRARIES}
${EIGEN3_LIBRARIES}
)

# 添加依赖关系
add_dependencies(controller_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 更新安装规则
install(TARGETS ${PROJECT_NAME} local_planner_node controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}  
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)