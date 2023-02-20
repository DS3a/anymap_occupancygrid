cmake_minimum_required(VERSION 3.8)
project(anymap_occupancygrid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)


find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(grid_map REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_pcl REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)



### NOTE if you want to add python libs add them here
ament_python_install_package(python_lib)
### NOTE

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


### NOTE add Cpp nodes
add_executable(anymap_node src/anymap_node.cpp)
target_include_directories(anymap_node PUBLIC
### NOTE you can add additional include dirs for Cpp here
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
### NOTE
  $<INSTALL_INTERFACE:include>)
target_compile_features(anymap_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

### NOTE add dependencies to the cpp target
ament_target_dependencies(anymap_node
  "rclcpp"
  "std_msgs"
  "nav_msgs"
  "grid_map"
  "grid_map_core"
  "grid_map_ros"
  "grid_map_pcl"
  "geometry_msgs"
  "PCL"
  "pcl_ros"
  "pcl_conversions"
)
### NOTE


install(TARGETS anymap_node
  DESTINATION lib/${PROJECT_NAME})


### NOTE add python nodes
install(PROGRAMS
	scripts/node_py.py
	DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
