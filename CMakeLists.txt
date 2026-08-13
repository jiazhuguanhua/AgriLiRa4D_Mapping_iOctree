cmake_minimum_required(VERSION 3.0.2)
project(mapping)

message(STATUS "Building mapping package...")
add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -s")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
set(PACKAGE_DEPENDENCIES
  geometry_msgs
  jsk_rviz_plugins
  nav_msgs
  octomap_msgs
  roscpp
  rosbag
  sensor_msgs
  tf2
  tf2_geometry_msgs
  pcl_conversions
  pcl_ros
)


find_package(catkin REQUIRED COMPONENTS ${PACKAGE_DEPENDENCIES})

# Find Eigen3
find_package(Eigen3 REQUIRED)

# Find PCL
find_package(PCL REQUIRED)

# Find YAML
find_package(yaml-cpp REQUIRED)

# Find Octomap
find_package(octomap REQUIRED)

set(IOCTREE_INCLUDE_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/i-octree/octree2
)
if(NOT EXISTS "${IOCTREE_INCLUDE_DIR}/Octree.h")
  message(FATAL_ERROR
    "i-Octree submodule is missing. Run: git submodule update --init --recursive")
endif()

catkin_package(CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES})


###########
## Build ##
###########
set(MAPPING_SRC
  src/main.cpp
  src/mapping.cpp
  src/options.cpp
  src/preprocess.cpp
  src/mapper/mapper.cpp
  src/mapper/mapper_octomap.cpp
  src/mapper/mapper_ioctree.cpp
)

add_executable(mapping_node ${MAPPING_SRC})
target_include_directories(mapping_node PUBLIC 
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${IOCTREE_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)
target_link_libraries(mapping_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  option(MAPPING_ENABLE_SANITIZERS
    "Enable AddressSanitizer and UndefinedBehaviorSanitizer for mapper tests"
    OFF)
  catkin_add_gtest(test_mapper_ioctree
    test/test_mapper_ioctree.cpp
    src/mapper/mapper.cpp
    src/mapper/mapper_ioctree.cpp
    src/mapper/mapper_octomap.cpp
    src/options.cpp
  )
  if(TARGET test_mapper_ioctree)
    target_include_directories(test_mapper_ioctree PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${IOCTREE_INCLUDE_DIR}
      ${catkin_INCLUDE_DIRS}
      ${EIGEN3_INCLUDE_DIRS}
      ${PCL_INCLUDE_DIRS}
      ${OCTOMAP_INCLUDE_DIRS}
    )
    target_link_libraries(test_mapper_ioctree
      ${catkin_LIBRARIES}
      ${PCL_LIBRARIES}
      ${OCTOMAP_LIBRARIES}
      ${YAML_CPP_LIBRARIES}
    )
    target_compile_definitions(test_mapper_ioctree PRIVATE
      MAPPING_TEST_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
    if(MAPPING_ENABLE_SANITIZERS)
      target_compile_options(test_mapper_ioctree PRIVATE
        -fsanitize=address,undefined -fno-omit-frame-pointer)
      target_link_libraries(test_mapper_ioctree
        -fsanitize=address,undefined)
    endif()
  endif()

  find_package(rostest REQUIRED)
  catkin_install_python(PROGRAMS test/ioctree_integration_test.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  add_rostest(test/ioctree_integration.test)
endif()
