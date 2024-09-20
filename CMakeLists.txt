cmake_minimum_required(VERSION 3.5)
project(tm_rrt)

# set cpp version
set(CMAKE_CXX_STANDARD 20)
# -g gives information for the sigsegv-backtrace
add_compile_options(-g)

# find libraries
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

# package settings for ROS2
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

file(GLOB SOURCES RELATIVE ${CMAKE_SOURCE_DIR}
     "src/**/*.cpp" "src/**/*.h" "src/tm_rrt.cpp" "src/tm_rrt.h")
ament_auto_add_library(${PROJECT_NAME} ${SOURCES})

include_directories(
  /usr/lib/swi-prolog/include
)
target_link_libraries(${PROJECT_NAME}
  /usr/lib/swi-prolog/lib/x86_64-linux/libswipl.so
)

ament_auto_add_executable(${PROJECT_NAME}_node src/main.cpp)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

find_package(backward_ros REQUIRED)
find_package(Backward REQUIRED)
add_backward(${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  domains
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()

