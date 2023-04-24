cmake_minimum_required(VERSION 3.0.2)
project(tailored_mpc)

#add_compile_options(-std=c++0x)
set(CMAKE_CXX_FLAGS "-fpermissive -fopenmp -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

message(STATUS "TAILORED MPC says hi :)")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  visualization_msgs
  as_msgs
)

find_package(OpenMP REQUIRED)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
  set(OpenMP_LIBS gomp)
else()
  message(WARNING "OPENMP NOT FOUND, multithread isn't available")
endif()

find_library(FORCESPRO_LIBRARY libTailoredSolver.a PATHS ${PROJECT_SOURCE_DIR}/solver/codeGen/TailoredSolver/lib_target)

generate_dynamic_reconfigure_options(
  cfg/dynamic.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_exec src/main.cpp)

add_library(${PROJECT_NAME}
  src/utils/params.cpp
  src/utils/vis_tools.cpp
  src/mpc.cpp

  solver/codeGen/TailoredSolver_casadi.c
  solver/codeGen/TailoredSolver_adtool2forces.c
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${FORCESPRO_LIBRARY})
target_link_libraries(${PROJECT_NAME}_exec ${catkin_LIBRARIES} ${PROJECT_NAME})

# add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

# Mandatory for parallelism (OpenMP)
target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})
