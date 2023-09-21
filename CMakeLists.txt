cmake_minimum_required(VERSION 3.0.2)
project(tailored_mpc)

#add_compile_options(-std=c++0x)
# set(CMAKE_CXX_FLAGS "-fpermissive -fopenmp -std=c++11")
# set(CMAKE_CXX_FLAGS "-fpermissive -std=c++11")
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

find_package(Eigen3 REQUIRED)

find_path(CASADI_INCLUDE_DIR
  casadi/casadi.hpp
  HINTS ${HOME}/casadi/
)

# find_package(OpenMP REQUIRED)
# if(OPENMP_FOUND)
#   message(STATUS "OPENMP FOUND")
#   set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
#   set(OpenMP_LIBS gomp)
# else()
#   message(WARNING "OPENMP NOT FOUND, multithread isn't available")
# endif()

if(CASADI_INCLUDE_DIR)
  set(CASADI_INCLUDE_DIR ${CASADI_INCLUDE_DIR} ${CASADI_INCLUDE_DIR}/casadi)
  set(CASADI_FOUND_INCLUDE TRUE)
  message(STATUS "Found CasADi include dir: ${CASADI_INCLUDE_DIR}")
else()
  message(STATUS "Could not find CasADi include dir")
endif()

if(CASADI_FOUND_INCLUDE AND CASADI_LIBRARIES)
  set(CASADI_FOUND TRUE)
endif()

find_library(CASADI_LIBRARY 
    NAMES casadi 
    PATHS ${HOME}/casadi/build/lib/
    )
if(CASADI_LIBRARY)
    set(CASADI_LIBRARIES ${CASADI_LIBRARIES} ${CASADI_LIBRARY})
endif()

if(CASADI_LIBRARIES)
  message(STATUS "Found CasADi libs: ${CASADI_LIBRARIES}")
else()
  message(STATUS "Could not find CasADi libs")
endif()

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
  src/utils/optimizer.cpp
  src/mpc.cpp

)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${CASADI_LIBRARY})
target_link_libraries(${PROJECT_NAME}_exec ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

# Mandatory for parallelism (OpenMP)
# target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_FLAGS})
