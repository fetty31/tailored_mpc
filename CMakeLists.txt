cmake_minimum_required(VERSION 3.0.2)
project(tailored_mpc)

#add_compile_options(-std=c++0x)
set(CMAKE_CXX_FLAGS "-fpermissive")
# set(CMAKE_BUILD_TYPE "Release")

message(STATUS "TAILORED MPC says hi :)")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  dynamic_reconfigure
  as_msgs
)

find_path(CASADI_INCLUDE_DIR
  casadi/casadi.hpp
  HINTS ${HOME}/casadi/
)

message(${PROJECT_SOURCE_DIR})
find_library(FORCESPRO_LIBRARY libTailoredSolver.a PATHS ${PROJECT_SOURCE_DIR}/solver/codeGen/TailoredSolver/lib_target)

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
  src/structures/params.cpp
  src/optimizer.cpp
  src/mpc.cpp

  solver/codeGen/TailoredSolver_casadi.c
  solver/codeGen/TailoredSolver_adtool2forces.c
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${FORCESPRO_LIBRARY} ${CASADI_LIBRARY})
target_link_libraries(${PROJECT_NAME}_exec ${catkin_LIBRARIES} ${PROJECT_NAME})

add_dependencies(${PROJECT_NAME} as_msgs_generate_messages_cpp)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
