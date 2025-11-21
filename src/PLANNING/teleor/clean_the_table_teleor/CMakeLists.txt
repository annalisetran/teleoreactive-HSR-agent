cmake_minimum_required(VERSION 3.0.2)
project(clean_the_table_teleor)

## Find catkin and required packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  unsw_action_msg
  message_generation
)

## Manually set PostgreSQL paths
set(PostgreSQL_INCLUDE_DIRS "/usr/include/postgresql")
set(PostgreSQL_LIBRARIES "pqxx;pq")

## Set QuProlog paths from environment variable
if(DEFINED ENV{QUPROLOG_ROOT})
  set(QUPROLOG_ROOT $ENV{QUPROLOG_ROOT})
  message(STATUS "Using QuProlog from: ${QUPROLOG_ROOT}")
else()
  message(FATAL_ERROR "QUPROLOG_ROOT environment variable not set.")
endif()

set(QUPROLOG_INCLUDE_DIR "${QUPROLOG_ROOT}/src")

## Find qcc compiler
find_program(QCC_COMPILER qcc HINTS ${QUPROLOG_ROOT}/bin)
if(NOT QCC_COMPILER)
  message(FATAL_ERROR "qcc compiler not found. Make sure QuProlog/bin is in your PATH.")
else()
  message(STATUS "Found qcc: ${QCC_COMPILER}")
endif()

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp std_msgs unsw_action_msg
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PostgreSQL_INCLUDE_DIRS}
  ${QUPROLOG_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

## Build include flags for compilation with absolute paths
set(INCLUDE_FLAGS
  "-I${QUPROLOG_INCLUDE_DIR}"
  "-I${CMAKE_CURRENT_SOURCE_DIR}/include"
)

# Get absolute path for devel include (converts ~ and relative paths)
get_filename_component(DEVEL_INCLUDE_ABS "${CATKIN_DEVEL_PREFIX}/include" ABSOLUTE)
list(APPEND INCLUDE_FLAGS "-I${DEVEL_INCLUDE_ABS}")

foreach(dir ${catkin_INCLUDE_DIRS})
  get_filename_component(dir_abs ${dir} ABSOLUTE)
  list(APPEND INCLUDE_FLAGS "-I${dir_abs}")
endforeach()

list(APPEND INCLUDE_FLAGS "-I${PostgreSQL_INCLUDE_DIRS}")

## Compile clean_the_table_foreign.cpp with g++
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/ql/clean_the_table_foreign.o
  COMMAND g++ -DNDEBUG -c 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/clean_the_table_foreign.cpp
    ${INCLUDE_FLAGS}
    -fPIC
    -o ${CMAKE_CURRENT_SOURCE_DIR}/ql/clean_the_table_foreign.o
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/clean_the_table_foreign.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/include/clean_the_table_foreign.hpp
  COMMENT "Compiling clean_the_table_foreign.cpp with g++"
  VERBATIM
)

## Compile clean_the_table_interface.cpp with qcc
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/ql/clean_the_table_interface.o
  COMMAND ${QCC_COMPILER} -DNDEBUG
    ${CMAKE_CURRENT_SOURCE_DIR}/src/clean_the_table_interface.cpp
    ${INCLUDE_FLAGS}
    -fPIC
    -o ${CMAKE_CURRENT_SOURCE_DIR}/ql/clean_the_table_interface.o
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/clean_the_table_interface.cpp
          ${CMAKE_CURRENT_SOURCE_DIR}/include/clean_the_table_foreign.hpp
  COMMENT "Compiling clean_the_table_interface.cpp with qcc"
  VERBATIM
)

## Create a custom target that depends on both .o files
add_custom_target(quprolog_objects ALL
  DEPENDS 
    ${CMAKE_CURRENT_SOURCE_DIR}/ql/clean_the_table_foreign.o
    ${CMAKE_CURRENT_SOURCE_DIR}/ql/clean_the_table_interface.o
)

## Make sure messages are built first
add_dependencies(quprolog_objects ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/ql/clean_the_table_foreign.o
  ${CMAKE_CURRENT_SOURCE_DIR}/ql/clean_the_table_interface.o
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ql
)

install(FILES
  include/clean_the_table_foreign.hpp
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY ql/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ql
  FILES_MATCHING PATTERN "*.qlg" PATTERN "*.ql"
)