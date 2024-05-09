cmake_minimum_required(VERSION 3.16)

set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
set(MSVC_INCREMENTAL_DEFAULT ON)

project(
  template_arbml
  VERSION 2.3.7
  DESCRIPTION "MuJoCo Template Repository using ARBML libaray"
)

enable_language(C)
enable_language(CXX)
if(APPLE)
  enable_language(OBJC)
  enable_language(OBJCXX)
endif()



#################################
# Open Libraries ADD here
#################################
option(MUJOCO_BUILD_EXAMPLES "Build samples for MuJoCo" OFF)
option(MUJOCO_BUILD_SIMULATE "Build simulate library for MuJoCo" ON)
option(MUJOCO_BUILD_TESTS "Build tests for MuJoCo" OFF)
option(MUJOCO_TEST_PYTHON_UTIL "Build and test utility libraries for Python bindings" ON)
add_subdirectory(${PROJECT_SOURCE_DIR}/libraries/mujoco)                             # Add subdirectory
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/libraries/mujoco/sample/cmake") # CMake Path append
#################################
# Open Libraries ENDS here
#################################



# Check if we are building as standalone project.
set(SAMPLE_STANDALONE OFF)
set(_INSTALL_SAMPLES ON)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(SAMPLE_STANDALONE ON)
  # If standalone, do not install the samples.
  set(_INSTALL_SAMPLES OFF)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(SAMPLE_STANDALONE)
  include(SampleOptions)
else()
  enforce_mujoco_macosx_min_version()
endif()
include(SampleDependencies)

set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${AVX_COMPILE_OPTIONS}" "${EXTRA_COMPILE_OPTIONS}")
set(MUJOCO_SAMPLE_LINK_OPTIONS "${EXTRA_LINK_OPTIONS}")

if(MUJOCO_HARDEN)
  if(WIN32)
    set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -O3 -Wl,/DYNAMICBASE)
  else()
    set(MUJOCO_SAMPLE_COMPILE_OPTIONS "${MUJOCO_SAMPLE_COMPILE_OPTIONS}" -O3 -fPIE)
    if(APPLE)
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -O3 -Wl,-pie)
    else()
      set(MUJOCO_SAMPLE_LINK_OPTIONS "${MUJOCO_SAMPLE_LINK_OPTIONS}" -O3 -pie)
    endif()
  endif()
endif()


#################################
# Custom Sources STARTS from here
#################################
find_package(Eigen3 REQUIRED)

include_directories(
    include
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    ${EIGEN3_INCLUDE_DIR}
    DEPENDS system_lib
)

set(ARBML_SRC 
  include/ARBMLlib/ARBML.cpp
  include/ARBMLlib/RigidBody.cpp
)

add_executable(template 
  src/main.cpp 
  src/Robot_Control.cpp 
  ${SIMULATE_RESOURCE_FILES} 
  ${ARBML_SRC}
)
target_include_directories(template PUBLIC
  ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(template
  libsimulate
  mujoco::mujoco
  glfw
  Threads::Threads
  lodepng
)
target_compile_options(template PUBLIC ${MUJOCO_SIMULATE_COMPILE_OPTIONS})
#################################
# Custom Sources ENDS here
#################################


if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
  embed_in_bundle(template)
endif()

# Do not install if macOS Bundles are created as RPATH is managed manually there.
if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
  set(_INSTALL_SAMPLES OFF)
endif()