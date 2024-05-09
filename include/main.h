//
//  Copyright 2021 DeepMind Technologies Limited
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
  #include <windows.h>
#else
  #if defined(__APPLE__)
    #include <mach-o/dyld.h>
  #endif
    #include <dirent.h>
    #include <dlfcn.h>
    #include <sys/errno.h>
    #include <unistd.h>
#endif
}

namespace {
    namespace mj = ::mujoco;
    namespace mju = ::mujoco::sample_util;

    // using ::mujoco::Glfw;

    // Constants
    const double syncMisalign = 0.1;        // Maximum mis-alignment before re-sync (simulation seconds)
    const double simRefreshFraction = 0.7;  // Fraction of refresh available for simulation
    const int kErrorLength = 1024;          // Load error string length

    // Model and Data
    mjModel* m = nullptr;
    mjData* d = nullptr;

    // Control noise variables
    mjtNum* ctrlnoise = nullptr;

    using Seconds = std::chrono::duration<double>;
}

//////////////////// USER Code Start ////////////////////
#include <fstream>
#include "ARBMLlib/ARBML.h"
#include "Robot_Control.h"

// CARBML robot;                 //  Create User Robot Model Class
CRobotControl robot_control;	//	Create User Control Class

//////////////////// USER Code End ////////////////////