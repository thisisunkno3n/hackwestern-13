# Cross-compile for the Pi 5 on QNX 8.0 from a laptop with QNX SDP 8.0 installed.
#   source ~/qnx800/qnxsdp-env.sh
#   cmake -S . -B build-qnx -DSEEKER_TARGET=qnx -DCMAKE_TOOLCHAIN_FILE=cmake/qnx-aarch64le.cmake
# Not run tonight (no SDP on the build host). CMake's QNX platform turns
# CMAKE_C_COMPILER_TARGET into qcc's -V option; check `qcc -V` lists gcc_ntoaarch64le.
set(CMAKE_SYSTEM_NAME QNX)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_C_COMPILER qcc)
set(CMAKE_C_COMPILER_TARGET gcc_ntoaarch64le)
if(DEFINED ENV{QNX_TARGET})
  set(CMAKE_SYSROOT "$ENV{QNX_TARGET}")
  set(CMAKE_FIND_ROOT_PATH "$ENV{QNX_TARGET};$ENV{QNX_TARGET}/aarch64le")
endif()
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
