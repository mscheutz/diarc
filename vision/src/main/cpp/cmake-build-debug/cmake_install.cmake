# Install script for directory: /home/evan/code/ade/vision/src/main/cpp

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/capture/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/common/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/detector/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/display/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/imgproc/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/learn/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/point_clouds/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/shape_match/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/stm/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/third_party/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/tracker/cmake_install.cmake")
  include("/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/visionproc/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/evan/code/ade/vision/src/main/cpp/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
