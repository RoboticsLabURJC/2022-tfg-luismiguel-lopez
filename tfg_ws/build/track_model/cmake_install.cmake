# Install script for directory: /home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/src/track_model

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/build/track_model/catkin_generated/safe_execute_install.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/build/track_model/catkin_generated/installspace/track_model.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/track_model/cmake" TYPE FILE FILES
    "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/build/track_model/catkin_generated/installspace/track_modelConfig.cmake"
    "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/build/track_model/catkin_generated/installspace/track_modelConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/track_model" TYPE FILE FILES "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/src/track_model/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/track_model" TYPE PROGRAM FILES "/home/chuismi/Desktop/robotica/cuarto/tfg/2022-tfg-luismiguel-lopez/tfg_ws/build/track_model/catkin_generated/installspace/teleop.py")
endif()

