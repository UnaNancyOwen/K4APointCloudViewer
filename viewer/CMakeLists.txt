cmake_minimum_required( VERSION 3.6 )

# Language
enable_language( CXX )

# Compiler Settings
set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )
set( CMAKE_CXX_EXTENSIONS OFF )

# Project
project( viewer LANGUAGES CXX )
add_executable( viewer kinect.hpp kinect.cpp main.cpp )

# (Option) Start-Up Project for Visual Studio
set_property( DIRECTORY PROPERTY VS_STARTUP_PROJECT "viewer" )

# Find Package
find_package( k4a REQUIRED )
find_package( PCL REQUIRED )

# Set Package to Project
if( k4a_FOUND AND PCL_FOUND )
  target_link_libraries( viewer k4a::k4a )
  target_link_libraries( viewer ${PCL_LIBRARIES} )
endif()
