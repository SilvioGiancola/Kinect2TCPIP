cmake_minimum_required (VERSION 2.8.12 FATAL_ERROR)

# Set Project Name
PROJECT(Kinect2TCPIP)



## Find includes in corresponding build directories
#set(CMAKE_INCLUDE_CURRENT_DIR ON)
## Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)
## Executables fail to build with Qt 5 in the default configuration
## without -fPIE. We add that here.
#set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")

#SET(CMAKE_CXX_FLAGS "-std=c++11")



IF("${CLIENT}" STREQUAL "YES")
    ADD_SUBDIRECTORY( Client )
ENDIF()


IF("${SERVER}" STREQUAL "YES")
    ADD_SUBDIRECTORY( Server )
ENDIF()


IF("${ELAB}" STREQUAL "YES")
    ADD_SUBDIRECTORY( Elab )
ENDIF()


IF("${VIEWER}" STREQUAL "YES")
    ADD_SUBDIRECTORY( Viewer )
ENDIF()

