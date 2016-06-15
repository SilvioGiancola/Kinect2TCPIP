cmake_minimum_required (VERSION 2.8.12 FATAL_ERROR)

# Set Project Name
PROJECT(Kinect2TCPIP)

find_package (Qt4 REQUIRED COMPONENTS QtCore QtGui QtNetwork) # component of QT4
find_package (VTK REQUIRED) # find VTK
find_package (PCL 1.8 REQUIRED) # find PCL

 # find freenect2
find_package (freenect2 REQUIRED)


# include the current directory
SET(CMAKE_INCLUDE_CURRENT_DIR ON)





include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES main.cpp mainwindow.cpp mykinect.cpp)
set  (project_HEADERS mainwindow.h mykinect.h)
set  (project_FORMS   mainwindow.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)



QT4_WRAP_CPP(project_HEADERS_MOC ${project_HEADERS})
QT4_WRAP_UI(project_FORMS_HEADERS ${project_FORMS})
include(${QT_USE_FILE})
add_definitions(${QT_DEFINITION})
#set(HEADERS adafruit_uart.h pololu_controller.h pololu_protocol.h yei_imu.h)
#set(SOURCES main.cpp mainwindow.cpp pololu_controller_widget.cpp adafruit_widget.cpp yei_widget.cpp)

#QT4_WRAP_CPP(QT_HEADERS mainwindow.h pololu_controller_widget.h adafruit_widget.h yei_widget.h yei_imu.h pololu_controller.h)
#QT4_WRAP_UI(QT_UIS mainwindow.ui pololu_controller_widget.ui adafruit_widget.ui yei_widget.ui)
#set(HEADERS adafruit_uart.h pololu_controller.h pololu_protocol.h yei_imu.h)
#set(SOURCES main.cpp mainwindow.cpp pololu_controller_widget.cpp adafruit_widget.cpp yei_widget.cpp)

ADD_EXECUTABLE(Kinect2Client ${project_SOURCES}
                ${project_FORMS_HEADERS}
                ${project_HEADERS_MOC} )

TARGET_LINK_LIBRARIES(Kinect2Client ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY})
