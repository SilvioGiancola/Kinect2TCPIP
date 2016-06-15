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

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITION})

set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)


set  (server_SOURCES main.cpp mainwindow.cpp mykinect.cpp)
set  (server_HEADERS mainwindow.h mykinect.h)
set  (server_FORMS   mainwindow.ui)


QT4_WRAP_CPP(server_HEADERS_MOC ${server_HEADERS})
QT4_WRAP_UI(server_FORMS_HEADERS ${server_FORMS})



#set(HEADERS adafruit_uart.h pololu_controller.h pololu_protocol.h yei_imu.h)
#set(SOURCES main.cpp mainwindow.cpp pololu_controller_widget.cpp adafruit_widget.cpp yei_widget.cpp)

#QT4_WRAP_CPP(QT_HEADERS mainwindow.h pololu_controller_widget.h adafruit_widget.h yei_widget.h yei_imu.h pololu_controller.h)
#QT4_WRAP_UI(QT_UIS mainwindow.ui pololu_controller_widget.ui adafruit_widget.ui yei_widget.ui)
#set(HEADERS adafruit_uart.h pololu_controller.h pololu_protocol.h yei_imu.h)
#set(SOURCES main.cpp mainwindow.cpp pololu_controller_widget.cpp adafruit_widget.cpp yei_widget.cpp)

ADD_EXECUTABLE(Kinect2Server ${server_SOURCES} ${server_FORMS_HEADERS} ${server_HEADERS_MOC} )

TARGET_LINK_LIBRARIES(Kinect2Server ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY})




set  (client_SOURCES ClientMain.cpp ClientWindow.cpp)
set  (client_HEADERS ClientWindow.h)
set  (client_FORMS   ClientWindow.ui)


QT4_WRAP_CPP(client_HEADERS_MOC ${client_HEADERS})
QT4_WRAP_UI(client_FORMS_HEADERS ${client_FORMS})

ADD_EXECUTABLE(Kinect2Client ${client_SOURCES} ${client_FORMS_HEADERS} ${client_HEADERS_MOC} )

TARGET_LINK_LIBRARIES(Kinect2Client ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY})
