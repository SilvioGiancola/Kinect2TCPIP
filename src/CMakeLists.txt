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




# SERVER SOLUTION (Kinect V2 Handler)
set  (server_SOURCES ServerMain.cpp ServerWindow.cpp )
set  (server_HEADERS ServerWindow.h define.h )
set  (server_FORMS   ServerWindow.ui)


QT4_WRAP_CPP(server_HEADERS_MOC ${server_HEADERS})
QT4_WRAP_UI(server_FORMS_HEADERS ${server_FORMS})


ADD_EXECUTABLE(Kinect2Server ${server_SOURCES} ${server_FORMS_HEADERS} ${server_HEADERS_MOC} ${server_HEADERS} ${server_SOURCES} ../CompileScript.sh)

TARGET_LINK_LIBRARIES(Kinect2Server ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY})




# CLIENT SOLUTION (remote control)
set  (client_SOURCES ClientMain.cpp ClientWidget.cpp MultiClientWindow.cpp)
set  (client_HEADERS ClientWidget.h MultiClientWindow.h define.h )
set  (client_FORMS   ClientWidget.ui MultiClientWindow.ui)


QT4_WRAP_CPP(client_HEADERS_MOC ${client_HEADERS})
QT4_WRAP_UI(client_FORMS_HEADERS ${client_FORMS})

ADD_EXECUTABLE(Kinect2Client ${client_SOURCES} ${client_FORMS_HEADERS} ${client_HEADERS_MOC} ${client_HEADERS} ${client_SOURCES})

TARGET_LINK_LIBRARIES(Kinect2Client ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY})
