include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Utilities)

#find_package (Qt4 REQUIRED COMPONENTS QtCore QtGui QtNetwork) # component of QT4
find_package(Qt5Widgets REQUIRED)
find_package(Qt5SerialPort REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5Network REQUIRED)

set(VTK_DIR "/usr/local/lib")
find_package (VTK 6.2 REQUIRED) # find VTK
include(${VTK_USE_FILE})

find_package (PCL 1.8 REQUIRED) # find PCL

 # find freenect2
find_package (freenect2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# include the current directory
SET(CMAKE_INCLUDE_CURRENT_DIR ON)


#include(${QT_USE_FILE})
#add_definitions(${QT_DEFINITION})

#set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)





# CLIENT SOLUTION (remote control)
set  (client_SOURCES ClientMain.cpp ClientWidget.cpp MultiClientWindow.cpp ../Utilities/CloudViewer.cpp)
set  (client_HEADERS ClientWidget.h MultiClientWindow.h ../Utilities/define.h ../Utilities/CloudViewer.h )
set  (client_FORMS   ClientWidget.ui MultiClientWindow.ui ../Utilities/CloudViewer.ui)


QT5_WRAP_CPP(client_HEADERS_MOC ${client_HEADERS})
QT5_WRAP_UI(client_FORMS_HEADERS ${client_FORMS})

ADD_EXECUTABLE(Kinect2Client ${client_SOURCES} ${client_FORMS_HEADERS} ${client_HEADERS_MOC} ${client_HEADERS} ${client_SOURCES})

TARGET_LINK_LIBRARIES(Kinect2Client ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${Qt5Network_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY})
