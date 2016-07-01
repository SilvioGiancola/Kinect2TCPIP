include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Utilities)

find_package (Qt4 REQUIRED COMPONENTS QtCore QtGui QtNetwork) # component of QT4
find_package (VTK REQUIRED QUIET) # find VTK
find_package (PCL 1.8 REQUIRED QUIET) # find PCL

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
set  (server_SOURCES ServerMain.cpp ServerWindow.cpp KinectWidget.cpp ../Utilities/TransformationWidget.cpp ../Utilities/CloudViewer.cpp ../Utilities/Transform.cpp)
set  (server_HEADERS ServerWindow.h KinectWidget.h ../Utilities/TransformationWidget.h ../Utilities/CloudViewer.h ../Utilities/define.h ../Utilities/Transform.h  )
set  (server_FORMS   ServerWindow.ui KinectWidget.ui ../Utilities/TransformationWidget.ui ../Utilities/CloudViewer.ui)


QT4_WRAP_CPP(server_HEADERS_MOC ${server_HEADERS})
QT4_WRAP_UI(server_FORMS_HEADERS ${server_FORMS})


ADD_EXECUTABLE(Kinect2Server ${server_SOURCES} ${server_FORMS_HEADERS} ${server_HEADERS_MOC} ${server_HEADERS} ${server_SOURCES} ../../CompileScript.sh)

TARGET_LINK_LIBRARIES(Kinect2Server ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY})



