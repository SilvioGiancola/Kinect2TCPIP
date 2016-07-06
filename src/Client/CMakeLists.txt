include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Utilities)

find_package(PCL 1.8 REQUIRED QUIET)
find_package(freenect2 REQUIRED QUIET)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# include the current directory
SET(CMAKE_INCLUDE_CURRENT_DIR ON)



# CLIENT SOLUTION (remote control)
set(client_SOURCES ClientMain.cpp ClientWidget.cpp MultiClientWindow.cpp ../Utilities/CloudViewer.cpp ../Utilities/TransformationWidget.cpp ../Utilities/Transform.cpp ../Utilities/LogWidget.cpp)
set(client_HEADERS ClientWidget.h MultiClientWindow.h ../Utilities/define.h ../Utilities/CloudViewer.h  ../Utilities/TransformationWidget.h ../Utilities/Transform.h ../Utilities/LogWidget.h)
set(client_FORMS   ClientWidget.ui MultiClientWindow.ui ../Utilities/CloudViewer.ui ../Utilities/TransformationWidget.ui ../Utilities/LogWidget.ui)



IF("${QT_VERSION}" STREQUAL "5")

    #find QT5
    find_package(Qt5Core REQUIRED)
    find_package(Qt5Widgets REQUIRED)
    find_package(Qt5SerialPort REQUIRED)
    find_package(Qt5Gui REQUIRED)
    find_package(Qt5Network REQUIRED)
    set(QT_LIB ${Qt5Widgets_LIBRARIES} ${Qt5SerialPort_LIBRARIES} ${Qt5Network_LIBRARIES})

    #use local VTK
    set(VTK_DIR "/usr/local/lib")
    find_package(VTK 6.2 REQUIRED) # find VTK
    include(${VTK_USE_FILE})

ELSEIF("${QT_VERSION}" STREQUAL "4")

    #find QT4
    find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtNetwork) # component of QT4
    include(${QT_USE_FILE})
    add_definitions(${QT_DEFINITION})
    set(QT_LIB ${QT_LIBRARIES})

    #use system VTK
    set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

ENDIF()



IF("${QT_VERSION}" STREQUAL "5")

    QT5_WRAP_CPP(client_HEADERS_MOC ${client_HEADERS})
    QT5_WRAP_UI(client_FORMS_HEADERS ${client_FORMS})

ELSEIF("${QT_VERSION}" STREQUAL "4")

    QT4_WRAP_CPP(client_HEADERS_MOC ${client_HEADERS})
    QT4_WRAP_UI(client_FORMS_HEADERS ${client_FORMS})

endif()


ADD_EXECUTABLE(Kinect2Client ${client_SOURCES} ${client_FORMS_HEADERS} ${client_HEADERS_MOC} ${client_HEADERS} ${client_SOURCES})

TARGET_LINK_LIBRARIES(Kinect2Client ${QT_LIB} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY})
