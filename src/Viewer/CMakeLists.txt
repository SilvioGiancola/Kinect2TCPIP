# VIEWER SOLUTION (Kinect V2 Handler)


# ALL WIDGETS
FILE(GLOB viewer_SOURCES ViewerMain.cpp ViewerWindow.cpp  ../Utilities/*.cpp)
FILE(GLOB viewer_HEADERS ViewerWindow.h  ../Utilities/*.h)
FILE(GLOB viewer_FORMS ViewerWindow.ui  ../Utilities/*.ui)
#FILE(GLOB Widgets_FORMS Widgets/*.ui)
#QT5_WRAP_UI(Widgets_FORMS_HEADERS ${Widgets_FORMS})


#set(viewer_SOURCES ViewerMain.cpp ViewerWindow.cpp  ../Utilities/CloudViewer.cpp ../Utilities/CloudList.cpp)
#set(viewer_HEADERS ViewerWindow.h   ../Utilities/CloudViewer.h ../Utilities/CloudList.h ../Utilities/define.h)
#set(viewer_FORMS   ViewerWindow.ui ../Utilities/CloudViewer.ui ../Utilities/CloudList.ui)





find_package(OpenCV 3.1.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
#set(OpenCV_LIBS opencv_core opencv_imgproc opencv_calib3d opencv_video opencv_features2d opencv_ml opencv_highgui opencv_objdetect opencv_contrib opencv_legacy opencv_gpu)
IF(OpenCV_FOUND)
   MESSAGE(STATUS "Found OpenCV: ${OpenCV_INCLUDE_DIRS} VERSION : ${OpenCV_VERSION}")
    MESSAGE(STATUS ${OpenCV_LIBS})
ENDIF(OpenCV_FOUND)
link_directories(${OpenCV_INCLUDE_DIRS})
add_definitions(${OpenCV_DEFINITIONS})



IF("${QT_VERSION}" STREQUAL "4")

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




    QT4_WRAP_CPP(viewer_HEADERS_MOC ${viewer_HEADERS})
    QT4_WRAP_UI(viewer_FORMS_HEADERS ${viewer_FORMS})


    ADD_EXECUTABLE(Viewer ${viewer_SOURCES} ${viewer_FORMS_HEADERS} ${viewer_HEADERS_MOC} ${viewer_HEADERS} ${viewer_SOURCES} ../../CompileScript.sh)

    TARGET_LINK_LIBRARIES(Viewer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY}  ${OpenCV_LIBS})

else()


    include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../Utilities)

    find_package(PCL 1.8 REQUIRED QUIET)
    find_package(freenect2 REQUIRED QUIET)

    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})

    # include the current directory
    SET(CMAKE_INCLUDE_CURRENT_DIR ON)




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

        QT5_WRAP_CPP(viewer_HEADERS_MOC ${viewer_HEADERS})
        QT5_WRAP_UI(viewer_FORMS_HEADERS ${viewer_FORMS})

    ELSEIF("${QT_VERSION}" STREQUAL "4")

        #find QT4
        find_package(Qt4 REQUIRED COMPONENTS QtCore QtGui QtNetwork QtWidgets) # component of QT4
        include(${QT_USE_FILE})
        add_definitions(${QT_DEFINITION})
       set(QT_LIB ${QT_LIBRARIES})

        #use system VTK
        set(VTK_LIBRARIES vtkRendering vtkGraphics vtkHybrid QVTK)

        QT4_WRAP_CPP(viewer_HEADERS_MOC ${viewer_HEADERS})
        QT4_WRAP_UI(viewer_FORMS_HEADERS ${viewer_FORMS})

    ENDIF()



    ADD_EXECUTABLE(Viewer ${viewer_SOURCES} ${viewer_FORMS_HEADERS} ${viewer_HEADERS_MOC} ${viewer_HEADERS} ${viewer_SOURCES} ../../CompileScript.sh)


    TARGET_LINK_LIBRARIES(Viewer ${QT_LIB} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${freenect2_LIBRARY}  ${OpenCV_LIBS})

endif()

