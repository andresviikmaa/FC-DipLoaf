cmake_minimum_required(VERSION 2.8.4)
project(robotiina)

    set(Boost_USE_STATIC_LIBS OFF)
    set(WINSOCK_LIB "")
    set(WINSOCK2_LIB "")

 set(Boost_DEBUG 1)

#boost
SET(Boost_ADDITIONAL_VERSIONS "1.56" "1.56.0" "1.55" "1.55.0")

#list( APPEND CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -ftest-coverage -fprofile-arcs")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
find_package( OpenCV REQUIRED )

set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost 1.58.0 COMPONENTS program_options system thread chrono date_time filesystem atomic)

#if(Boost_FOUND)
include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

#endif()
OPTION(SHOW_UI "Show UI" OFF)

IF(SHOW_UI)
    ADD_DEFINITIONS(-DSHOW_UI)
ENDIF(SHOW_UI)

ADD_LIBRARY( CommonModule STATIC
   CommonModule/ConfigurableModule.cpp
   CommonModule/FieldState.cpp
   CommonModule/ThreadedClass.cpp
   CommonModule/UdpServer.cpp
   CommonModule/WheelConfig.cpp
   CommonModule/Settings.cpp
)
ADD_LIBRARY( DisplayModule STATIC
   DisplayModule/Dialog.cpp
)
ADD_LIBRARY( HardwareModule STATIC
   HardwareModule/Camera.cpp
   HardwareModule/CoilBoard.cpp
   HardwareModule/ComModule.cpp
   HardwareModule/refereeCom.cpp

)
ADD_LIBRARY( RobotModule STATIC
   RobotModule/Robot.cpp
   RobotModule/ManualControl.cpp
)
ADD_LIBRARY( VisionModule STATIC
   VisionModule/BallFinder.cpp
   VisionModule/DummyVision.cpp
   VisionModule/FrontCameraVision.cpp
   VisionModule/GateFinder.cpp
   VisionModule/ImageThresholder.cpp
   VisionModule/MainCameraVision.cpp
   VisionModule/RobotFinder.cpp
   VisionModule/SimpleImageThresholder.cpp
)
ADD_LIBRARY( StateMachine STATIC
   StateMachine/DriveInstruction.cpp
   StateMachine/MultiModePlay.cpp
   StateMachine/SingleModePlay.cpp
   StateMachine/StateMachine.cpp
)
TARGET_LINK_LIBRARIES( CommonModule  ${OpenCV_LIBS} ${Boost_LIBRARIES} atomic )
TARGET_LINK_LIBRARIES( DisplayModule CommonModule ${OpenCV_LIBS} ${Boost_LIBRARIES} atomic )
TARGET_LINK_LIBRARIES( HardwareModule CommonModule ${OpenCV_LIBS} ${Boost_LIBRARIES} atomic )
TARGET_LINK_LIBRARIES( RobotModule CommonModule ${OpenCV_LIBS} ${Boost_LIBRARIES} atomic )
TARGET_LINK_LIBRARIES( VisionModule CommonModule ${OpenCV_LIBS} ${Boost_LIBRARIES} atomic )
TARGET_LINK_LIBRARIES( StateMachine CommonModule ${OpenCV_LIBS} ${Boost_LIBRARIES} atomic )

set(ROBOTIINA_FILES  Robotiina/Robotiina.cpp)

add_executable(robotiina ${ROBOTIINA_FILES})
target_link_libraries( robotiina RobotModule CommonModule HardwareModule VisionModule StateMachine ${OpenCV_LIBS} ${Boost_LIBRARIES} -lX11 )

set(COLORCALIB_FILES  ColorCalibrator/ColorCalibrator.cpp ColorCalibrator/AutoCalibrator.cpp)

add_executable(colorcalibrator ${COLORCALIB_FILES})
target_link_libraries( colorcalibrator CommonModule HardwareModule  DisplayModule ${OpenCV_LIBS} ${Boost_LIBRARIES} -lX11 )


#set(WHEELEMULATOR_FILES WheelEmulator.cpp)
#add_executable(wheelemulator ${WHEELEMULATOR_FILES})
#target_link_libraries( wheelemulator ${OpenCV_LIBS} ${Boost_LIBRARIES} ${WINSOCK_LIB})
