LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

#Opencv
find_package( OpenCV HINTS /usr/local/Cellar/opencv/4.1.2 )
#------------------------------------------------

ADD_LIBRARY( videoStable videoStable.cpp )
TARGET_LINK_LIBRARIES( videoStable
    ${OpenCV_LIBS} 
)

#------------------------------------------------

ADD_EXECUTABLE( bin main.cpp )
TARGET_LINK_LIBRARIES( bin 
    videoStable
    ${OpenCV_LIBS}  
)
