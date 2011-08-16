         cmake_minimum_required(VERSION 2.8)

         set (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

         # infer path for Torch7
         string (REGEX REPLACE "(.*)lib/luarocks/rocks.*" "\\1" TORCH_PREFIX "${CMAKE_INSTALL_PREFIX}" )
         message (STATUS "Found Torch7, installed in: " ${TORCH_PREFIX})

         find_package (Torch REQUIRED)
         find_package (Freenect REQUIRED)
         find_package (Libusb REQUIRED)

         include_directories (${LIBUSB_INCLUDE_DIR} ${FREENECT_INCLUDE_DIR} ${TORCH_INCLUDE_DIR} ${PROJECT_SOURCE_DIR})
         include_directories ("cross_bf")
         include_directories ("cross_bf/include")
         add_library (kinect SHARED kinect.cpp preprocess.cpp cbf.cpp)
         target_link_libraries (kinect ${TORCH_LIBRARIES} ${FREENECT_LIBRARIES} ${LIBUSB_LIBRARIES})

         MESSAGE(${LIBUSB_LIBRARIES})

         install_targets (/lib kinect)
         install_files(/lua kinect.lua)
   