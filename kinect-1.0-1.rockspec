
package = "kinect"
version = "1.0-1"

source = {
   url = "kinect-1.0-1.tgz"
}

description = {
   summary = "A wrapper for the Kinect",
   detailed = [[
   ]],
   homepage = "",
   license = "MIT/X11" -- or whatever you like
}

dependencies = {
   "lua >= 5.1",
   "xlua",
   "sys",
   "torch",
   "image",
}

build = {
   type = "cmake",

   cmake = [[
         cmake_minimum_required(VERSION 2.8)

         set (CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

         # infer path for Torch7
         string (REGEX REPLACE "(.*)lib/luarocks/rocks.*" "\\1" TORCH_PREFIX "${CMAKE_INSTALL_PREFIX}" )
         message (STATUS "Found Torch7, installed in: " ${TORCH_PREFIX})

         find_package (Torch REQUIRED)
         find_package (Freenect REQUIRED)

         include_directories (${FREENECT_INCLUDE_DIR} ${TORCH_INCLUDE_DIR} ${PROJECT_SOURCE_DIR})
         add_library (kinect SHARED kinect.cpp)
         target_link_libraries (kinect ${TORCH_LIBRARIES} ${FREENECT_LIBRARIES})
         install_targets (/lib kinect)
         install_files(/lua kinect.lua)
   ]],

   variables = {
      CMAKE_INSTALL_PREFIX = "$(PREFIX)"
   }
}
