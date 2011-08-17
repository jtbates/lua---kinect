
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
         find_package (Libusb REQUIRED)
         find_package (Cuda REQUIRED)
				 
				 message (STATUS "Found Cuda installed in " ${CUDA_INCLUDE_DIR})

         # Compile CUDA code.
				 set (CUDA_TARGET_SM "sm_13")
				 set (src-cuda depth_to_point_cloud_par.cu)
				 cuda_compile (gen-cuda ${src-cuda})
				 

         include_directories (${LIBUSB_INCLUDE_DIR})
         include_directories (${FREENECT_INCLUDE_DIR})
         include_directories (${TORCH_INCLUDE_DIR})

         include_directories (${CUDA_INCLUDE_DIR})

         include_directories (${PROJECT_SOURCE_DIR})
         include_directories ("cross_bf")
         include_directories ("cross_bf/include")
				 
         cuda_add_library (kinect SHARED kinect.cpp preprocess.cpp cbf.cpp depth_to_point_cloud_par.cu)
         target_link_libraries (kinect ${TORCH_LIBRARIES}
				                               ${FREENECT_LIBRARIES}
																			 ${LIBUSB_LIBRARIES})

         install_targets (/lib kinect)
         install_files(/lua kinect.lua)
   ]],

   variables = {
      CMAKE_INSTALL_PREFIX = "$(PREFIX)"
   }
}
