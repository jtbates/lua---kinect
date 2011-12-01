# - Find the libusb includes and library
# This module defines
#  LIBUSB_INCLUDE_DIR, path to libusb/libusb.h, etc.
#  LIBUSB_LIBRARIES, the libraries required to use LIBUSB.
#  LIBUSB_FOUND, If false, do not try to use LIBUSB.
# also defined, but not for general use are
#  LIBUSB_libusb_LIBRARY, where to find the LIBUSB library.

FIND_PATH(LIBUSB_INCLUDE_DIR libusb.h
  /usr/local/include
  /usr/local/include/libusb-1.0/
  /usr/include
)

FIND_LIBRARY(LIBUSB_libusb_LIBRARY NAMES libusb libusb-1.0.dylib
  /usr/local/lib
  /usr/lib
)

MARK_AS_ADVANCED(
  LIBUSB_INCLUDE_DIR
  LIBUSB_libusb_LIBRARY)

SET( LIBUSB_FOUND "NO" )
IF(LIBUSB_INCLUDE_DIR)
  IF(LIBUSB_libusb_LIBRARY)
    SET( LIBUSB_FOUND "YES" )
    SET( LIBUSB_LIBRARIES
      ${LIBUSB_libusb_LIBRARY})
  ENDIF(LIBUSB_libusb_LIBRARY)
ENDIF(LIBUSB_INCLUDE_DIR)

IF(LIBUSB_FOUND)
  MESSAGE(STATUS "Found libusb library")
ELSE(LIBUSB_FOUND)
  IF(LIBUSB_FIND_REQUIRED)
    MESSAGE(FATAL_ERROR "Could not find libusb
-- please give some paths to CMake or make sure libusb is installed in your system")
  ELSE(LIBUSB_FIND_REQUIRED)
    MESSAGE(STATUS "Could not find libusb
-- please give some paths to CMake or make sure libusb is installed in your system")
  ENDIF(LIBUSB_FIND_REQUIRED)
ENDIF(LIBUSB_FOUND)
