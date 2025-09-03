#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "mimpclib::mimpclib" for configuration "Debug"
set_property(TARGET mimpclib::mimpclib APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(mimpclib::mimpclib PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_DEBUG "CXX"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/lib/libmimpclib.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS mimpclib::mimpclib )
list(APPEND _IMPORT_CHECK_FILES_FOR_mimpclib::mimpclib "${_IMPORT_PREFIX}/lib/libmimpclib.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
