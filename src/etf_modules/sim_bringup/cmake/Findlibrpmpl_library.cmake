file(GLOB_RECURSE librpmpl_library_HEADERS /home/nermin/RPMPLv2/include/*.h)
find_path(librpmpl_library_INCLUDE_DIRS
  NAMES ${librpmpl_library_INCLUDE_DIRS}
  PATHS /home/nermin/RPMPLv2/include
)

find_library(librpmpl_library 
  NAMES librpmpl_library
  PATHS /home/nermin/RPMPLv2/build/src
)

add_library(librpmpl_library SHARED IMPORTED)
set_target_properties(librpmpl_library PROPERTIES
  IMPORTED_LOCATION ${librpmpl_library}
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(librpmpl_library
  DEFAULT_MSG
  librpmpl_library
  librpmpl_library_INCLUDE_DIRS
)

mark_as_advanced(librpmpl_library_INCLUDE_DIRS librpmpl_library)

if(librpmpl_library_FOUND)
  message(STATUS "found librpmpl_library")
endif(librpmpl_library_FOUND)


set(librpmpl_library_INCLUDE_DIRS ${librpmpl_library_INCLUDE_DIRS} PARENT_SCOPE)
set(librpmpl_library ${librpmpl_library} PARENT_SCOPE)