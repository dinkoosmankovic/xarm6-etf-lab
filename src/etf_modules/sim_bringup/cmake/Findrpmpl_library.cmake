find_library(RPMPL_LIBRARY
  NAMES rpmpl_library
  PATHS /home/nermin/RPMPLv2/build/src
)

set(RPMPL_PATH /home/nermin/RPMPLv2)
set(RPMPL_LIBRARY_INCLUDE_DIRS 
  ${RPMPL_PATH}/include
  ${RPMPL_PATH}/include/state_spaces
  ${RPMPL_PATH}/include/state_spaces/real_vector_space
  ${RPMPL_PATH}/include/planners
  ${RPMPL_PATH}/include/planners/rrt
  ${RPMPL_PATH}/include/planners/rbt
  ${RPMPL_PATH}/external/nanoflann/include
  ${RPMPL_PATH}/include/robots
  ${RPMPL_PATH}/include/environments
  ${RPMPL_PATH}/include/scenario
  ${RPMPL_PATH}/include/configurations
  ${RPMPL_PATH}/include/benchmark
  ${RPMPL_PATH}/external/QuadProgpp/src
)

# file(GLOB_RECURSE RPMPL_LIBRARY_INCLUDE_DIRS /home/nermin/RPMPLv2/include/*.h)

# file(GLOB_RECURSE RPMPL_LIBRARY_HEADERS /home/nermin/RPMPLv2/include/*.h)
# find_path(RPMPL_LIBRARY_INCLUDE_DIRS ${RPMPL_LIBRARY_HEADERS})
# get_filename_component(RPMPL_LIBRARY_INCLUDE_DIRS "${RPMPL_LIBRARY_INCLUDE_DIRS}" REALPATH)
# include_directories(${RPMPL_LIBRARY_INCLUDE_DIRS})

# file(GLOB_RECURSE RPMPL_LIBRARY_HEADERS /home/nermin/RPMPLv2/include/*.h)
# find_path(RPMPL_LIBRARY_INCLUDE_DIRS
#   NAMES ${RPMPL_LIBRARY_INCLUDE_DIRS}
#   PATHS /home/nermin/RPMPLv2/include
# )

include_directories(${RPMPL_LIBRARY_INCLUDE_DIRS})
# message("Included directories are: ")
# message(${RPMPL_LIBRARY_INCLUDE_DIRS})

# include(FindPackageHandleStandardArgs)
# find_package_handle_standard_args(${RPMPL_LIBRARY}
#   DEFAULT_MSG
#   ${RPMPL_LIBRARY}
#   ${RPMPL_LIBRARY_INCLUDE_DIRS}
# )

# mark_as_advanced(${RPMPL_LIBRARY_INCLUDE_DIRS} ${RPMPL_LIBRARY})

if(RPMPL_LIBRARY)
    message(STATUS "Found rpmpl_library at ${RPMPL_LIBRARY}")
else(RPMPL_LIBRARY)
    message(STATUS "Not found rpmpl_library")
endif(RPMPL_LIBRARY)