# Locate the openMVG libraries.
#
# Defines the following variables:
#
#   CCTAG_FOUND        - TRUE if the CCTag headers and libs are found
#   CCTAG_INCLUDE_DIRS - The path to CCTag headers
#
#   CCTAG_LIBRARIES    - Libraries to link against to use CCTag.
#   CCTAG_LIBRARY_DIR  - The base directory to search for CCTag.
#
# Accepts the following variables as input:
#
#   CCTAG_DIR - (as a CMake or environment variable)
#                The root directory of the CCTag install prefix

MESSAGE(STATUS "Looking for CCTag.")

# Try to find the headers location
FIND_PATH(CCTAG_INCLUDE_DIR cctag/ICCTag.hpp
  HINTS
  $ENV{CCTAG_DIR}/include
  ${CCTAG_DIR}/include
  PATH_SUFFIXES
  CCTag
)
IF(CCTAG_INCLUDE_DIR)
  MESSAGE(STATUS "CCTag headers found in ${CCTAG_INCLUDE_DIR}")
ELSE()
  MESSAGE(WARNING "CCTag headers not found")
ENDIF (CCTAG_INCLUDE_DIR)

# Locate libCCTag.a
FIND_LIBRARY(CCTAG_LIBRARY NAMES CCTag 
  HINTS
  $ENV{CCTAG_DIR}/lib
  ${CCTAG_DIR}/lib
  PATH_SUFFIXES
  CCTag
)
IF(CCTAG_LIBRARY)
  MESSAGE(STATUS "CCTag library found: ${CCTAG_LIBRARY}")
ENDIF (CCTAG_LIBRARY)


#
# Try to find the packages needed to work with CCTags
# note that openMVG is already providing openCV, Ceres and Glog (or minilog)
# TODO : add boost ?
# TODO : the CCTag library could export a CCTagConfig.cmake to provide the path
#        of its dependencies
find_package(OPTPP REQUIRED)
find_package(OpenCV REQUIRED) # TODO: try to use openCV provided by openMVG
if (NOT Ceres_FOUND) # Shouldn't this test be in FindCeres.cmake ?
	             # Or may be we only need to check that ${Ceres_LIBRARIES} is set
  find_package(Ceres QUIET HINTS ${CERES_DIR_HINTS})
endif()

# Not sure that we need glog, commenting it for now
#find_package(Glog QUIET)

# Look for the cuda version of the lib
FIND_LIBRARY(CCTAGCUDA_LIBRARY NAMES CCTagCuda 
  HINTS
  $ENV{CCTAG_DIR}/lib
  ${CCTAG_DIR}/lib
  PATH_SUFFIXES
  CCTag
)
# To work with cuda, cctags needs additional CUDA cudadevrt library
IF(CCTAGCUDA_LIBRARY)
  SET(CCTAGCUDA_LIBRARIES ${CCTAGCUDA_LIBRARY} cudadevrt)
ENDIF()

# Sets the libraries needed to link with CCTag
SET(CCTAG_LIBRARIES 
	${CCTAG_LIBRARY} 
	boost_filesystem boost_system boost_serialization 
	dl 
	${OpenCV_LIBS} 
	${OPTPP_LIBRARIES} 
	${Ceres_LIBRARIES} 
	lapack
	#${GLOG_LIBRARIES}
	${CCTAGCUDA_LIBRARIES}
)

# Sets the include dirs we need to compile with cctags
SET(CCTAG_INCLUDE_DIRS ${CCTAG_INCLUDE_DIR} ${OpenCV_INCLUDE_DIRS})


include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set CCTAG_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(CCTag  DEFAULT_MSG
                                  CCTAG_LIBRARY CCTAG_INCLUDE_DIR)

MARK_AS_ADVANCED(CCTAG_INCLUDE_DIR CCTAG_LIBRARY)
