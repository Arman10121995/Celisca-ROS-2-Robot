# Try to find ORB_SLAM3
# Set alternative paths to search for using ORB_SLAM3_DIR or ORB_SLAM3_ROOT_DIR.
# You should ensure your ORB_SLAM3 can run correctly (i.e. built via its build.sh).
#
# Priority:
#   1. If ORB_SLAM3_ROOT_DIR is already defined (CMake -D or previous set)
#   2. If environment variable ORB_SLAM3_ROOT_DIR is set
#   3. Probe common locations (works for this workspace layout and typical user clones)
set(ORB_SLAM3_ROOT_DIR "/home/molar1/workspace/molar/ORB_SLAM3" CACHE PATH "Root directory of the ORB_SLAM3 source tree (where include/System.h lives)")

# message(STATUS "ORB_SLAM3 search root: ${ORB_SLAM3_ROOT_DIR}")

# Find ORB_SLAM3
find_path(ORB_SLAM3_INCLUDE_DIR NAMES System.h
          PATHS ${ORB_SLAM3_ROOT_DIR}/include
          NO_DEFAULT_PATH)

find_library(ORB_SLAM3_LIBRARY NAMES ORB_SLAM3 libORB_SLAM3
             PATHS ${ORB_SLAM3_ROOT_DIR}/lib
             NO_DEFAULT_PATH)

# Find built-in DBoW2 (headers live under Thirdparty/DBoW2/DBoW2/)
find_path(DBoW2_INCLUDE_DIR NAMES Thirdparty/DBoW2/DBoW2/BowVector.h
          PATHS ${ORB_SLAM3_ROOT_DIR}
          NO_DEFAULT_PATH)

find_library(DBoW2_LIBRARY NAMES DBoW2
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/DBoW2/lib
             NO_DEFAULT_PATH)

# Find built-in g2o
find_library(g2o_LIBRARY NAMES g2o g2o_core
             PATHS ${ORB_SLAM3_ROOT_DIR}/Thirdparty/g2o/lib
             NO_DEFAULT_PATH)

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set ORB_SLAM3_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(ORB_SLAM3  DEFAULT_MSG
                                  ORB_SLAM3_LIBRARY ORB_SLAM3_INCLUDE_DIR DBoW2_INCLUDE_DIR DBoW2_LIBRARY g2o_LIBRARY)

mark_as_advanced(ORB_SLAM3_INCLUDE_DIR ORB_SLAM3_LIBRARY )

set(ORB_SLAM3_LIBRARIES ${ORB_SLAM3_LIBRARY} ${DBoW2_LIBRARY} ${g2o_LIBRARY})
set(ORB_SLAM3_INCLUDE_DIRS ${ORB_SLAM3_INCLUDE_DIR} ${DBoW2_INCLUDE_DIR})