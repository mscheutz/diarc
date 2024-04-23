###############################################################################
# Find SIFTGPU
#
# This sets the following variables:
# SIFTGPU_INCLUDE_DIRS - Directories to include to use SIFT
# SIFTGPU_LIBRARIES    - Default library to link against to use SIFT
# SIFTGPU_LINK_FLAGS   - Flags to be added to linker's options
# SIFTGPU_FOUND        - If false, don't try to use SIFT

find_path(SIFTGPU_INCLUDE_DIR SiftGPU.h
    PATH /usr/local/src/SiftGPU/src/SiftGPU )

find_library(SIFTGPU_LIBRARY NAMES siftgpu libsiftgpu
    PATH /usr/local/src/SiftGPU/linux/bin )

set(SIFTGPU_LIBRARIES ${SIFTGPU_LIBRARY} )
set(SIFTGPU_INCLUDE_DIRS ${SIFTGPU_INCLUDE_DIR} )

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set SIFTGPU_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(SIFTGPU  DEFAULT_MSG
                                  SIFTGPU_LIBRARY SIFTGPU_INCLUDE_DIR)

mark_as_advanced(SIFTGPU_INCLUDE_DIR SIFTGPU_LIBRARY )
