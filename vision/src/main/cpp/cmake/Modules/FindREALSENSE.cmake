###########################################
#Find REALSENSE library
#
#This helps find:
#> The header files for realsense2 library
#> The shared library realsense2.so
#
SET(HEADERS_GRASP
        rs.hpp
        rs_advanced_mode.hpp
        #rs_advanced_mode_command.h
        #rs_context.h
        #rs_device.h
        #rs_frame.h
        #rs_internal.h
        #rs_option.h
        #rs_pipeline.h
        #rs_processing.h
        #rs_record_playback.h
        #rs_sensor.h
        #rs_types.h
        rsutil.h
        )

FIND_PATH(REALSENSE_INCLUDE_DIR ${HEADERS_GRASP}
        HINTS /usr/include/librealsense2 /usr/local/include/librealsense2)

FIND_LIBRARY(REALSENSE_LIBRARY librealsense2.so
        PATHS /usr/lib /usr/local/lib)

SET(REALSENSE_LIBRARY_DIRS ${REALSENSE_LIBRARY} )
SET(REALSENSE_INCLUDE_DIRS ${REALSENSE_INCLUDE_DIR} )

message("REALSENSE_INCLUDE_DIR: ${REALSENSE}")

INCLUDE(FindPackageHandleStandardArgs)
# check that all listed variables are TRUE
find_package_handle_standard_args(REALSENSE DEFAULT_MSG
        REALSENSE_LIBRARY REALSENSE_INCLUDE_DIR)

mark_as_advanced(REALSENSE_INCLUDE_DIR REALSENSE_LIBRARY)
