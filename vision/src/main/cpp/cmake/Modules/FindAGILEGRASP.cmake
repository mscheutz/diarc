###########################################
#Find AGILEGRASP library
#
#This helps find:
#> The header files for agile_grasp library
#> The shared library libagile_grasp.so
#
SET(HEADERS_GRASP
    localization.h
    antipodal.h  
    finger_hand.h  
    grasp_hypothesis.h
    grasp_localizer.h
    handle.h  
    handle_search.h  
    hand_search.h  
    learning.h    
    plot.h  
    quadric.h  
    rotating_hand.h
    )

FIND_PATH(AGILEGRASP_INCLUDE_DIR ${HEADERS_GRASP}
    PATH /usr/local/include/agile_grasp )

FIND_LIBRARY(AGILEGRASP_LIBRARY libagile_grasp.so
    PATH /usr/local/lib )

SET(AGILEGRASP_LIBRARIES ${AGILEGRASP_LIBRARY} )
SET(AGILEGRASP_INCLUDE_DIRS ${AGILEGRASP_INCLUDE_DIR} )

message("AGILEGRASP_INCLUDE_DIR: ${AGILEGRASP_INCLUDE_DIR}")

INCLUDE(FindPackageHandleStandardArgs)
# check that all listed variables are TRUE
find_package_handle_standard_args(AGILEGRASP DEFAULT_MSG
                                  AGILEGRASP_LIBRARY AGILEGRASP_INCLUDE_DIR)

mark_as_advanced(AGILEGRASP_INCLUDE_DIR AGILEGRASP_LIBRARY)
