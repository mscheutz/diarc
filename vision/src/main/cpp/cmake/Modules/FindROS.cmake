# Finds ROS packages.
# If successful, sets the following variables:
#
# ROS_VERSION: string for the found ROS version (if not already set)
# ROS_INCLUDE_DIR: ROS header include path
# ROS_LIBRARIES: library files to link with in order to get ROS functionality
#
# If ROS_VERSION is not set, it will check ros versions jade, indigo, hydro, and groovy
# and configure variables for the newest version.  ROS_VERSION will also be set.
#
# If ROSDEPS is set, then it will add these dependencies to ROS_LIBRARIES
#


macro(list_append_unique listname)
    foreach(_item ${ARGN})
        list(FIND ${listname} ${_item} _index)
        if(_index EQUAL -1)
            list(APPEND ${listname} ${_item})
        endif()
    endforeach()
endmacro()

macro ( mark_as_internal _var )
    set ( ${_var} ${${_var}} CACHE INTERNAL "hide this!" FORCE )
endmacro( mark_as_internal _var )

macro(find_ros_package name)
    cmake_parse_arguments(_ARG "REQUIRED" "" "" ${ARGN})
    if(_ARG_UNPARSED_ARGUMENTS)
        message(FATAL_ERROR
                "find_ros_package() called with unused arguments: ${ARG_UNPARSED_ARGUMENTS}")
    endif()

    if(_ARG_REQUIRED)
        set(_required "REQUIRED")
    else()
        set(_required "")
    endif()

    pkg_check_modules(ros1_${name} ${_required} ${name})
    if(ros1_${name}_FOUND)
        set(_libraries "${ros1_${name}_LIBRARIES}")
        set(_library_dirs "${ros1_${name}_LIBRARY_DIRS}")
        set(ros1_${name}_LIBRARIES "")
        set(ros1_${name}_LIBRARY_DIRS "")
        foreach(_library ${_libraries})
            string(SUBSTRING "${_library}" 0 1 _prefix)
            if("${_prefix} " STREQUAL ": ")
                string(SUBSTRING "${_library}" 1 -1 _rest)
                list(APPEND ros1_${name}_LIBRARIES ${_rest})
            elseif(IS_ABSOLUTE ${_library})
                list(APPEND ros1_${name}_LIBRARIES ${_library})
            else()
                set(_lib "${_library}-NOTFOUND" CACHE INTERNAL "")
                set(_lib_path "")
                # since the path where the library is found is not returned
                # this has to be done for each path separately
                foreach(_path ${_library_dirs})
                    find_library(_lib ${_library}
                            PATHS ${_path}
                            NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
                    if(_lib)
                        set(_lib_path ${_path})
                        break()
                    endif()
                endforeach()
                if(_lib)
                    list(APPEND ros1_${name}_LIBRARIES ${_lib})
                    list_append_unique(ros1_${name}_LIBRARY_DIRS ${_lib_path})
                else()
                    # as a fall back try to search globally
                    find_library(_lib ${_library})
                    if(NOT _lib)
                        message(FATAL_ERROR "pkg-config module '${name}' failed to find library '${_library}'")
                    endif()
                    list(APPEND ros1_${name}_LIBRARIES ${_lib})
                endif()
            endif()
        endforeach()

        mark_as_internal(_lib)
    endif()
endmacro()

SET(AVAILABLE_ROS_VERSIONS "noetic;melodic;kinetic;jade;indigo;hydro;groovy")
SET(ROS_VERSION CACHE STRING "ROS version")
SET(ROS_FOUND FALSE)

IF(ROS_VERSION MATCHES "")
    MESSAGE("ROS_VERSION not defined")
    FOREACH(version ${AVAILABLE_ROS_VERSIONS})
        IF(NOT ROS_FOUND)
            FIND_PATH(ROS_H ros.h PATHS /opt/ros/${version}/include/ros)
            IF(ROS_H)
                MESSAGE(STATUS "Found ros version ${version}")
                SET(ROS_VERSION2 ${version})
                SET(ROS_FOUND 1)
            ENDIF(ROS_H)
        ENDIF(NOT ROS_FOUND)
    ENDFOREACH(version)
ELSE(ROS_VERSION MATCHES "")
    MESSAGE("ROS_VERSION defined as " ${ROS_VERSION})
    FIND_PATH(ROS_H ros.h PATHS /opt/ros/${ROS_VERSION}/include/ros)
    IF(ROS_H)
        MESSAGE("Found ros version ${ROS_VERSION}")
        SET(ROS_FOUND 1)
        SET(ROS_VERSION2 ${version})
    ENDIF(ROS_H)
ENDIF(ROS_VERSION MATCHES "")

IF(NOT ROS_FOUND)
    MESSAGE(WARNING "ROS files not found")
ELSE(NOT ROS_FOUND)
    SET(ROS_PATH /opt/ros/${ROS_VERSION2})
    SET(ROS_INCLUDE_DIR ${ROS_PATH}/include)
    MESSAGE(STATUS "Detected ROS version ${ROS_VERSION2}")
    FOREACH(pkg ${ROS_FIND_COMPONENTS})
        FIND_ROS_PACKAGE(${pkg})
        IF(NOT ros1_${pkg}_FOUND)
            MESSAGE("Could not find ROS package: ${pkg}.")

            # set to false so ROS_FOUND gets set to false in find_package_handle_standard_args
            SET(ROS_LIBRARY_DIRS FALSE)
            SET(ROS_LIBRARIES FALSE)
            BREAK()
        ENDIF(NOT ros1_${pkg}_FOUND)
        LIST_APPEND_UNIQUE(ROS_LIBRARY_DIRS ${ros1_${pkg}_LIBRARY_DIRS})
        LIST_APPEND_UNIQUE(ROS_LIBRARIES ${ros1_${pkg}_LIBRARIES})
    ENDFOREACH(pkg)
    MESSAGE(STATUS "ROS dependencies ${ROS_FIND_COMPONENTS} need the following libraries:")
    FOREACH(NAME ${ROS_LIBRARIES})
        MESSAGE(STATUS "  " ${NAME})
    ENDFOREACH(NAME)
ENDIF(NOT ROS_FOUND)

INCLUDE(FindPackageHandleStandardArgs)
# check that all listed variables are TRUE
FIND_PACKAGE_HANDLE_STANDARD_ARGS(ROS DEFAULT_MSG ROS_INCLUDE_DIR ROS_LIBRARY_DIRS)

MARK_AS_ADVANCED(ROS_INCLUDE_DIR ROS_LIBRARY_DIRS)
