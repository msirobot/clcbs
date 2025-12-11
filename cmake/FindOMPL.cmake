# Find OMPL using pkg-config
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(OMPL QUIET ompl)
endif()

if(OMPL_FOUND)
    set(OMPL_INCLUDE_DIRS ${OMPL_INCLUDE_DIRS})
    set(OMPL_LIBRARIES ${OMPL_LIBRARIES})
    set(OMPL_LIBRARY_DIRS ${OMPL_LIBRARY_DIRS})
else()
    # Fallback: try to find manually
    find_path(OMPL_INCLUDE_DIRS ompl/base/State.h
        PATHS /usr/include /usr/local/include
        PATH_SUFFIXES ompl-1.5 ompl-1.6 ompl)
    
    find_library(OMPL_LIBRARIES
        NAMES ompl
        PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu)
    
    if(OMPL_INCLUDE_DIRS AND OMPL_LIBRARIES)
        set(OMPL_FOUND TRUE)
    endif()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(OMPL DEFAULT_MSG OMPL_LIBRARIES OMPL_INCLUDE_DIRS)

mark_as_advanced(OMPL_INCLUDE_DIRS OMPL_LIBRARIES OMPL_LIBRARY_DIRS)
