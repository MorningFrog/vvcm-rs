get_filename_component(_VVCM_RS_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)

if(NOT TARGET vvcm_rs::vvcm_rs)
    add_library(vvcm_rs::vvcm_rs UNKNOWN IMPORTED)
    set_target_properties(vvcm_rs::vvcm_rs PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${_VVCM_RS_PREFIX}/include"
        MAP_IMPORTED_CONFIG_MINSIZEREL RELEASE
        MAP_IMPORTED_CONFIG_RELWITHDEBINFO RELEASE
    )

    find_library(_VVCM_RS_LIBRARY_RELEASE NAMES vvcm_rs PATHS "${_VVCM_RS_PREFIX}/lib" NO_DEFAULT_PATH)
    find_library(_VVCM_RS_LIBRARY_DEBUG NAMES vvcm_rs PATHS "${_VVCM_RS_PREFIX}/debug/lib" NO_DEFAULT_PATH)

    set(_VVCM_RS_IMPORTED_CONFIGS)
    if(_VVCM_RS_LIBRARY_RELEASE)
        list(APPEND _VVCM_RS_IMPORTED_CONFIGS RELEASE)
        set_property(TARGET vvcm_rs::vvcm_rs PROPERTY IMPORTED_LOCATION_RELEASE "${_VVCM_RS_LIBRARY_RELEASE}")
    endif()

    if(_VVCM_RS_LIBRARY_DEBUG)
        list(APPEND _VVCM_RS_IMPORTED_CONFIGS DEBUG)
        set_property(TARGET vvcm_rs::vvcm_rs PROPERTY IMPORTED_LOCATION_DEBUG "${_VVCM_RS_LIBRARY_DEBUG}")
    endif()

    if(NOT _VVCM_RS_IMPORTED_CONFIGS)
        message(FATAL_ERROR "The vvcm-rs library was not found in the vcpkg installation.")
    endif()

    set_property(TARGET vvcm_rs::vvcm_rs PROPERTY IMPORTED_CONFIGURATIONS "${_VVCM_RS_IMPORTED_CONFIGS}")

    if(UNIX)
        find_package(Threads QUIET)
        if(TARGET Threads::Threads)
            set_property(TARGET vvcm_rs::vvcm_rs APPEND PROPERTY INTERFACE_LINK_LIBRARIES Threads::Threads)
        endif()
        if(CMAKE_DL_LIBS)
            set_property(TARGET vvcm_rs::vvcm_rs APPEND PROPERTY INTERFACE_LINK_LIBRARIES "${CMAKE_DL_LIBS}")
        endif()
        if(NOT APPLE)
            set_property(TARGET vvcm_rs::vvcm_rs APPEND PROPERTY INTERFACE_LINK_LIBRARIES m)
        endif()
    endif()
endif()

unset(_VVCM_RS_IMPORTED_CONFIGS)
unset(_VVCM_RS_LIBRARY_DEBUG CACHE)
unset(_VVCM_RS_LIBRARY_RELEASE CACHE)
unset(_VVCM_RS_PREFIX)
