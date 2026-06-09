set(VVCM_RS_SOURCE_PATH "${CURRENT_PORT_DIR}/../../..")
get_filename_component(VVCM_RS_SOURCE_PATH "${VVCM_RS_SOURCE_PATH}" ABSOLUTE)

if(NOT EXISTS "${VVCM_RS_SOURCE_PATH}/Cargo.toml")
    message(FATAL_ERROR "The vvcm-rs vcpkg overlay port must be used from a vvcm-rs source tree.")
endif()

set(cargo_search_hints)
if(DEFINED ENV{CARGO_HOME} AND NOT "$ENV{CARGO_HOME}" STREQUAL "")
    list(APPEND cargo_search_hints "$ENV{CARGO_HOME}/bin")
endif()
if(VCPKG_HOST_IS_WINDOWS)
    if(DEFINED ENV{USERPROFILE} AND NOT "$ENV{USERPROFILE}" STREQUAL "")
        list(APPEND cargo_search_hints "$ENV{USERPROFILE}/.cargo/bin")
    endif()
elseif(DEFINED ENV{HOME} AND NOT "$ENV{HOME}" STREQUAL "")
    list(APPEND cargo_search_hints "$ENV{HOME}/.cargo/bin")
endif()

if(cargo_search_hints)
    find_program(
        CARGO_EXECUTABLE
        NAMES cargo cargo.exe
        HINTS ${cargo_search_hints}
    )
else()
    find_program(CARGO_EXECUTABLE NAMES cargo cargo.exe)
endif()
if(NOT CARGO_EXECUTABLE)
    message(FATAL_ERROR
        "cargo is required to build vvcm-rs. Install Rust with rustup, reopen your shell so cargo is on PATH, "
        "or set CARGO_HOME to your Rust toolchain directory before running vcpkg."
    )
endif()

function(vvcm_rs_install_profile PROFILE OUTPUT_PROFILE PACKAGE_SUBDIR)
    set(cargo_target_dir "${CURRENT_BUILDTREES_DIR}/${TARGET_TRIPLET}-${PROFILE}")
    set(cargo_args
        build
        --lib
        --locked
        --manifest-path "${VVCM_RS_SOURCE_PATH}/Cargo.toml"
        --target-dir "${cargo_target_dir}"
    )

    if(PROFILE STREQUAL "release")
        list(APPEND cargo_args --release)
    endif()

    vcpkg_execute_required_process(
        COMMAND "${CARGO_EXECUTABLE}" ${cargo_args}
        WORKING_DIRECTORY "${VVCM_RS_SOURCE_PATH}"
        LOGNAME "cargo-build-${TARGET_TRIPLET}-${PROFILE}"
    )

    set(profile_dir "${cargo_target_dir}/${OUTPUT_PROFILE}")
    if(VCPKG_LIBRARY_LINKAGE STREQUAL "dynamic")
        if(VCPKG_TARGET_IS_WINDOWS)
            file(INSTALL "${profile_dir}/vvcm_rs.dll" DESTINATION "${CURRENT_PACKAGES_DIR}/${PACKAGE_SUBDIR}bin")
            file(MAKE_DIRECTORY "${CURRENT_PACKAGES_DIR}/${PACKAGE_SUBDIR}lib")
            configure_file(
                "${profile_dir}/vvcm_rs.dll.lib"
                "${CURRENT_PACKAGES_DIR}/${PACKAGE_SUBDIR}lib/vvcm_rs.lib"
                COPYONLY
            )
        elseif(VCPKG_TARGET_IS_OSX)
            file(INSTALL "${profile_dir}/libvvcm_rs.dylib" DESTINATION "${CURRENT_PACKAGES_DIR}/${PACKAGE_SUBDIR}lib")
        else()
            file(INSTALL "${profile_dir}/libvvcm_rs.so" DESTINATION "${CURRENT_PACKAGES_DIR}/${PACKAGE_SUBDIR}lib")
        endif()
    else()
        if(VCPKG_TARGET_IS_WINDOWS)
            file(INSTALL "${profile_dir}/vvcm_rs.lib" DESTINATION "${CURRENT_PACKAGES_DIR}/${PACKAGE_SUBDIR}lib")
        else()
            file(INSTALL "${profile_dir}/libvvcm_rs.a" DESTINATION "${CURRENT_PACKAGES_DIR}/${PACKAGE_SUBDIR}lib")
        endif()
    endif()
endfunction()

if(NOT DEFINED VCPKG_BUILD_TYPE OR VCPKG_BUILD_TYPE STREQUAL "release")
    vvcm_rs_install_profile(release release "")
endif()

if(NOT DEFINED VCPKG_BUILD_TYPE OR VCPKG_BUILD_TYPE STREQUAL "debug")
    vvcm_rs_install_profile(debug debug "debug/")
endif()

file(INSTALL
    "${VVCM_RS_SOURCE_PATH}/include/vvcm_rs.h"
    "${VVCM_RS_SOURCE_PATH}/include/vvcm_rs.hpp"
    DESTINATION "${CURRENT_PACKAGES_DIR}/include"
)

file(INSTALL
    "${CURRENT_PORT_DIR}/vvcm-rs-config.cmake"
    "${CURRENT_PORT_DIR}/usage"
    DESTINATION "${CURRENT_PACKAGES_DIR}/share/${PORT}"
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "${CURRENT_PACKAGES_DIR}/share/${PORT}/vvcm-rs-config-version.cmake"
    VERSION "${VERSION}"
    COMPATIBILITY SameMajorVersion
)

vcpkg_install_copyright(FILE_LIST "${VVCM_RS_SOURCE_PATH}/LICENSE")
