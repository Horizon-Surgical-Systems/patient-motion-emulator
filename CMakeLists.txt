cmake_minimum_required(VERSION 3.16)
project(PatientMotionControl VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Qt6 auto-processing (MOC, UIC, RCC)
find_package(Qt6 REQUIRED COMPONENTS Widgets Network)
qt_standard_project_setup()

# ─── Source files ──────────────────────────────────────────────────────────────
set(SOURCES
    src/main.cpp
    src/utils.cpp
    src/meca500_client.cpp
    src/control_window.cpp
)

set(HEADERS
    include/parameter.h
    include/utils.h
    include/meca_client.h
    include/control_window.h
)

qt_add_executable(patient_motion_control
    ${SOURCES}
    ${HEADERS}
)

# ─── Include directories ───────────────────────────────────────────────────────
target_include_directories(patient_motion_control PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# ─── Dynamixel SDK ─────────────────────────────────────────────────────────────
# On macOS: brew install dynamixel-sdk or set DYNAMIXEL_SDK_DIR
# On Linux: install via apt or build from source
#
# Locate the Dynamixel SDK.  Set -DDYNAMIXEL_SDK_DIR=/path/to/sdk on cmake
# command line if it is not in a standard location.
if(DEFINED DYNAMIXEL_SDK_DIR)
    target_include_directories(patient_motion_control PRIVATE
        ${DYNAMIXEL_SDK_DIR}/include/dynamixel_sdk
    )
    if(APPLE)
        target_link_directories(patient_motion_control PRIVATE
            ${DYNAMIXEL_SDK_DIR}/lib
        )
        target_link_libraries(patient_motion_control PRIVATE dxl_x64_mac)
    elseif(UNIX)
        target_link_directories(patient_motion_control PRIVATE
            ${DYNAMIXEL_SDK_DIR}/lib
        )
        target_link_libraries(patient_motion_control PRIVATE dxl_x64_linux)
    elseif(WIN32)
        target_link_directories(patient_motion_control PRIVATE
            ${DYNAMIXEL_SDK_DIR}/lib
        )
        target_link_libraries(patient_motion_control PRIVATE dxl_x64_windows)
    endif()
else()
    # Fallback: try pkg-config or system install
    find_library(DYNAMIXEL_LIB
        NAMES dxl_x64_mac dxl_x64_linux dxl_x86_64-mac dxl_x86_64-linux
        PATHS /usr/local/lib /usr/lib
    )
    if(DYNAMIXEL_LIB)
        target_link_libraries(patient_motion_control PRIVATE ${DYNAMIXEL_LIB})
    else()
        message(WARNING
            "Dynamixel SDK library not found. "
            "Set -DDYNAMIXEL_SDK_DIR=/path/to/sdk or install it system-wide. "
            "Building without Dynamixel support (gimbal features will be stubs).")
        target_compile_definitions(patient_motion_control PRIVATE NO_DYNAMIXEL)
    endif()

    find_path(DYNAMIXEL_INCLUDE_DIR
        NAMES dynamixel_sdk.h
        PATHS /usr/local/include /usr/include
        PATH_SUFFIXES dynamixel_sdk
    )
    if(DYNAMIXEL_INCLUDE_DIR)
        target_include_directories(patient_motion_control PRIVATE ${DYNAMIXEL_INCLUDE_DIR})
    endif()
endif()

# ─── Qt + system libraries ─────────────────────────────────────────────────────
target_link_libraries(patient_motion_control PRIVATE
    Qt6::Widgets
    Qt6::Network
)

if(UNIX)
    target_link_libraries(patient_motion_control PRIVATE pthread)
endif()

# ─── Compiler warnings ─────────────────────────────────────────────────────────
if(MSVC)
    target_compile_options(patient_motion_control PRIVATE /W4)
else()
    target_compile_options(patient_motion_control PRIVATE
        -Wall -Wextra -Wpedantic -Wno-unused-parameter
    )
endif()

# ─── Install (optional) ────────────────────────────────────────────────────────
# install(TARGETS patient_motion_control
#     RUNTIME DESTINATION bin
# )
# install(DIRECTORY head_motion_profile eye_motion_profile
#     DESTINATION share/patient_motion_control
# )
