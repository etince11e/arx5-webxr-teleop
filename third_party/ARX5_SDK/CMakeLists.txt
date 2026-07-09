cmake_minimum_required( VERSION 3.16.3 )

project( arx5-sdk )

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # global PIC

# Prefer conda-managed dependencies over system packages.
if(DEFINED ENV{CONDA_PREFIX})
    list(PREPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
endif()

# Use spdlog's bundled fmt. MUST be set before find_package(spdlog).
set(SPDLOG_FMT_EXTERNAL OFF CACHE BOOL "Use spdlog bundled fmt, not system fmt" FORCE)

find_package(Threads REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(spdlog REQUIRED)

message(STATUS "spdlog version: ${spdlog_VERSION}")
get_target_property(_SPDLOG_INC spdlog::spdlog INTERFACE_INCLUDE_DIRECTORIES)
message(STATUS "spdlog include dirs: ${_SPDLOG_INC}")
get_target_property(_SPDLOG_LIBS spdlog::spdlog INTERFACE_LINK_LIBRARIES)
message(STATUS "spdlog link libs: ${_SPDLOG_LIBS}")
if(TARGET fmt::fmt)
    get_target_property(_FMT_INC fmt::fmt INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "fmt include dirs: ${_FMT_INC}")
    message(STATUS "fmt version: ${fmt_VERSION}")
else()
    message(STATUS "fmt: using spdlog bundled fmt (no external fmt::fmt target)")
endif()

# ---------------------------------------------------------------------------
# Avoid python-site spdlog header pollution
# ---------------------------------------------------------------------------
# In some conda setups, spdlog's CMake package may contribute include paths
# from: $CONDA_PREFIX/include/python3.x/spdlog.
# Those headers can shadow conda's C++ spdlog headers (and can break
# spdlog/fmt compatibility). For our ARX5_SDK build we want to use only
# headers coming from: $CONDA_PREFIX/include (not the python-specific tree).
if(DEFINED ENV{CONDA_PREFIX})
    set(_ARX5_CONDA_INCLUDE_DIR "$ENV{CONDA_PREFIX}/include")

    # Filter imported target include dirs to remove python3.x include tree.
    if(TARGET spdlog::spdlog)
        get_target_property(_ARX5_SPDLOG_INC spdlog::spdlog INTERFACE_INCLUDE_DIRECTORIES)
        if(_ARX5_SPDLOG_INC)
            set(_ARX5_SPDLOG_INC_FILTERED "")
            foreach(_d IN LISTS _ARX5_SPDLOG_INC)
                if(NOT _d MATCHES "include/python3\\.[0-9]+/spdlog")
                    list(APPEND _ARX5_SPDLOG_INC_FILTERED "${_d}")
                endif()
            endforeach()
            # Also ensure conda include dir is present.
            if(NOT _ARX5_CONDA_INCLUDE_DIR IN_LIST _ARX5_SPDLOG_INC_FILTERED)
                list(APPEND _ARX5_SPDLOG_INC_FILTERED "${_ARX5_CONDA_INCLUDE_DIR}")
            endif()
            set_target_properties(spdlog::spdlog PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${_ARX5_SPDLOG_INC_FILTERED}")
        endif()
    endif()
endif()

# ---------------------------------------------------------------------------
# RPATH: embed conda lib dir so the built .so files find spdlog at runtime
# without needing LD_LIBRARY_PATH.
# ---------------------------------------------------------------------------
set(CMAKE_INSTALL_RPATH "$ENV{CONDA_PREFIX}/lib")
set(CMAKE_BUILD_RPATH   "$ENV{CONDA_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

add_compile_definitions(SDK_ROOT="${CMAKE_BINARY_DIR}/..")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm.*|arm|aarch64")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/aarch64)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86_64)|(AMD64)|(^i686)")
    set(LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/x86_64)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(
    # IMPORTANT:
    # Do NOT add $ENV{CONDA_PREFIX}/include/ wholesale here.
    # It can shadow the spdlog headers selected by find_package(spdlog)
    # (and therefore break spdlog/fmt compatibility), because our build
    # relies on spdlog's bundled fmt (see SPDLOG_FMT_EXTERNAL OFF).
    #
    # Keep only the specific conda-provided header dirs that are required.
    $ENV{CONDA_PREFIX}/include/urdfdom_headers
)

# Add conda lib directory to link directories
link_directories($ENV{CONDA_PREFIX}/lib)

add_library(ArxJointController SHARED
    src/app/joint_controller.cpp
    src/app/controller_base.cpp
    src/utils.cpp
)
target_link_libraries(ArxJointController
    ${LIB_DIR}/libhardware.so
    ${LIB_DIR}/libsolver.so
    Eigen3::Eigen
    Threads::Threads
    spdlog::spdlog
    kdl_parser::kdl_parser
    orocos-kdl
    soem
)
add_library(ArxCartesianController SHARED
    src/app/cartesian_controller.cpp
    src/app/controller_base.cpp
    src/utils.cpp
)
target_link_libraries(ArxCartesianController
    ${LIB_DIR}/libhardware.so
    ${LIB_DIR}/libsolver.so
    Eigen3::Eigen
    Threads::Threads
    spdlog::spdlog
    kdl_parser::kdl_parser
    orocos-kdl
    soem
)

# Hack for py310 (conda environment is slightly different from other python versions)
target_include_directories(ArxJointController PUBLIC $ENV{CONDA_PREFIX}/include/kdl_parser  $ENV{CONDA_PREFIX}/include/urdfdom_headers)
target_include_directories(ArxCartesianController PUBLIC $ENV{CONDA_PREFIX}/include/kdl_parser  $ENV{CONDA_PREFIX}/include/urdfdom_headers)


add_executable(test_joint_controller examples/test_joint_controller.cpp)
target_link_libraries(test_joint_controller
    ${LIB_DIR}/libhardware.so
    ${LIB_DIR}/libsolver.so
    ArxJointController
    Eigen3::Eigen
    Threads::Threads
    spdlog::spdlog
    kdl_parser::kdl_parser
    orocos-kdl
    soem
)

add_executable(test_cartesian_controller examples/test_cartesian_controller.cpp)
target_link_libraries(test_cartesian_controller
    ${LIB_DIR}/libhardware.so
    ${LIB_DIR}/libsolver.so
    ArxCartesianController
    spdlog::spdlog
    Eigen3::Eigen
    Threads::Threads
    kdl_parser::kdl_parser
    orocos-kdl
    soem
)

add_subdirectory(python)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}
)

install(FILES ${LIB_DIR}/libhardware.so ${LIB_DIR}/libsolver.so
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)

install(TARGETS ArxJointController ArxCartesianController
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
)
