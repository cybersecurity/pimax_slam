cmake_minimum_required(VERSION 3.20)
project(pimax_slam C CXX)

# Reconstruction of pimax_slam.pi.dll (Pimax_SLAM_2.0.0.1).
# Must be built for x64 Windows with the MSVC ABI, because it links the C++ APIs of the prebuilt
# OpenCV 4.5.3 DLLs shipped with the Pimax runtime. On Linux use cmake/clang-cl-xwin.cmake
# (clang-cl + lld-link + xwin MSVC sysroot).

# The original was built with MSVC's default /std:c++14 (operands of `<<` in log statements are
# evaluated before the stream object is constructed, which C++17 forbids). clang-cl needs C++17 for
# some of the dependencies' headers, so the project code is built as C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreadedDLL")

set(TP ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

# The original is an optimized build with asserts left on (Eigen asserts call _wassert).
string(REPLACE "/DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
string(REPLACE "-DNDEBUG" "" CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE}")
string(REPLACE "/DNDEBUG" "" CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE}")
set(PS_COMMON_FLAGS /EHsc /bigobj /utf-8)
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    list(APPEND PS_COMMON_FLAGS -Wno-everything)
endif()
set(PS_COMMON_DEFS _USE_MATH_DEFINES NOMINMAX _CRT_SECURE_NO_WARNINGS _SILENCE_ALL_CXX17_DEPRECATION_WARNINGS
    BOOST_ALL_NO_LIB BOOST_SERIALIZATION_STATIC_LINK BOOST_FILESYSTEM_STATIC_LINK)

# ---------------------------------------------------------------------------------------------
# glog 0.5.0 + gflags 2.2.2, static (used by the project code).
include(ExternalProject)
set(DEPS_PREFIX ${CMAKE_BINARY_DIR}/deps)
set(DEPS_ARGS
    -G ${CMAKE_GENERATOR}
    -DCMAKE_BUILD_TYPE=Release
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
    -DCMAKE_INSTALL_PREFIX=${DEPS_PREFIX}
    -DCMAKE_MSVC_RUNTIME_LIBRARY=MultiThreadedDLL)
if(CMAKE_TOOLCHAIN_FILE)
    list(APPEND DEPS_ARGS -DCMAKE_TOOLCHAIN_FILE=${CMAKE_TOOLCHAIN_FILE})
endif()
if(XWIN_DIR)
    list(APPEND DEPS_ARGS -DXWIN_DIR=${XWIN_DIR})
endif()
ExternalProject_Add(gflags_ext
    SOURCE_DIR ${TP}/gflags
    CMAKE_ARGS ${DEPS_ARGS} -DBUILD_SHARED_LIBS=OFF -DBUILD_STATIC_LIBS=ON -DBUILD_TESTING=OFF
    BUILD_BYPRODUCTS ${DEPS_PREFIX}/lib/gflags_static.lib)
ExternalProject_Add(glog_ext
    DEPENDS gflags_ext
    SOURCE_DIR ${TP}/glog
    CMAKE_ARGS ${DEPS_ARGS} -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF -DWITH_GFLAGS=ON
               -Dgflags_DIR=${DEPS_PREFIX}/lib/cmake/gflags -DWITH_UNWIND=OFF
    BUILD_BYPRODUCTS ${DEPS_PREFIX}/lib/glog.lib)
set(GLOG_DEFS GLOG_NO_ABBREVIATED_SEVERITIES GLOG_STATIC_DEFINE GOOGLE_GLOG_DLL_DECL= GFLAGS_IS_A_DLL=0)

if(NOT EXISTS ${TP}/ceres-solver/include/ceres/ceres.h)
    message(FATAL_ERROR "third_party submodules missing: git submodule update --init --depth 1")
endif()

# ---------------------------------------------------------------------------------------------
# Ceres Solver 2.1.0, static, MINIGLOG (its own logging, separate from the project's glog),
# EIGEN_SPARSE, C++ threads, all Schur specializations (third_party/ceres-config).
# third_party/ceres-solver is the pristine 2.1.0 submodule; the files Pimax patched come from the
# third_party/ceres-pimax overlay (its internal/ directory is searched first, for parameter_block.h).
include(cmake/ceres_sources.cmake)
list(TRANSFORM CERES_UPSTREAM_SOURCES PREPEND ${TP}/ceres-solver/)
list(TRANSFORM CERES_PIMAX_SOURCES PREPEND ${TP}/ceres-pimax/)
add_library(ceres STATIC ${CERES_UPSTREAM_SOURCES} ${CERES_PIMAX_SOURCES})
target_compile_options(ceres PRIVATE ${PS_COMMON_FLAGS})
target_compile_definitions(ceres PRIVATE ${PS_COMMON_DEFS} CERES_BUILDING_SHARED_LIBRARY=0 MAX_LOG_LEVEL=2)
target_include_directories(ceres
    PUBLIC ${TP}/ceres-config ${TP}/ceres-solver/include ${TP}/eigen
    PRIVATE ${TP}/ceres-pimax/internal ${TP}/ceres-solver/internal ${TP}/ceres-solver/internal/ceres/miniglog)

# ---------------------------------------------------------------------------------------------
# Boost 1.74: serialization (binary archives) + filesystem, static. Headers: third_party/boost.
set(BOOST ${TP}/boost)
set(BS ${BOOST}/libs/serialization/src)
add_library(boost_serialization STATIC
    ${BS}/archive_exception.cpp ${BS}/basic_archive.cpp ${BS}/basic_iarchive.cpp ${BS}/basic_iserializer.cpp
    ${BS}/basic_oarchive.cpp ${BS}/basic_oserializer.cpp ${BS}/basic_pointer_iserializer.cpp
    ${BS}/basic_pointer_oserializer.cpp ${BS}/basic_serializer_map.cpp ${BS}/binary_iarchive.cpp
    ${BS}/binary_oarchive.cpp ${BS}/codecvt_null.cpp ${BS}/extended_type_info.cpp
    ${BS}/extended_type_info_no_rtti.cpp ${BS}/extended_type_info_typeid.cpp ${BS}/stl_port.cpp
    ${BS}/utf8_codecvt_facet.cpp ${BS}/void_cast.cpp)
file(GLOB BOOST_FS_SRC ${BOOST}/libs/filesystem/src/*.cpp)
add_library(boost_filesystem STATIC ${BOOST_FS_SRC})
foreach(t boost_serialization boost_filesystem)
    target_include_directories(${t} PUBLIC ${BOOST})
    target_compile_definitions(${t} PRIVATE ${PS_COMMON_DEFS})
    target_compile_options(${t} PRIVATE ${PS_COMMON_FLAGS})
endforeach()

# ---------------------------------------------------------------------------------------------
add_library(tinyxml2 STATIC ${TP}/tinyxml2/tinyxml2.cpp)
target_include_directories(tinyxml2 PUBLIC ${TP}/tinyxml2)

# Import libraries for the prebuilt OpenCV DLLs of the Pimax runtime, built from the export lists in
# third_party/implib/*.def (taken from the DLLs' export tables) with the librarian (lib.exe / llvm-lib).
set(OPENCV_LIBS)
foreach(m core imgproc calib3d features2d imgcodecs highgui aruco)
    set(def ${TP}/implib/opencv_${m}453.def)
    set(lib ${CMAKE_BINARY_DIR}/implib/opencv_${m}453.lib)
    add_custom_command(OUTPUT ${lib}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/implib
        COMMAND ${CMAKE_AR} /nologo /machine:x64 /def:${def} /out:${lib}
        DEPENDS ${def} VERBATIM)
    list(APPEND OPENCV_LIBS ${lib})
endforeach()
add_custom_target(opencv_implibs DEPENDS ${OPENCV_LIBS})

# ---------------------------------------------------------------------------------------------
# The DLL. Project sources (src/) plus the in-tree, Pimax-modified third-party code that the original
# compiled into the same target: vikit (vendored, Pimax-modified), DBoW2 + the Pimax DBoW3-style loader
# (submodule + third_party/DBoW2-pimax overlay), QuickLZ (DBow3 submodule), FAST (submodule +
# third_party/fast-pimax overlay).
file(GLOB_RECURSE PS_SOURCES CONFIGURE_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
file(GLOB_RECURSE PS_TP_SOURCES CONFIGURE_DEPENDS
    ${TP}/vikit/*.cpp ${TP}/DBoW2-pimax/src/*.cpp ${TP}/fast-pimax/src/*.cpp)
list(APPEND PS_TP_SOURCES
    ${TP}/DBoW2/src/BowVector.cpp ${TP}/DBoW2/src/FeatureVector.cpp ${TP}/DBoW2/src/FORB.cpp
    ${TP}/DBoW2/src/ScoringObject.cpp
    ${TP}/fast/src/fast_10.cpp ${TP}/fast/src/fast_10_score.cpp ${TP}/fast/src/nonmax_3x3.cpp
    ${TP}/DBow3/src/quicklz.c)
set(OPENCV_INCLUDES ${TP}/opencv-config ${TP}/opencv/include)
foreach(m core imgproc calib3d features2d flann imgcodecs highgui videoio)
    list(APPEND OPENCV_INCLUDES ${TP}/opencv/modules/${m}/include)
endforeach()
list(APPEND OPENCV_INCLUDES ${TP}/opencv_contrib/modules/aruco/include)
set(PS_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${TP}/eigen
    ${TP}/ceres-config
    ${TP}/ceres-solver/include
    ${OPENCV_INCLUDES}
    ${BOOST}
    ${TP}/minkindr-pimax/include
    ${TP}/minkindr/minkindr/include
    ${TP}/vikit/include
    ${TP}/DBoW2-pimax/include
    ${TP}/DBoW2-pimax/include/DBoW2
    ${TP}/DBoW2/include
    ${TP}/DBoW2/include/DBoW2
    ${TP}/fast/include
    ${TP}/DBow3/src
    ${TP}/tinyxml2
    ${DEPS_PREFIX}/include)

add_library(pimax_slam SHARED ${PS_SOURCES} ${PS_TP_SOURCES} src/pimax_slam.def)
set_target_properties(pimax_slam PROPERTIES OUTPUT_NAME "pimax_slam.pi" PREFIX "")
add_dependencies(pimax_slam glog_ext opencv_implibs)
target_compile_options(pimax_slam PRIVATE ${PS_COMMON_FLAGS})
target_compile_definitions(pimax_slam PRIVATE ${PS_COMMON_DEFS} ${GLOG_DEFS})

# Test build for differential runs: no wall-clock solver budget in Estimator::optimize and a fixed
# std::mt19937 seed in StereoTriangulation::compute (the original gets the same treatment from
# test/make_deterministic_orig.py). Off for the real build.
option(PIMAX_SLAM_TEST_DETERMINISTIC "deterministic test build (see test/README)" OFF)
if(PIMAX_SLAM_TEST_DETERMINISTIC)
    target_compile_definitions(pimax_slam PRIVATE PIMAX_SLAM_TEST_DETERMINISTIC)
endif()
# Test builds: initial level of the printf logger (1 debug .. 4 error; the shipped value is 3). The original
# test copy gets the same with test/make_deterministic_orig.py --log-level.
set(PIMAX_SLAM_TEST_LOG_LEVEL "" CACHE STRING "test build: initial logger level (empty = 3 as shipped)")
if(NOT PIMAX_SLAM_TEST_LOG_LEVEL STREQUAL "")
    target_compile_definitions(pimax_slam PRIVATE PIMAX_SLAM_TEST_LOG_LEVEL=${PIMAX_SLAM_TEST_LOG_LEVEL})
endif()
# Debug info (CodeView + PDB) for symbolizing wine backtraces; does not change code generation.
option(PIMAX_SLAM_DEBUG_INFO "emit /Z7 debug info and a PDB" OFF)
if(PIMAX_SLAM_DEBUG_INFO)
    target_compile_options(pimax_slam PRIVATE /Z7)
    target_link_options(pimax_slam PRIVATE /DEBUG)
endif()
target_include_directories(pimax_slam PRIVATE ${PS_INCLUDES})
target_link_libraries(pimax_slam PRIVATE
    ceres boost_serialization boost_filesystem tinyxml2
    ${DEPS_PREFIX}/lib/glog.lib ${DEPS_PREFIX}/lib/gflags_static.lib
    ${OPENCV_LIBS}
    shlwapi.lib dbghelp.lib)

# Helper for tools/cc.sh: dump the flags used for the DLL's translation units.
file(GENERATE OUTPUT ${CMAKE_BINARY_DIR}/ps_includes.txt CONTENT "$<JOIN:${PS_INCLUDES},\n>\n")
file(GENERATE OUTPUT ${CMAKE_BINARY_DIR}/ps_defines.txt CONTENT "$<JOIN:${PS_COMMON_DEFS};${GLOG_DEFS},\n>\n")
