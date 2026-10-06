# Cross-compile for x64 Windows (MSVC ABI) with clang-cl + lld-link and an xwin-splatted
# MSVC CRT / Windows SDK (default location: ~/.xwin, override with -DXWIN_DIR=...).
set(CMAKE_SYSTEM_NAME Windows)
set(CMAKE_SYSTEM_PROCESSOR AMD64)
set(CMAKE_C_COMPILER clang-cl)
set(CMAKE_CXX_COMPILER clang-cl)
set(CMAKE_LINKER lld-link)
set(CMAKE_AR llvm-lib)
set(CMAKE_RC_COMPILER llvm-rc)
set(CMAKE_MT llvm-mt)

if(NOT XWIN_DIR)
    set(XWIN_DIR "$ENV{HOME}/.xwin")
endif()

set(_xwin_flags
    "--target=x86_64-pc-windows-msvc"
    "-fms-compatibility-version=19.29"
    "/imsvc${XWIN_DIR}/crt/include"
    "/imsvc${XWIN_DIR}/sdk/include/ucrt"
    "/imsvc${XWIN_DIR}/sdk/include/um"
    "/imsvc${XWIN_DIR}/sdk/include/shared"
    "-Wno-unused-command-line-argument")
string(JOIN " " _xwin_flags_str ${_xwin_flags})
set(CMAKE_C_FLAGS_INIT "${_xwin_flags_str}")
set(CMAKE_CXX_FLAGS_INIT "${_xwin_flags_str}")

set(_xwin_link "/libpath:${XWIN_DIR}/crt/lib/x86_64 /libpath:${XWIN_DIR}/sdk/lib/um/x86_64 /libpath:${XWIN_DIR}/sdk/lib/ucrt/x86_64")
set(CMAKE_EXE_LINKER_FLAGS_INIT "${_xwin_link}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_xwin_link}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_xwin_link}")

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
