cmake_minimum_required(VERSION 3.16)
project(arbotix_sim C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# simavr v1.8 from third_party/simavr.
# simavr uses its own Makefile, so CMake runs make and links the result.
pkg_check_modules(LIBELF REQUIRED IMPORTED_TARGET libelf)
find_program(MAKE_EXECUTABLE NAMES gmake make REQUIRED)
include(ExternalProject)
include(ProcessorCount)
ProcessorCount(NPROC)

set(SIMAVR_DIR ${CMAKE_SOURCE_DIR}/third_party/simavr/simavr)
if(NOT EXISTS ${SIMAVR_DIR}/Makefile)
    message(FATAL_ERROR "third_party/simavr is empty. Run: git submodule update --init")
endif()

# simavr puts the library in obj-<machine name>/ e.g. obj-x86_64-linux-gnu/
execute_process(
    COMMAND ${CMAKE_C_COMPILER} -dumpmachine
    OUTPUT_VARIABLE SIMAVR_TRIPLE
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(SIMAVR_LIB ${SIMAVR_DIR}/obj-${SIMAVR_TRIPLE}/libsimavr.a)
set(SIMAVR_MAKE ${MAKE_EXECUTABLE} -C ${SIMAVR_DIR} RELEASE=1 CC=${CMAKE_C_COMPILER})

ExternalProject_Add(simavr_build
    SOURCE_DIR        ${SIMAVR_DIR}
    CONFIGURE_COMMAND ""
    # "config" must run first: it creates headers the library build needs
    BUILD_COMMAND     ${SIMAVR_MAKE} config
    COMMAND           ${SIMAVR_MAKE} -j${NPROC} libsimavr
    BUILD_IN_SOURCE   1
    BUILD_ALWAYS      1   # make only rebuilds what changed
    INSTALL_COMMAND   ""
    BUILD_BYPRODUCTS  ${SIMAVR_LIB}
)

add_library(simavr STATIC IMPORTED)
set_target_properties(simavr PROPERTIES IMPORTED_LOCATION ${SIMAVR_LIB})
target_include_directories(simavr INTERFACE ${SIMAVR_DIR}/sim)
target_link_libraries(simavr INTERFACE PkgConfig::LIBELF)
# simavr uses libdwarf when it's installed, so link it too
if(EXISTS /usr/include/libdwarf/libdwarf.h)
    target_link_libraries(simavr INTERFACE dwarf)
endif()
add_dependencies(simavr simavr_build)

# Arbotix Core Simulator
add_library(core STATIC
    core/simulator.c
    core/dynbus.c
    core/servo.c
)
target_include_directories(core PUBLIC core)
target_link_libraries(core PUBLIC simavr m Threads::Threads)
if(NOT APPLE)
    # openpty() needs -lutil on Linux; on macOS it's already in libc.
    target_link_libraries(core PUBLIC util)
endif()
target_compile_options(core PRIVATE
    -Wall -Wextra -Wno-missing-field-initializers
)

# GUI front end (Dear ImGui + GLFW + OpenGL3).
pkg_check_modules(GLFW REQUIRED IMPORTED_TARGET glfw3)
find_package(OpenGL REQUIRED)

set(IMGUI_DIR third_party/imgui)
add_library(imgui STATIC
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
)
target_link_libraries(imgui PUBLIC PkgConfig::GLFW OpenGL::GL)

add_executable(arbotix_gui gui/main.cpp)
target_link_libraries(arbotix_gui PRIVATE core imgui)
target_compile_options(arbotix_gui PRIVATE -Wall -Wextra)

if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
  pkg_check_modules(LIBEVDEV REQUIRED IMPORTED_TARGET libevdev)
  target_sources(arbotix_gui PRIVATE gui/ps_input.cpp)
  target_link_libraries(arbotix_gui PRIVATE PkgConfig::LIBEVDEV)
  target_compile_definitions(arbotix_gui PRIVATE HAVE_PS_INPUT=1)
endif()
