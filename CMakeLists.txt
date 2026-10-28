cmake_minimum_required(VERSION 3.16)
project(arbotix_sim C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SIMAVR REQUIRED simavr)
find_package(Threads REQUIRED)

if(APPLE)
    # simavr's pkg-config file links against -lelf but doesn't carry a path
    # to wherever Homebrew put libelf (a separate package), so that -lelf
    # reference can't resolve on its own. Ask Homebrew for the real
    # directory and add it to the link search path.
    find_program(HOMEBREW_EXECUTABLE brew)
    if(HOMEBREW_EXECUTABLE)
        execute_process(
            COMMAND ${HOMEBREW_EXECUTABLE} --prefix libelf
            OUTPUT_VARIABLE HOMEBREW_LIBELF_PREFIX
            OUTPUT_STRIP_TRAILING_WHITESPACE
            ERROR_QUIET
        )
    endif()
    if(NOT HOMEBREW_LIBELF_PREFIX)
        message(FATAL_ERROR "libelf not found. Install it with: brew install libelf")
    endif()
endif()

# Arbotix Core Simulator
add_library(core STATIC
    core/simulator.c
    core/dynbus.c
    core/servo.c
)
target_include_directories(core PUBLIC
    ${SIMAVR_INCLUDE_DIRS}
    core
)
target_link_directories(core PUBLIC ${SIMAVR_LIBRARY_DIRS})
if(APPLE)
    target_link_directories(core PUBLIC "${HOMEBREW_LIBELF_PREFIX}/lib")
endif()
target_link_libraries(core PUBLIC ${SIMAVR_LIBRARIES} m Threads::Threads)
if(NOT APPLE)
    # openpty() needs -lutil on Linux; on macOS it's already in libc.
    target_link_libraries(core PUBLIC util)
endif()
target_compile_options(core PRIVATE
    -Wall -Wextra -Wno-missing-field-initializers ${SIMAVR_CFLAGS_OTHER}
)

# GUI front end (Dear ImGui + GLFW + OpenGL3).
pkg_check_modules(GLFW REQUIRED glfw3)
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
    ${GLFW_INCLUDE_DIRS}
)
target_link_directories(imgui PUBLIC ${GLFW_LIBRARY_DIRS})
target_link_libraries(imgui PUBLIC ${GLFW_LIBRARIES} OpenGL::GL)

add_executable(arbotix_gui gui/main.cpp)
target_link_libraries(arbotix_gui PRIVATE core imgui Threads::Threads)
target_compile_options(arbotix_gui PRIVATE -Wall -Wextra)
