cmake_minimum_required(VERSION 3.20)

project(op1emu)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

# Fetch spdlog
message(STATUS "Fetching spdlog...")
FetchContent_Declare(
    spdlog
    GIT_REPOSITORY https://github.com/gabime/spdlog.git
    GIT_TAG v1.11.0
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(spdlog)
message(STATUS "spdlog fetched successfully")

# Fetch GLFW
message(STATUS "Fetching GLFW...")
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL " " FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL " " FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL " " FORCE)
set(GLFW_INSTALL OFF CACHE BOOL " " FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(glfw)
message(STATUS "GLFW fetched successfully")
find_package(OpenGL REQUIRED)

# Fetch nlohmann/json
message(STATUS "Fetching nlohmann/json...")
FetchContent_Declare(
    json
    GIT_REPOSITORY https://github.com/nlohmann/json.git
    GIT_TAG v3.12.0
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(json)
message(STATUS "nlohmann/json fetched successfully")

# Fetch stb (for image loading)
message(STATUS "Fetching stb...")
FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG master
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(stb)
message(STATUS "stb fetched successfully")

# Fetch cqueue
message(STATUS "Fetching cqueue...")
FetchContent_Declare(cqueue
  GIT_REPOSITORY https://github.com/torrentg/cqueue.git
  GIT_TAG        1.0.6
)
FetchContent_MakeAvailable(cqueue)
message(STATUS "cqueue fetched successfully")
add_library(cqueue INTERFACE)
target_include_directories(cqueue INTERFACE ${cqueue_SOURCE_DIR})

# Fetch uvw
message(STATUS "Fetching uvw...")
FetchContent_Declare(
    uvw
    GIT_REPOSITORY https://github.com/skypjack/uvw.git
    GIT_TAG        v3.4.0_libuv_v1.48
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(BUILD_UVW_LIBS ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(uvw)
message(STATUS "uvw fetched successfully")

# Fetch miniaudio (single-header audio library)
message(STATUS "Fetching miniaudio...")
FetchContent_Declare(
    miniaudio
    GIT_REPOSITORY https://github.com/mackron/miniaudio.git
    GIT_TAG 0.11.21
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_MakeAvailable(miniaudio)
message(STATUS "miniaudio fetched successfully")

find_package(Threads REQUIRED)

# Measurement build (docs/profiling.md). OFF compiles every profiling hook out
# of the emulator and of bcore, so a normal build carries none of it.
option(ENABLE_PROFILING "Build the profiling hooks: phase marks, trace, census, perf/jitdump" OFF)
set(BCORE_ENABLE_PROFILE ${ENABLE_PROFILING} CACHE BOOL "bcore observability API (follows ENABLE_PROFILING)" FORCE)
# LLVM's perf JIT listener (jitdump) is Linux-only.
if(ENABLE_PROFILING AND NOT APPLE)
    set(_perf_jit ON)
else()
    set(_perf_jit OFF)
endif()
set(BCORE_PERF_JIT_EVENTS ${_perf_jit} CACHE BOOL "bcore jitdump support (follows ENABLE_PROFILING, Linux only)" FORCE)
if(ENABLE_PROFILING)
    # perf call graphs through host code need frame pointers.
    add_compile_options(-fno-omit-frame-pointer)
endif()

add_subdirectory(ext/bcore EXCLUDE_FROM_ALL)

file(GLOB SOURCES ext/cqueue/*.cpp)
add_library(ext ${SOURCES})
# use C++20 for ext
set_target_properties(ext PROPERTIES CXX_STANDARD 20)
target_include_directories(ext PUBLIC ext)
target_link_libraries(ext PRIVATE cqueue)

file(GLOB_RECURSE SOURCES src/*.cpp)
if(NOT ENABLE_PROFILING)
    list(FILTER SOURCES EXCLUDE REGEX "/src/profiling/")
endif()
add_library(emulator ${SOURCES})
target_include_directories(emulator PUBLIC src)
target_include_directories(emulator PRIVATE ext)
target_link_libraries(emulator PRIVATE bfin-core spdlog::spdlog)
if(ENABLE_PROFILING)
    target_compile_definitions(emulator PUBLIC ENABLE_PROFILING)
    target_link_libraries(emulator PRIVATE nlohmann_json::nlohmann_json)
endif()

# Interpret code the JIT is still compiling (async compile) with the GDB Blackfin simulator's interpreter.
# ext/bfin_sim is GPL: with this ON the op1emu binary combines MIT and GPL code.
option(OP1EMU_INTERP "Run uncompiled code on ext/bfin_sim while the JIT compiles it (links GPL code)" ON)
if(OP1EMU_INTERP)
    enable_language(C)
    # A patched copy (src/cpu/bfin-sim-op1emu.patch: flag fixes so the interpreter agrees with the JIT);
    # the submodule stays as upstream has it.
    set(_bfinsim_dir ${CMAKE_CURRENT_BINARY_DIR}/bfinsim)
    file(MAKE_DIRECTORY ${_bfinsim_dir})
    add_custom_command(
        OUTPUT ${_bfinsim_dir}/bfin-sim.c
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/ext/bfin_sim/bfin-sim.c ${_bfinsim_dir}/bfin-sim.c
        COMMAND patch -s -p0 -d ${_bfinsim_dir} -i ${CMAKE_CURRENT_SOURCE_DIR}/src/cpu/bfin-sim-op1emu.patch
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/ext/bfin_sim/bfin-sim.c ${CMAKE_CURRENT_SOURCE_DIR}/src/cpu/bfin-sim-op1emu.patch
        COMMENT "Patching bfin-sim.c"
        VERBATIM)
    add_library(bfinsim STATIC ${_bfinsim_dir}/bfin-sim.c src/cpu/interp_glue.c)
    target_include_directories(bfinsim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ext/bfin_sim src/cpu)
    target_compile_options(bfinsim PRIVATE -w)
    # bcore exports cec_* with C linkage too: give the simulator's side its own names.
    foreach(sym cec_cli cec_sti cec_exception cec_hwerr cec_latch cec_pop_reti cec_push_reti cec_return
                cec_is_user_mode cec_require_supervisor mmu_check_addr mmu_check_cache_addr mmu_process_fault
                bfin_trace_queue sim_core_read_buffer sim_core_write_buffer interp_insn_bfin hwloop_get_next_pc)
        target_compile_definitions(bfinsim PRIVATE ${sym}=bfsim_${sym})
    endforeach()
    target_link_libraries(emulator PRIVATE bfinsim)
    target_compile_definitions(emulator PRIVATE OP1EMU_INTERP=1)
endif()

add_executable(ldrdump tools/ldrdump.cpp)
target_link_libraries(ldrdump emulator)

file(GLOB_RECURSE SOURCES gui/*.cpp)
add_executable(op1emu ${SOURCES})
target_link_libraries(op1emu PRIVATE emulator glfw OpenGL::GL nlohmann_json::nlohmann_json uvw ext Threads::Threads ${CMAKE_DL_LIBS})
target_include_directories(op1emu PRIVATE ${stb_SOURCE_DIR} ${miniaudio_SOURCE_DIR})

option(OP1_BUILD_HEADLESS_TESTS "Build headless frontend regressions" OFF)
if(OP1_BUILD_HEADLESS_TESTS)
    enable_testing()
    add_executable(op1-headless-test tests/headless_test.cpp gui/headless.cpp)
    target_include_directories(op1-headless-test PRIVATE gui)
    target_link_libraries(op1-headless-test PRIVATE emulator nlohmann_json::nlohmann_json Threads::Threads)
    add_test(NAME op1-headless-test COMMAND op1-headless-test)
    set_tests_properties(op1-headless-test PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    add_executable(op1-audio-input-test tests/audio_input_test.cpp)
    target_include_directories(op1-audio-input-test PRIVATE src gui)
    add_test(NAME op1-audio-input-test COMMAND op1-audio-input-test)
    add_executable(op1-time-source-test tests/time_source_test.cpp)
    target_include_directories(op1-time-source-test PRIVATE src)
    add_test(NAME op1-time-source-test COMMAND op1-time-source-test)
endif()

if(BCORE_BUILD_FAULT_TESTS)
    enable_testing()
    add_executable(op1-data-fault-test tests/data_fault_test.cpp)
    target_link_libraries(op1-data-fault-test PRIVATE emulator bfin-core)
    add_test(NAME op1-data-fault-test COMMAND op1-data-fault-test)
endif()
