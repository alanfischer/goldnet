if(GOLDNET_BUILD_EXAMPLES)
    add_executable(goldnet_demo demo_terminal.cpp)
    target_link_libraries(goldnet_demo PRIVATE goldnet)
endif()

# shove: a multiplayer game on goldnet, hop and raylib. raylib comes from find_package if it's
# installed, and is fetched otherwise; hop is fetched at a pinned commit, or taken from HOP_DIR.
if(GOLDNET_BUILD_SHOVE)
    include(FetchContent)

    find_package(raylib QUIET)
    if(NOT raylib_FOUND)
        FetchContent_Declare(raylib
            GIT_REPOSITORY https://github.com/raysan5/raylib.git
            GIT_TAG 5.5
            GIT_SHALLOW TRUE)
        set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
        FetchContent_MakeAvailable(raylib)
    endif()

    if(NOT TARGET hop)
        set(HOP_DIR "" CACHE PATH "A local hop checkout; empty fetches the pinned one")
        set(HOP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
        if(HOP_DIR)
            add_subdirectory(${HOP_DIR} ${CMAKE_BINARY_DIR}/hop)
        else()
            FetchContent_Declare(hop
                GIT_REPOSITORY https://github.com/alanfischer/hop.git
                GIT_TAG 1a2916849955123fecc4ed2cab06c5a3299088b0)
            FetchContent_MakeAvailable(hop)
        endif()
    endif()

    add_executable(shove shove/main.cpp)
    target_link_libraries(shove PRIVATE goldnet hop raylib)
endif()
