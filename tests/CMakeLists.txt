# goldnet's tests. Built from the top-level CMakeLists (GOLDNET_BUILD_TESTS, on by default):
#
#   cmake -S . -B build
#   cmake --build build
#   ctest --test-dir build --output-on-failure
#
# Tests rely on assert() — keep it live regardless of build type.
add_compile_options(-UNDEBUG)

# UndefinedBehaviorSanitizer, with recovery off so a violation aborts the run rather than
# printing and continuing (ctest keys on the exit code).
#
# Load-bearing, not hygiene. The angle quantizer's crash was a float→uint16 cast of NaN:
# undefined behaviour that a debug Godot build traps on but which, unsanitized, usually
# just yields 0 here — the same value the guarded path produces. Without this the
# non-finite tests in test_codec.cpp pass either way and prove nothing. Verified by
# mutation: reverting the guard fails the suite with this on, and passes with it off.
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
    add_compile_options(-fsanitize=undefined -fno-sanitize-recover=undefined)
    add_link_options(-fsanitize=undefined)
endif()

foreach(test_name test_codec test_seq test_protocol test_netsim test_uplink test_netchan test_auto_rate)
    add_executable(${test_name} ${test_name}.cpp)
    target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(${test_name} PRIVATE goldnet)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
