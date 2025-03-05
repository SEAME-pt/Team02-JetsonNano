# Copyright (c) 2012 - 2017, Lars Bilke
# All rights reserved.

include(CMakeParseArguments)

# Check prereqs
find_program(GCOV_PATH gcov)
find_program(GCOVR_PATH gcovr PATHS ${CMAKE_SOURCE_DIR}/scripts/test)

if(NOT GCOV_PATH)
    message(FATAL_ERROR "gcov not found! Aborting...")
endif()

function(append_coverage_compiler_flags)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage" PARENT_SCOPE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage" PARENT_SCOPE)
endfunction()

function(setup_target_for_coverage_gcovr_html)
    cmake_parse_arguments(Coverage "" "NAME;EXECUTABLE" "DEPENDENCIES;EXCLUDE" ${ARGN})
    
    if(NOT GCOVR_PATH)
        message(FATAL_ERROR "gcovr not found! Aborting...")
    endif()

    # Fixed exclude patterns
    set(GCOVR_EXCLUDE_FLAGS
        --exclude-directories="_deps"
        --exclude-directories="catch2"
        --exclude=".*test.*"
    )

    add_custom_target(${Coverage_NAME}
        # Run tests
        COMMAND ${Coverage_EXECUTABLE}
        
        # Create folder
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/${Coverage_NAME}
        
        # Running gcovr with correct exclude syntax
        COMMAND ${GCOVR_PATH} 
            --html --html-details
            -r ${PROJECT_SOURCE_DIR}
            ${GCOVR_EXCLUDE_FLAGS}
            --object-directory=${PROJECT_BINARY_DIR}
            -o ${PROJECT_BINARY_DIR}/${Coverage_NAME}/index.html
        
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        DEPENDS ${Coverage_DEPENDENCIES}
        COMMENT "Processing code coverage counters and generating report."
    )
endfunction()