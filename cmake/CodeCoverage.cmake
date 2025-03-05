
# Find required tools
find_program(GCOVR_PATH gcovr)

if(NOT GCOVR_PATH)
    message(FATAL_ERROR "gcovr not found!")
endif()

# Add compiler flags for coverage
function(append_coverage_compiler_flags)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage" PARENT_SCOPE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -fprofile-arcs -ftest-coverage" PARENT_SCOPE)
endfunction()

# Setup target for coverage using gcovr
function(setup_target_for_coverage_gcovr_html)
    cmake_parse_arguments(Coverage "" "NAME;EXECUTABLE" "DEPENDENCIES;EXCLUDE" ${ARGN})
    
    # Setup coverage filters for source files
    set(COVERAGE_FILTERS
        --filter "${PROJECT_SOURCE_DIR}/src/.*"
    )

    add_custom_target(${Coverage_NAME}
        # Run tests
        COMMAND ${Coverage_EXECUTABLE}
        
        # Create folder
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/${Coverage_NAME}
        
        # Generate HTML report
        COMMAND ${GCOVR_PATH}
            --html --html-details
            -r ${PROJECT_SOURCE_DIR}
            --object-directory=${PROJECT_BINARY_DIR}
            ${COVERAGE_FILTERS}
            -o ${PROJECT_BINARY_DIR}/${Coverage_NAME}/index.html
        
        WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
        DEPENDS ${Coverage_DEPENDENCIES}
        COMMENT "Processing code coverage counters and generating HTML reports."
    )
endfunction()