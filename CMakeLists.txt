CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)
if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project(STEREOCAMCALIB)

option(ENABLE_STATIC "Build static (.a) library" ON)

# Build and install shared library
add_library(stereocamcalib SHARED "stereocalibration.cpp" "cameracalibration.cpp" "path.cpp")
# target_link_libraries(stereocamcalib)
install(TARGETS "stereocamcalib" LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)

# Build and install static library
if(ENABLE_STATIC)
    add_library(stereocamcalib-static STATIC "stereocalibration.cpp" "cameracalibration.cpp" "path.cpp")
    set_target_properties(stereocamcalib-static PROPERTIES OUTPUT_NAME "stereocamcalib")
    install(TARGETS "stereocamcalib-static" ARCHIVE DESTINATION lib)
endif(ENABLE_STATIC)

# Install header files
install(FILES "stereocalibration.h" "cameracalibration.h" "path.h" DESTINATION include)

# target_link_libraries(stereocamcalib)
