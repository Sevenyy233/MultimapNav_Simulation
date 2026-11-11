# Loaded by build_livox_workspace.sh, outside the two unmodified vendor trees.
# DEFER runs after each vendor has created its targets.
function(multimap_vendor_compat)
  if(PROJECT_NAME STREQUAL "ros2_livox_simulation")
    find_library(multimap_protobuf NAMES protobuf REQUIRED)
    find_library(multimap_chrono NAMES boost_chrono REQUIRED)
    get_target_property(libraries ros2_livox LINK_LIBRARIES)
    list(TRANSFORM libraries REPLACE "^libprotobuf\\.so\\.9$" "${multimap_protobuf}")
    list(TRANSFORM libraries REPLACE "^libboost_chrono\\.so\\.1\\.71\\.0$" "${multimap_chrono}")
    set_property(TARGET ros2_livox PROPERTY LINK_LIBRARIES "${libraries}")
    message(STATUS "Livox: linking installed Protobuf/Boost without modifying vendor sources")
  elseif(PROJECT_NAME STREQUAL "fast_lio")
    # Upstream opens debug files unconditionally, even with logging disabled.
    # Redirect ROOT_DIR at compile time so running it never writes to src/.
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/runtime/Log" "${CMAKE_BINARY_DIR}/runtime/PCD")
    get_directory_property(definitions COMPILE_DEFINITIONS)
    list(FILTER definitions EXCLUDE REGEX "^ROOT_DIR=")
    list(APPEND definitions "ROOT_DIR=\"${CMAKE_BINARY_DIR}/runtime/\"")
    set_property(DIRECTORY PROPERTY COMPILE_DEFINITIONS "${definitions}")
  endif()
endfunction()

if(PROJECT_NAME STREQUAL "ros2_livox_simulation" OR PROJECT_NAME STREQUAL "fast_lio")
  cmake_language(DEFER CALL multimap_vendor_compat)
endif()
