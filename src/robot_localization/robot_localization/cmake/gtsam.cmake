find_package (GTSAM REQUIRED QUIET)

include_directories(${GTSAM_INCLUDE_DIR})
list(APPEND THIRD_PART_LIBRARIES gtsam)