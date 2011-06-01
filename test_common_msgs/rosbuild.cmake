include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

rosbuild_add_pyunit(test/test_common_msgs_migration.py)

