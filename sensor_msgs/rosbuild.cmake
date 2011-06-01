include(${CMAKE_CURRENT_BINARY_DIR}/package.cmake)

include_directories(include)

rosbuild_add_library(${PROJECT_NAME} 
  src/image_encodings.cpp src/point_cloud_conversion.cpp src/distortion_models.cpp)
