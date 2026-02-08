#cmake_minimum_required(VERSION 3.0)
#project(CALIB_CAM)

#find_package(OpenCV REQUIRED)
#include_directories(${OpenCV_INCLUDE_DIRS})
#include_directories(${PROJECT_SOURCE_DIR})

#add_executable(calibrate calib_intrinsic.cpp)
#target_link_libraries(calibrate ${OpenCV_LIBS})

#add_executable(read read_images.cpp)
#target_link_libraries(read ${OpenCV_LIBS})

#add_executable(calibrate_stereo calib_stereo.cpp)
#target_link_libraries(calibrate_stereo ${OpenCV_LIBS})

#add_executable(undistort_rectify undistort_rectify.cpp)
#target_link_libraries(undistort_rectify ${OpenCV_LIBS})



cmake_minimum_required(VERSION 3.0)
project(CALIB_CAM)

# 1. 寻找 OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# 2. 内参标定程序 (已完成)
add_executable(calibrate calib_intrinsic.cpp)
target_link_libraries(calibrate ${OpenCV_LIBS})

# 3. 双目标定程序 (当前目标)
# 注意：必须添加 target_link_libraries 才能正常编译
add_executable(calibrate_stereo calib_stereo.cpp)
target_link_libraries(calibrate_stereo ${OpenCV_LIBS})

# 其他程序暂时保持屏蔽
# add_executable(read read_images.cpp)
# add_executable(undistort_rectify undistort_rectify.cpp)