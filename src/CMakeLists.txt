cmake_minimum_required(VERSION 3.22)

# TODO: set up VCPKG
message(NOTICE "Preset VCPKG triplet: ${VCPKG_TARGET_TRIPLET}")
set(MY_VCPKG_TARGET_TRIPLET "x64-windows" CACHE STRING "Custom triplet for VCPKG")
if(VCPKG_TARGET_TRIPLET)
    message(NOTICE "Using preset VCPKG triplet: ${VCPKG_TARGET_TRIPLET}")
else()
   set(VCPKG_TARGET_TRIPLET "${MY_VCPKG_TARGET_TRIPLET}")
    message(NOTICE "Using custuom VCPKG triplet: ${VCPKG_TARGET_TRIPLET}")
endif()

project(RelativePoseFromTrajectory
        VERSION 1.0
        DESCRIPTION "Estimates relative pose of two sensors (cameras) based on their observed trajectory"
        LANGUAGES CXX)
        
# TODO: Set up compiler
        
find_package(g2o CONFIG REQUIRED)

add_subdirectory(rel_pose_from_trajectory)
