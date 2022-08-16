add_executable(rel_pose_from_trajectory
               rel_pose_from_trajectory.cpp
               RelPoseFromTrajectorySE3.cpp
               RelPoseFromTrajectorySE3.hpp
               RelPoseFromTrajectorySim3.cpp
               RelPoseFromTrajectorySim3.hpp)
               
               
target_include_directories(rel_pose_from_trajectory
                           PUBLIC
                           ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(rel_pose_from_trajectory
                      PUBLIC
                      g2o::core
                      # g2o::stuff
                      g2o::types_data
                      g2o::types_slam2d
                      g2o::types_slam3d
                      g2o::types_sba
                      g2o::types_sim3
                      g2o::types_icp
                      g2o::types_sclam2d
                      g2o::types_slam2d_addons
                      g2o::types_slam3d_addons
                      # g2o::solver_pcg
                      # g2o::solver_dense
                      # g2o::solver_structure_only
                      # g2o::solver_csparse
                      # g2o::csparse_extension
                      # g2o::solver_slam2d_linear
                      # g2o::solver_cholmod
                      g2o::solver_eigen
                      Eigen3::Eigen
                      )