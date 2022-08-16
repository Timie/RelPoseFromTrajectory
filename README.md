# Relative Pose From Trajectory
Estimate the relative pose of two sensors given their trajectories expressed in different coordinate spaces. Their coordinate spaces may be related by rotation, translation and uniform scale, all unknown to the user. This is handy in cases when you e.g. want to calculate relative pose of two motion sensors or motion sensor and a reference point with known movement.

Example of application: two cameras mounted on the robot without any overlap in their field of view (which makes usual stereo calibration impossible).

## Status
This project is work in progress. At the current state, it cannot be used directly. However, it shows that the approach used in the implementation works well.

## Usage

The project currently does not offer a library, but it already provides two classes with somehow usable interface:
* RelPoseFromTrajectorySE3 - for case when the unknown coordinate spaces of trajectories are related by rigid transformation - rotation and translation.
* RelPoseFromTrajectorySim3 - for case when the unknown coordinate spaces of trajectories are related by similarity transformation - uniform scale, rotation, and translation.

Just copy them to your C++ project.

## Dependencies

* g2o - https://github.com/RainerKuemmerle/g2o - BSD License - tested with version 2020-02-07 (port 2) from VCPKG.
* Eigen - https://eigen.tuxfamily.org - MPL2 licence - tested with version 3.4.0

