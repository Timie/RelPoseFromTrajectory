#pragma once


#include <g2o/types/slam3d/se3quat.h>
#include <vector>

namespace RelPose
{
	class RelPoseFromTrajectorySE3
	{
	public:

		static g2o::SE3Quat estimateLeverArm(
			const std::vector<g2o::SE3Quat>& mainSensorMeasuredTrajectory,
			const std::vector<g2o::SE3Quat>& slaveSensorMeasuredTrajectory,
			const g2o::SE3Quat& initRelativeSensorPoseEstim,
			const g2o::SE3Quat& initSlaveSensorLocalCoordinateSpaceEstim // TODO: REMOVE! We should calculate this from initRelativeSensorPoseEstim 
		);
	private:
	};
}