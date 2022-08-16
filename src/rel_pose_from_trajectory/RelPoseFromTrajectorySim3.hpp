#pragma once

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/se3quat.h>

#include <g2o/types/sim3/sim3.h>
#include <g2o/core/optimization_algorithm.h>

#include <vector>

namespace g2o
{
	class EdgeSE3Prior;
	class ParameterSE3Offset;
	class VertexSE3;
	class VertexSim3Expmap;
}


namespace RelPose
{
	namespace impl
	{
		class LocalSlaveSensorPoseSim3Edge;
	}

	class RelPoseFromTrajectorySim3
	{
	public:

		RelPoseFromTrajectorySim3();

		std::size_t addPosePair(
			const g2o::SE3Quat& mainSensorMeasuredPose,
			const Eigen::Matrix<double, 6, 6>& mainSensorMeasuredPoseInfoMat,
			const g2o::SE3Quat& slaveSensorMeasuredPose,
			const Eigen::Matrix<double, 6, 6>& slaveSensorMeasuredPoseInfoMat
		);

		double estimate(const g2o::SE3Quat& initSlaveLeverArmEstimate,
			const g2o::Sim3* initSlaveLocalCoordinateSpace = nullptr,
			int iterCount = 30);

		g2o::SE3Quat getEstimatedSlaveLeverArm() const;
		g2o::Sim3 getEstimatedSlaveLocalCoordinateSpace() const;


		inline std::size_t poseCount() const { return mainSensorPosesVertices_.size(); }

	private: // definitions


	private:
		g2o::SparseOptimizer optimizer_;
		g2o::OptimizationAlgorithm* optimisationAlgorithm_;
		int graphIdGenerator_ = 0;
		int globalOffsetParameterId_ = -1;
		g2o::ParameterSE3Offset* globalOffsetParameter_ = nullptr;

		// Main sensor poses - estimates and observations
		std::vector<g2o::VertexSE3*> mainSensorPosesVertices_;
		std::vector<g2o::EdgeSE3Prior*> mainSensorPoseObservations_;

		// Slave sensor poses - only observations
		std::vector<impl::LocalSlaveSensorPoseSim3Edge*> slaveSensorPoseObservations_;

		// - Lever arm - this will contain the estimated result
		g2o::VertexSE3* relativeSlaveSensorPoseVertex_ = nullptr;

		// - Slave sensor local coordinate space
		g2o::VertexSim3Expmap* localSlaveCoordinateSpaceVertex_ = nullptr;

	};
}