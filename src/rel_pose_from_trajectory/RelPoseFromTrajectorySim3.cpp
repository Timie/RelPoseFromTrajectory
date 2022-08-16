#include "RelPoseFromTrajectorySim3.hpp"

#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <g2o/types/slam3d/edge_se3_prior.h>

#include <iostream> // TODO: remove this. Just for debug

namespace RelPose
{
    namespace impl
    {
        class LocalSlaveSensorPoseSim3Edge : public g2o::BaseMultiEdge<6, g2o::Isometry3>
        {
        public:
            LocalSlaveSensorPoseSim3Edge(
                g2o::VertexSE3* mainSensorPoseVertex,
                g2o::VertexSE3* relativeSlaveSensorPoseVertex,
                g2o::VertexSim3Expmap* localSlaveCoordinateSpaceVertex)
            {
                assert(mainSensorPoseVertex != nullptr);
                assert(relativeSlaveSensorPoseVertex != nullptr);
                assert(localSlaveCoordinateSpaceVertex != nullptr);

                resize(3);
                setVertex(0, mainSensorPoseVertex);
                setVertex(1, relativeSlaveSensorPoseVertex);
                setVertex(2, localSlaveCoordinateSpaceVertex);
            }

            virtual void computeError() override
            {
                assert(vertices().size() == 3);

                g2o::VertexSE3* mainSensorPoseVertex = dynamic_cast<g2o::VertexSE3*>(vertices()[0]);
                g2o::VertexSE3* relativeSlaveSensorPoseVertex = dynamic_cast<g2o::VertexSE3*>(vertices()[1]);
                g2o::VertexSim3Expmap* localSlaveCoordinateSpaceVertex = dynamic_cast<g2o::VertexSim3Expmap*>(vertices()[2]);

                assert(mainSensorPoseVertex != nullptr);
                assert(relativeSlaveSensorPoseVertex != nullptr);
                assert(localSlaveCoordinateSpaceVertex != nullptr);

                const auto& localSlaveSensorPose = measurement();
                const auto& mainSensorPose = mainSensorPoseVertex->estimate();
                const auto& relativeSlaveSensorPose = relativeSlaveSensorPoseVertex->estimate(); // aka, lever arm
                const auto& localSlaveCoordinateSpace = localSlaveCoordinateSpaceVertex->estimate();

                auto estimatedSlaveSensorPose = mainSensorPose * relativeSlaveSensorPose;
                estimatedSlaveSensorPose.translation() *= localSlaveCoordinateSpace.scale();
                auto expectedMeasurement =
                    g2o::Isometry3(g2o::SE3Quat(localSlaveCoordinateSpace.rotation(), localSlaveCoordinateSpace.translation()))
                    * estimatedSlaveSensorPose;

                const g2o::Isometry3 error = expectedMeasurement.inverse() * localSlaveSensorPose;
                _error = g2o::internal::toVectorMQT(error);
            }

            virtual bool read(std::istream& is) override { return true; }
            virtual bool write(std::ostream& is) const override { return true; }
        };
    }
}

RelPose::RelPoseFromTrajectorySim3::RelPoseFromTrajectorySim3():
    optimisationAlgorithm_()
{
    optimizer_.setVerbose(true);

    // allocate the solver
    optimisationAlgorithm_ = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<
            g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>())
        );

    optimizer_.setAlgorithm(optimisationAlgorithm_);

    // Prepare global vertices
    relativeSlaveSensorPoseVertex_ = new g2o::VertexSE3();
    // TODO: relativeSlaveSensorPoseVertex->setEstimate(initRelativeSensorPoseEstim);
    relativeSlaveSensorPoseVertex_->setId(++graphIdGenerator_);

    bool addVertexRes = optimizer_.addVertex(relativeSlaveSensorPoseVertex_);
    if (!addVertexRes)
    {
        throw std::runtime_error("Failed to add vertex.");
    }

    localSlaveCoordinateSpaceVertex_ = new g2o::VertexSim3Expmap();
    // TODO: localSlaveCoordinateSpaceVertex_->setEstimate(slaveSensorLocalCoordinateSpaceSim3);
    localSlaveCoordinateSpaceVertex_->setId(++graphIdGenerator_);

    addVertexRes = optimizer_.addVertex(localSlaveCoordinateSpaceVertex_);
    if (!addVertexRes)
    {
        throw std::runtime_error("Failed to add vertex.");
    }

    // We need to add ParameterSE3Offset, otherwise the g2o is having troubles with prior edges. :/
    globalOffsetParameter_ = new g2o::ParameterSE3Offset();
    globalOffsetParameter_->setOffset();
    globalOffsetParameterId_ = 0;
    globalOffsetParameter_->setId(globalOffsetParameterId_);
    optimizer_.addParameter(globalOffsetParameter_);
}


std::size_t
RelPose::RelPoseFromTrajectorySim3::
addPosePair(
    const g2o::SE3Quat& mainSensorMeasuredPose,
    const Eigen::Matrix<double, 6, 6>& mainSensorMeasuredPoseInfoMat,
    const g2o::SE3Quat& slaveSensorMeasuredPose,
    const Eigen::Matrix<double, 6, 6>& slaveSensorMeasuredPoseInfoMat)
{
    // Main sensor pose vertex
    g2o::VertexSE3* mainSensorPosesVertex = new g2o::VertexSE3();
    mainSensorPosesVertex->setEstimate(mainSensorMeasuredPose);
    mainSensorPosesVertex->setId(++graphIdGenerator_);

    bool addVertexRes = optimizer_.addVertex(mainSensorPosesVertex);
    if (!addVertexRes)
    {
        throw std::runtime_error("Failed to add vertex.");
    }
    mainSensorPosesVertices_.push_back(mainSensorPosesVertex);

    // Main sensor pose prior edge
    g2o::EdgeSE3Prior* mainSensorPoseObservation = new g2o::EdgeSE3Prior();
    mainSensorPoseObservation->setVertex(0, mainSensorPosesVertex);
    mainSensorPoseObservation->setMeasurement(mainSensorMeasuredPose);
    mainSensorPoseObservation->setInformation(mainSensorMeasuredPoseInfoMat);
    mainSensorPoseObservation->setId(++graphIdGenerator_);
    mainSensorPoseObservation->setParameterId(0, globalOffsetParameterId_); // the first parameter must be global offset. :O No idea why.
    bool addEdgeRes = optimizer_.addEdge(mainSensorPoseObservation);
    if (!addEdgeRes)
    {
        throw std::runtime_error("Failed to add edge.");
    }
    mainSensorPoseObservations_.push_back(mainSensorPoseObservation);

    // slave sensor pose observations
    impl::LocalSlaveSensorPoseSim3Edge* slaveSensorPoseObservation = new impl::LocalSlaveSensorPoseSim3Edge(
        mainSensorPosesVertex, relativeSlaveSensorPoseVertex_, localSlaveCoordinateSpaceVertex_);
    slaveSensorPoseObservation->setMeasurement(slaveSensorMeasuredPose);
    slaveSensorPoseObservation->setInformation(slaveSensorMeasuredPoseInfoMat);
    slaveSensorPoseObservation->setId(++graphIdGenerator_);
    slaveSensorPoseObservation->setParameterId(0, globalOffsetParameterId_); // the first parameter must be global offset. :O No idea why.
    addEdgeRes = optimizer_.addEdge(slaveSensorPoseObservation);
    if (!addEdgeRes)
    {
        throw std::runtime_error("Failed to add edge.");
    }
    slaveSensorPoseObservations_.push_back(slaveSensorPoseObservation);

    return poseCount() - 1;
}

double
RelPose::RelPoseFromTrajectorySim3::
estimate(
    const g2o::SE3Quat& initSlaveLeverArmEstimate, 
    const g2o::Sim3* initSlaveLocalCoordinateSpace, 
    int iterCount)
{
    if (iterCount > 0)
    {
        if (initSlaveLocalCoordinateSpace == nullptr)
        {
            // Estimate it as follows:
            // Scale = ratio of distance traveled
            const auto pairCnt = mainSensorPosesVertices_.size();
            double observedTrajectoryLenght = 0;
            double predictedTrajectoryLenght = 0;
            for (std::size_t idx = 1; idx < pairCnt; ++idx)
            {
                const auto& preObservedTranslation = slaveSensorPoseObservations_[idx - 1]->measurement().translation();
                const auto& observedTranslation = slaveSensorPoseObservations_[idx]->measurement().translation();
                observedTrajectoryLenght += (preObservedTranslation - observedTranslation).norm();
            
                const auto& prevPredictedSlaveSensorPose = mainSensorPosesVertices_[idx - 1]->estimateAsSE3Quat() * initSlaveLeverArmEstimate;
                const auto& predictedSlaveSensorPose = mainSensorPosesVertices_[idx]->estimateAsSE3Quat() * initSlaveLeverArmEstimate;
                predictedTrajectoryLenght += (prevPredictedSlaveSensorPose.translation() - predictedSlaveSensorPose.translation()).norm();
            }
            
            double scale = observedTrajectoryLenght / predictedTrajectoryLenght;

            // Transformation: As average of per-pair transforms.
            // Derived as follows (see LocalSlaveSensorPoseSim3Edge):
            // Psl = Csl * scale(Pm La) => Csl = Psl (scale(Pm La))-1
            // Where:
            // * Psl = observed slave pose (in slave local coordinate space)
            // * Csl = slave local coordinate space
            // * Pm = observed main sensor pose
            // * La = slave lever arm (in main sensor coordinate frame)
            const g2o::Isometry3 invSlaveLeverArmEstimate = initSlaveLeverArmEstimate.inverse();
            g2o::Vector6 accumSlaveLocalCoordSpace(0, 0, 0, 0, 0, 0);
            for (std::size_t idx = 0; idx < pairCnt; ++idx)
            {
                const auto& mainSensorPose = mainSensorPosesVertices_[idx]->estimate();
                g2o::Isometry3 estimSlaveSensorPose = (mainSensorPose * initSlaveLeverArmEstimate).inverse();
                estimSlaveSensorPose.translation() *= (scale);

                const auto& observedSlaveSensorPose = slaveSensorPoseObservations_[idx]->measurement();
            
                g2o::Isometry3 estimatedLocalCoordSpace = observedSlaveSensorPose  * estimSlaveSensorPose;
                accumSlaveLocalCoordSpace += g2o::internal::toVectorMQT(estimatedLocalCoordSpace);
            }
            
            accumSlaveLocalCoordSpace *= (1.0 / pairCnt);
            
            g2o::SE3Quat slaveLocalCoordSpace;
            slaveLocalCoordSpace.fromMinimalVector(accumSlaveLocalCoordSpace);
            
            g2o::Sim3 slaveLocalCoordSpaceWithScale =
                g2o::Sim3(slaveLocalCoordSpace.rotation(), slaveLocalCoordSpace.translation(), scale);

            
            localSlaveCoordinateSpaceVertex_->setEstimate(slaveLocalCoordSpaceWithScale);
        }
        else
        {
            localSlaveCoordinateSpaceVertex_->setEstimate(*initSlaveLocalCoordinateSpace);
        }

        relativeSlaveSensorPoseVertex_->setEstimate(initSlaveLeverArmEstimate);

        if (!optimizer_.initializeOptimization())
        {
            throw std::runtime_error("Optimiser initialisation failed.");
        }

        optimizer_.computeActiveErrors();
        std::cout << "Sim3 Initial chi2 = " << FIXED(optimizer_.chi2()) << std::endl;

        optimizer_.setVerbose(true);

        optimizer_.optimize(iterCount);
    }

    return optimizer_.activeChi2();
}

g2o::SE3Quat RelPose::RelPoseFromTrajectorySim3::getEstimatedSlaveLeverArm() const
{
    return relativeSlaveSensorPoseVertex_->estimateAsSE3Quat();
}

g2o::Sim3 RelPose::RelPoseFromTrajectorySim3::getEstimatedSlaveLocalCoordinateSpace() const
{
    return localSlaveCoordinateSpaceVertex_->estimate();
}
