#define _USE_MATH_DEFINES

#include <iostream>

#include <g2o/core/solver.h>
#include <g2o/types/icp/types_icp.h>
#include <g2o/types/slam3d/se3quat.h>
#include <g2o/core/hyper_graph.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/block_solver.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <g2o/types/sim3/sim3.h>
#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <vector>
#include <cmath>
#include <iostream>
#include <random>

#include <RelPoseFromTrajectorySE3.hpp>
#include <RelPoseFromTrajectorySim3.hpp>


namespace my
{
    template <typename TRndEngine>
    g2o::SE3Quat makeRandomSE3Pose(const Eigen::Matrix<double, 6, 6>& mainSensorPoseCovarianceSqrt, TRndEngine& rndEng)
    {
        std::normal_distribution<double> numDist(0, 1);
        Eigen::Vector<double, 6> data;
        data(0) = numDist(rndEng);
        data(1) = numDist(rndEng);
        data(2) = numDist(rndEng);
        data(3) = numDist(rndEng);
        data(4) = numDist(rndEng);
        data(5) = numDist(rndEng);

        data = mainSensorPoseCovarianceSqrt * data;

        g2o::SE3Quat result;
        result.fromMinimalVector(data);
        return result;
    }


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

    class LocalSlaveSensorPoseEdge : public g2o::BaseMultiEdge<6, g2o::Isometry3>
    {
    public:
        LocalSlaveSensorPoseEdge(
            g2o::VertexSE3 *mainSensorPoseVertex,
            g2o::VertexSE3* relativeSlaveSensorPoseVertex,
            g2o::VertexSE3* localSlaveCoordinateSpaceVertex)
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
            g2o::VertexSE3* localSlaveCoordinateSpaceVertex = dynamic_cast<g2o::VertexSE3*>(vertices()[2]);

            assert(mainSensorPoseVertex != nullptr);
            assert(relativeSlaveSensorPoseVertex != nullptr);
            assert(localSlaveCoordinateSpaceVertex != nullptr);

            const auto& localSlaveSensorPose = measurement();
            const auto& mainSensorPose = mainSensorPoseVertex->estimate();
            const auto& relativeSlaveSensorPose = relativeSlaveSensorPoseVertex->estimate(); // aka, lever arm
            const auto& localSlaveCoordinateSpace = localSlaveCoordinateSpaceVertex->estimate();

            const g2o::Isometry3 error = (localSlaveCoordinateSpace * mainSensorPose * relativeSlaveSensorPose).inverse() * localSlaveSensorPose;
            _error = g2o::internal::toVectorMQT(error);
        }

        virtual bool read(std::istream& is) override { return true; }
        virtual bool write(std::ostream& is) const override { return true; }
    };
}

void simulateMovement(const g2o::SE3Quat &mainSensorPoseChange,
                      std::vector<g2o::SE3Quat>& trueMainSensorTrajectory,
                      std::vector<g2o::SE3Quat>& mainSensorMeasuredTrajectory,
                      std::vector<g2o::SE3Quat>& slaveSensorMeasuredTrajectory,
                      g2o::SE3Quat trueRelativeSensorPose)
{
    assert(!trueMainSensorTrajectory.empty());
    assert(!mainSensorMeasuredTrajectory.empty());
    assert(!slaveSensorMeasuredTrajectory.empty());

    {
        const auto& oldPose = trueMainSensorTrajectory.back();
        trueMainSensorTrajectory.push_back(oldPose * mainSensorPoseChange);
    }


    {
        const auto& oldPose = mainSensorMeasuredTrajectory.back();
        mainSensorMeasuredTrajectory.push_back(oldPose * mainSensorPoseChange);
    }

    {
        const auto& sensor2Movement = trueRelativeSensorPose.inverse() * mainSensorPoseChange * trueRelativeSensorPose;
        const auto& oldPose = slaveSensorMeasuredTrajectory.back();
        slaveSensorMeasuredTrajectory.push_back(oldPose * sensor2Movement);
    }
}

int main(int argc, const char *argv[])
{
    // Assumptions:
    // Z goes forward (depth)
    // X goes right
    // Y goes up

    g2o::SE3Quat trueRelativeSensorPose; // this should be recovered
    trueRelativeSensorPose.setTranslation(g2o::Vector3(1.8f, 2.1f, -1.2f));
    trueRelativeSensorPose.setRotation(g2o::Quaternion(1, 2, 3, 4).normalized());

    g2o::SE3Quat masterSensorLocalCoordinateSpace;
    masterSensorLocalCoordinateSpace.setTranslation(g2o::Vector3(0,0,0));
    masterSensorLocalCoordinateSpace.setRotation(g2o::Quaternion::Identity().normalized());
    //mainSensorLocalCoordinateSpace.setTranslation(g2o::Vector3(-1.0f, 4.3f, 12.0f));
    //mainSensorLocalCoordinateSpace.setRotation(g2o::Quaternion(-8, 0, 12, 3).normalized());

    g2o::SE3Quat slaveSensorLocalCoordinateSpace;
    slaveSensorLocalCoordinateSpace.setTranslation(g2o::Vector3(1.0f, 8.1f, 2.0f));
    slaveSensorLocalCoordinateSpace.setRotation(g2o::Quaternion(-5, 5, 12, 8).normalized());


    std::vector<g2o::SE3Quat> trueMainSensorTrajectory;
    std::vector<g2o::SE3Quat> mainSensorMeasuredTrajectory;
    std::vector<g2o::SE3Quat> slaveSensorMeasuredTrajectory;

    // Add first pose - identity
    trueMainSensorTrajectory.push_back(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3::Zero()));
    mainSensorMeasuredTrajectory.push_back(masterSensorLocalCoordinateSpace);
    slaveSensorMeasuredTrajectory.push_back(slaveSensorLocalCoordinateSpace * trueRelativeSensorPose);

    // move forward.
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(0, 0, 1)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);
    // Move right
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(1, 0, 0)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    // Move down
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(0, -1, 0)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    // Turn left
    simulateMovement(g2o::SE3Quat(g2o::Quaternion(Eigen::AngleAxis<double>(M_PI / 2, g2o::Vector3(0, 1, 0))), g2o::Vector3::Zero()),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);


    // move forward.
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(0, 0, 1)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);
    // Move Left
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(-1, 0, 0)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    // Turn down
    simulateMovement(g2o::SE3Quat(g2o::Quaternion(Eigen::AngleAxis<double>(M_PI / 2, g2o::Vector3(1, 0, 0))), g2o::Vector3::Zero()),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    // Move up
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(0, 1, 0)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    // Move back
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(0, 0, -1)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    // Tilt right up
    simulateMovement(g2o::SE3Quat(g2o::Quaternion(Eigen::AngleAxis<double>(M_PI / 2, g2o::Vector3(0, 0, 1))), g2o::Vector3::Zero()),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    // Move right
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(1, 0, 0)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    // Move down
    simulateMovement(g2o::SE3Quat(g2o::Quaternion::Identity(), g2o::Vector3(0, -1, 0)),
                     trueMainSensorTrajectory, mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory,
                     trueRelativeSensorPose);

    g2o::SE3Quat initRelativeSensorPoseEstim = trueRelativeSensorPose
        /* * g2o::SE3Quat(g2o::Quaternion(1, 0.1, 0.1, -0.1).normalized(), g2o::Vector3(0.1, -0.1, 0.1)) */; // here we add a little bit of error to the estimate
    const auto& estimatedLeverArm = RelPose::RelPoseFromTrajectorySE3::estimateLeverArm(mainSensorMeasuredTrajectory, slaveSensorMeasuredTrajectory, initRelativeSensorPoseEstim, slaveSensorLocalCoordinateSpace);

    std::cout << "True lever arm: \n" << trueRelativeSensorPose << "\n";
    std::cout << "Estimated lever arm from lib: \n" << estimatedLeverArm << "\n";

    

    // Now try Sim3 slave sensor local coordinate space
    {
        double slaveSensorLocalCoordScale = 0.5;
        g2o::Sim3 slaveSensorLocalCoordinateSpaceSim3(
            slaveSensorLocalCoordinateSpace.rotation(), 
            slaveSensorLocalCoordinateSpace.translation() * slaveSensorLocalCoordScale, 
            slaveSensorLocalCoordScale);


        std::vector<g2o::SE3Quat> slaveSensorMeasuredTrajectorySim3;
        slaveSensorMeasuredTrajectorySim3.reserve(slaveSensorMeasuredTrajectory.size());
        for (const g2o::SE3Quat& originalMeasuredSlavePosition : slaveSensorMeasuredTrajectory)
        {
            slaveSensorMeasuredTrajectorySim3.emplace_back(originalMeasuredSlavePosition.rotation(), originalMeasuredSlavePosition.translation() * slaveSensorLocalCoordScale);
        }

        {
            RelPose::RelPoseFromTrajectorySim3 relPoseSim3;
            for (std::size_t measurementIdx = 0; measurementIdx < mainSensorMeasuredTrajectory.size(); ++measurementIdx)
            {
                const auto& mainSensorMeasuredPose = mainSensorMeasuredTrajectory[measurementIdx];
                const auto& slaveSensorMeasuredPose = slaveSensorMeasuredTrajectorySim3[measurementIdx];
                relPoseSim3.addPosePair(
                    mainSensorMeasuredPose,
                    Eigen::Matrix<double, 6, 6>::Identity() * 10,
                    slaveSensorMeasuredPose,
                    Eigen::Matrix<double, 6, 6>::Identity() * 10
                );
            }

            double relPoseSim3Chi2 = relPoseSim3.estimate(
                initRelativeSensorPoseEstim,
                nullptr, //&slaveSensorLocalCoordinateSpaceSim3,
                30);

            std::cout << "Sim3 Lib Chi Squared: \n" << relPoseSim3Chi2 << "\n";

            std::cout << "Sim3 True lever arm: \n" << trueRelativeSensorPose << "\n";
            std::cout << "Sim3 Estimated lever arm from lib: \n" << relPoseSim3.getEstimatedSlaveLeverArm() << "\n";


            std::cout << "Sim3 True slave local coordinate space: \n" << slaveSensorLocalCoordinateSpaceSim3 << "\n";
            std::cout << "Sim3 Estimated slave local coordinate space from lib: \n" << relPoseSim3.getEstimatedSlaveLocalCoordinateSpace() << "\n";
        }


        // Try adding some noise
        const auto& mainSensorPoseCovarianceSqrt = Eigen::DiagonalMatrix<double, 6>(0.02, 0.02, 0.02, 0.01, 0.01, 0.01);
        const auto& mainSensorPoseCovariance = Eigen::DiagonalMatrix<double, 6>(mainSensorPoseCovarianceSqrt.diagonal().array().pow(2).matrix());
        const auto& mainSensorPoseInfo = mainSensorPoseCovariance.inverse();
        const auto& slaveSensorPoseCovarianceSqrt = mainSensorPoseCovarianceSqrt;
        const auto& slaveSensorPoseCovariance = mainSensorPoseCovariance;
        const auto& slaveSensorPoseInfo = mainSensorPoseCovariance.inverse();

        std::default_random_engine rndEng;

        {
            RelPose::RelPoseFromTrajectorySim3 relPoseSim3;
            for (std::size_t measurementIdx = 0; measurementIdx < mainSensorMeasuredTrajectory.size(); ++measurementIdx)
            {
                const auto& mainSensorMeasuredPose = mainSensorMeasuredTrajectory[measurementIdx] * my::makeRandomSE3Pose(mainSensorPoseCovarianceSqrt, rndEng);
                const auto& slaveSensorMeasuredPose = slaveSensorMeasuredTrajectorySim3[measurementIdx] * my::makeRandomSE3Pose(slaveSensorPoseCovarianceSqrt, rndEng);
                relPoseSim3.addPosePair(
                    mainSensorMeasuredPose,
                    mainSensorPoseInfo,
                    slaveSensorMeasuredPose,
                    slaveSensorPoseInfo
                );
            }

            double relPoseSim3Chi2 = relPoseSim3.estimate(
                initRelativeSensorPoseEstim * my::makeRandomSE3Pose(Eigen::DiagonalMatrix<double, 6>(0.1, 0.1, 0.1, 0.05, 0.05, 0.05), rndEng),
                nullptr, // &slaveSensorLocalCoordinateSpaceSim3
                30);

            std::cout << "Noisy Sim3 Lib Chi Squared: \n" << relPoseSim3Chi2 << "\n";

            std::cout << "Noisy Sim3 True lever arm: \n" << trueRelativeSensorPose << "\n";
            std::cout << "Noisy Sim3 Estimated lever arm from lib: \n" << relPoseSim3.getEstimatedSlaveLeverArm() << "\n";


            std::cout << "Noisy Sim3 True slave local coordinate space: \n" << slaveSensorLocalCoordinateSpaceSim3 << "\n";
            std::cout << "Noisy Sim3 Estimated slave local coordinate space from lib: \n" << relPoseSim3.getEstimatedSlaveLocalCoordinateSpace() << "\n";
        }

        
    }

    return 0;
}
