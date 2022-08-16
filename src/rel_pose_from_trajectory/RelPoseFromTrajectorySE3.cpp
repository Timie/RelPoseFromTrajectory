#include "RelPoseFromTrajectorySE3.hpp"

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

namespace
{
    class LocalSlaveSensorPoseEdge : public g2o::BaseMultiEdge<6, g2o::Isometry3>
    {
    public:
        LocalSlaveSensorPoseEdge(
            g2o::VertexSE3* mainSensorPoseVertex,
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

g2o::SE3Quat 
RelPose::RelPoseFromTrajectorySE3::
estimateLeverArm(
    const std::vector<g2o::SE3Quat>& mainSensorMeasuredTrajectory,
    const std::vector<g2o::SE3Quat>& slaveSensorMeasuredTrajectory,
    const g2o::SE3Quat &initRelativeSensorPoseEstim,
    const g2o::SE3Quat& initSlaveSensorLocalCoordinateSpaceEstim // TODO: REMOVE! We should calculate this from initRelativeSensorPoseEstim 
)
{
    // Prepare the graph
    // 
    // create the optimizer to load the data and carry out the optimization
    int graphIdGenerator = 0;
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);


    // allocate the solver
    g2o::OptimizationAlgorithmLevenberg* solver =
        new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<g2o::BlockSolverX>(
            g2o::make_unique<
            g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>>()));

    optimizer.setAlgorithm(solver);

    // Prepare graph
    std::vector<g2o::VertexSE3*> mainSensorPosesVertices;
    std::vector<g2o::EdgeSE3Prior*> mainSensorPoseObservations;

    // Global vertices
    // - Lever arm - this will contain the estimated result
    g2o::VertexSE3* relativeSlaveSensorPoseVertex = new g2o::VertexSE3();
    relativeSlaveSensorPoseVertex->setEstimate(initRelativeSensorPoseEstim);
    relativeSlaveSensorPoseVertex->setId(++graphIdGenerator);

    bool addVertexRes = optimizer.addVertex(relativeSlaveSensorPoseVertex);
    if (!addVertexRes)
    {
        throw std::runtime_error("Failed to add vertex.");
    }

    // - Slave sensor local coordinate space
    g2o::VertexSE3* localSlaveCoordinateSpaceVertex = new g2o::VertexSE3();
    localSlaveCoordinateSpaceVertex->setEstimate(initSlaveSensorLocalCoordinateSpaceEstim);
    localSlaveCoordinateSpaceVertex->setId(++graphIdGenerator);

    addVertexRes = optimizer.addVertex(localSlaveCoordinateSpaceVertex);
    if (!addVertexRes)
    {
        throw std::runtime_error("Failed to add vertex.");
    }

    std::vector<::LocalSlaveSensorPoseEdge*> slaveSensorPoseObservations;

    // We need to add ParameterSE3Offset, otherwise the g2o is having troubles with prior edges. :/
    g2o::ParameterSE3Offset* globalOffsetParameter = new g2o::ParameterSE3Offset;
    globalOffsetParameter->setOffset();
    int globalOffsetParameterId = 0;
    globalOffsetParameter->setId(globalOffsetParameterId);
    optimizer.addParameter(globalOffsetParameter);

    for (std::size_t measurementIdx = 0; measurementIdx < mainSensorMeasuredTrajectory.size(); ++measurementIdx)
    {
        // Main sensor pose vertex
        const auto& mainSensorMeasuredPose = mainSensorMeasuredTrajectory[measurementIdx];

        g2o::VertexSE3* mainSensorPosesVertex = new g2o::VertexSE3();
        mainSensorPosesVertex->setEstimate(mainSensorMeasuredPose);
        mainSensorPosesVertex->setId(++graphIdGenerator);

        addVertexRes = optimizer.addVertex(mainSensorPosesVertex);
        if (!addVertexRes)
        {
            throw std::runtime_error("Failed to add vertex.");
        }
        mainSensorPosesVertices.push_back(mainSensorPosesVertex);

        // Main sensor pose prior edge
        g2o::EdgeSE3Prior* mainSensorPoseObservation = new g2o::EdgeSE3Prior();
        mainSensorPoseObservation->setVertex(0, mainSensorPosesVertex);
        mainSensorPoseObservation->setMeasurement(mainSensorMeasuredPose);
        mainSensorPoseObservation->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 10);
        mainSensorPoseObservation->setId(++graphIdGenerator);
        mainSensorPoseObservation->setParameterId(0, globalOffsetParameterId); // the first parameter must be global offset. :O No idea why.
        bool addEdgeRes = optimizer.addEdge(mainSensorPoseObservation);
        if (!addEdgeRes)
        {
            throw std::runtime_error("Failed to add edge.");
        }
        mainSensorPoseObservations.push_back(mainSensorPoseObservation);

        // slave sensor pose observations
        const auto& slaveSensorMeasuredPose = slaveSensorMeasuredTrajectory[measurementIdx];
        ::LocalSlaveSensorPoseEdge* slaveSensorPoseObservation = new ::LocalSlaveSensorPoseEdge(
            mainSensorPosesVertex, relativeSlaveSensorPoseVertex, localSlaveCoordinateSpaceVertex);
        slaveSensorPoseObservation->setMeasurement(slaveSensorMeasuredPose);
        slaveSensorPoseObservation->setInformation(Eigen::Matrix<double, 6, 6>::Identity() * 10);
        slaveSensorPoseObservation->setId(++graphIdGenerator);
        slaveSensorPoseObservation->setParameterId(0, globalOffsetParameterId); // the first parameter must be global offset. :O No idea why.
        addEdgeRes = optimizer.addEdge(slaveSensorPoseObservation);
        if (!addEdgeRes)
        {
            throw std::runtime_error("Failed to add edge.");
        }
        slaveSensorPoseObservations.push_back(slaveSensorPoseObservation);
    }

    optimizer.initializeOptimization();
    optimizer.computeActiveErrors();
    std::cout << "Initial chi2 = " << FIXED(optimizer.chi2()) << std::endl;

    optimizer.setVerbose(true);

    optimizer.optimize(20);

    return relativeSlaveSensorPoseVertex->estimateAsSE3Quat();
}
