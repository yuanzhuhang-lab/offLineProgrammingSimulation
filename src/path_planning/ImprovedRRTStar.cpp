#include "ImprovedRRTStar.h"

ImprovedRRTStar::ImprovedRRTStar()
{

}

ImprovedRRTStar::~ImprovedRRTStar()
{
}

bool ImprovedRRTStar::plan(const std::vector<double>& q_start, const std::vector<double>& q_goal, std::vector<std::vector<double>>& finalPath)
{
        auto space = std::make_shared<ompl::base::RealVectorStateSpace>(6);

        ompl::base::RealVectorBounds jointBounds(6);
        for (int i = 0; i < 6; ++i) {
            jointBounds.setLow(i, qlimits[i].first);
            jointBounds.setHigh(i, qlimits[i].second);
        }
        space->setBounds(jointBounds);

        ompl::geometric::SimpleSetup ss(space);

        ss.setStateValidityChecker(
            [this, &q_start](const ompl::base::State* state)
            {
                const auto* s6 =
                    state->as<ompl::base::RealVectorStateSpace::StateType>();

                std::vector<double> q(9);

                for (int i = 0; i < 6; ++i)
                    q[i] = s6->values[i];

                // 后 3 轴固定
                q[6] = q_start[6];
                q[7] = q_start[7];
                q[8] = q_start[8];

                return !isStateInCollision(q);
            });

        ss.getSpaceInformation()->setMotionValidator(
            std::make_shared<ompl::base::DiscreteMotionValidator>(
                ss.getSpaceInformation()));

        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(space);
        ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(space);

        for (int i = 0; i < 6; ++i) {
            start->values[i] = q_start[i];
            goal->values[i]  = q_goal[i];
        }

        ss.setStartAndGoalStates(start, goal);

        auto goal_ptr =
            std::make_shared<ompl::base::GoalState>(ss.getSpaceInformation());
        goal_ptr->setState(goal);
        goal_ptr->setThreshold(0.1);
        ss.setGoal(goal_ptr);

        // ========= 5. 规划器 =========
        auto planner =
            std::make_shared<ompl::geometric::RRTConnect>(
                ss.getSpaceInformation());
        planner->setRange(0.3);
        ss.setPlanner(planner);

        ss.getSpaceInformation()->setStateValidityCheckingResolution(0.002);

        std::cout << "正在进行 6 轴规划..." << std::endl;

        ompl::base::PlannerStatus solved = ss.solve(25.0);

        if (!solved) {
            std::cout << "未能找到可行路径。" << std::endl;
            return false;
        }

        // ========= 6. 输出路径 =========
        std::cout << "成功找到路径！" << std::endl;

        ss.simplifySolution();
        ompl::geometric::PathGeometric& path = ss.getSolutionPath();

        finalPath.clear();
        for (size_t i = 0; i < path.getStateCount(); ++i) {

            const auto* s6 =
                path.getState(i)
                    ->as<ompl::base::RealVectorStateSpace::StateType>();

            std::vector<double> q(9);

            for (int j = 0; j < 6; ++j)
                q[j] = s6->values[j];

            q[6] = q_start[6];
            q[7] = q_start[7];
            q[8] = q_start[8];

            finalPath.push_back(q);
        }

        return true;
}

bool ImprovedRRTStar::isStateInCollision(const std::vector<double>& q)
{
    // 简化的基座变换矩阵
    Eigen::Matrix4d baseTransformMatrix;
    baseTransformMatrix << 1.0, 0.0, 0.0, q[6],
                          0.0, -1.0, 0.0, q[7],
                          0.0, 0.0, -1.0, q[8],
                          0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d cumulativeTransform = baseTransformMatrix;
    std::vector<Eigen::Matrix4d> linkTransforms(6);

    // 简化的DH变换计算
    for (size_t i = 0; i < dh_params.size(); ++i) {
        const auto& dh = dh_params[i];
        double theta = dh.theta + q[i];

        double ct = std::cos(theta), st = std::sin(theta);
        double ca = std::cos(dh.alpha), sa = std::sin(dh.alpha);

        Eigen::Matrix4d T;
        T << ct, -st * ca,  st * sa, dh.a * ct,
             st,  ct * ca, -ct * sa, dh.a * st,
              0,       sa,       ca,      dh.d,
              0,        0,        0,        1;

        cumulativeTransform = cumulativeTransform * T;
        linkTransforms[i] = cumulativeTransform;
    }

    // 工具变换矩阵
    Eigen::Matrix4d tool2endmatrix;
    tool2endmatrix << 0.0, -0.86083, 0.50889, 72.5,
                     1.0, 0.0, 0.0, 0.0,
                     0.0, 0.50889, 0.86083, 516.57368,
                     0.0, 0.0, 0.0, 1.0;

    Eigen::Matrix4d toolTransformMatrix = linkTransforms[5] * tool2endmatrix;


    // 简化的关节变换计算
    std::vector<Eigen::Matrix4d> joint_transforms;
    joint_transforms.reserve(7);

    // 基座变换
    joint_transforms.push_back(baseTransformMatrix);

    // 各连杆变换
    for (int i = 0; i < 5; ++i) {
        joint_transforms.push_back(linkTransforms[i]);
    }

    // 工具变换
    joint_transforms.push_back(toolTransformMatrix);

    CollisionDetection& collisionDetector = CollisionDetection::getInstance();
    collisionDetector.setEnvironmentMode(EnvDetectMask::GROUND_ONLY);
    collisionDetector.updateRobotPose(joint_transforms);

    // 执行碰撞检测
    if(collisionDetector.checkCollision()) {
        return true;
    }
    return false;
}




