#ifndef POSEESTIMATION_H
#define POSEESTIMATION_H

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <random>

#include <algorithm>
#include <functional>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <chrono>

#include "model_processing/common.h"
#include "collision_detection/CollisionDetection.h"
#include "robot_config/RobotConfig.h"

#include <vtkAxesActor.h>
#include <vtkTransform.h>

#include <pagmo/problem.hpp>
#include <pagmo/population.hpp>
#include <pagmo/algorithms/pso.hpp>
#include <pagmo/population.hpp>

using namespace Eigen;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using Vector6d = Matrix<double, 6, 1>;

class PoseEstimation
{
public:
    std::vector<std::pair<double,double>> qlimits = {
        {-170 * M_PI / 180, 170 * M_PI / 180},
        {-155 * M_PI / 180,  90 * M_PI / 180},
        { -85 * M_PI / 180, 140 * M_PI / 180},
        {-150 * M_PI / 180, 150 * M_PI / 180},
        {-135 * M_PI / 180, 90 * M_PI / 180},
        {-210 * M_PI / 180, 210 * M_PI / 180}
    };//二、四、六轴方向相反，后面改！！！

private:
    int pop_size_;     // 外层粒子数
    int gen_;          // 外层代数
    double inertia_;         // PSO 惯性
    double cognitive_;       // 认知系数
    double social_;          // 社会系数

    std::vector<DHParameters> dh_params_;

    struct State {
            double total_dist = std::numeric_limits<double>::max(); // 累计段间距离
            double total_manip = -1.0;                             // 累计可操作度（用于求平均）
            int prev_idx = -1;                                     // 回溯指针
        };

public:
    struct Placement {
        double x, y, z;
    };


    struct Result {
        Placement placement;          // 最优基座位姿
        double cost;                  // 最优代价
        std::vector<double> angles;   // 每段轨迹对应的角度 θ_i
    };

public:
    PoseEstimation(const RobotConfig& robotConfig, int pop_size, int gen, double inertia, double cognitive, double social);

    Result optimize(const std::vector<std::vector<Eigen::Matrix4d>> &trajectories, const std::vector<Eigen::Matrix4d> &viewPoint) const;

    Result optimizeBaseOnly(const std::vector<std::vector<Eigen::Matrix4d>> &trajectories, const std::vector<Eigen::Matrix4d> &viewPoint, const std::vector<double>& fixed_angles) const;

    double objective(std::vector<double> angle,
                          const Placement &placement,
                          const std::vector<std::vector<Eigen::Matrix4d>> &trajectories, const std::vector<Eigen::Matrix4d> &viewPoint) const;


private:
    Matrix4d forwardKinematics(const Vector6d& angles) const;

public:
    std::vector<Vector6d> inverseKinematics(const Matrix4d& target_pose) const;

private:

    double computeYoshikawaManipulability(const Vector6d& angles) const;

    Matrix4d dhTransform(double a, double alpha, double d, double theta) const;

    std::vector<Vector3d> solveFirstThreeJoints(const Vector3d& p_03) const;

    std::vector<Vector3d> solveLastThreeJoints(const Matrix4d& T_36) const;

    Matrix4d computeT03(const Vector3d& theta123) const;

    double normalizeAngle(double angle) const;

    double atan2_shifted(double y, double x) const;

public:
    double angleDistance(const Vector6d& q1, const Vector6d& q2) const;

    double angleWeightDistance(const Vector6d& q1, const Vector6d& q2) const;

    bool isValidSolution(const Vector6d& q, std::vector<std::pair<double,double>> qlimits) const;

    double calculateManipulability(const std::vector<Vector6d>& path) const;

private:
    bool inverseKinematics_wrist_partitioned(const Placement &placement, const std::vector<Matrix4d> &targetPose, double redundantAngle, std::vector<Vector6d> &q) const;

    bool inverseKinematics_wrist_partitioned2(const Placement &placement, const std::vector<std::vector<Eigen::Matrix4d>> &trajectories, std::vector<double> redundantAngle, std::vector<std::vector<Vector6d>> &qs) const;

    bool isContinuous(const Vector6d& prev_q, const Vector6d& curr_q) const;

    bool findBestNextSolution(const Vector6d& prev_q, const std::vector<Vector6d>& candidates, Vector6d& out_best_q) const;

    std::vector<std::vector<Vector6d>> selectBestPaths(const std::vector<std::vector<std::vector<Vector6d>>>& all_segments_paths) const;

public:
    std::vector<std::vector<Vector6d>> cartesianPlanning(const std::vector<std::vector<Eigen::Matrix4d>> &trajectories) const;

    bool isStateInCollision(const std::vector<double>& q) const;

    bool isStateInCollision2(const std::vector<double>& q) const;

    std::vector<std::vector<Vector6d>> cartesianPlanning2(const std::vector<std::vector<Eigen::Matrix4d>> &trajectories) const;

public:
    std::vector<Vector6d> selectBestTrajectory(std::vector<Matrix4d> targetPose) const;
};

#endif // POSE_ESTIMATION_H
