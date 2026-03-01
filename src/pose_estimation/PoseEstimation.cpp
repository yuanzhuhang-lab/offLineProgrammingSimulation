#include "PoseEstimation.h"

PoseEstimation::PoseEstimation(const RobotConfig& robotConfig, int pop_size, int gen, double inertia, double cognitive, double social):dh_params_(robotConfig.getDHParameters()), pop_size_(pop_size), gen_(gen), inertia_(inertia), cognitive_(cognitive), social_(social)
{
}

struct SingleLayerProblem {
    const PoseEstimation *pe;
    const std::vector<std::vector<Eigen::Matrix4d>> *traj_group;
    const std::vector<Eigen::Matrix4d> *viewPoint;

    SingleLayerProblem() = default;
    SingleLayerProblem(const PoseEstimation *pe_,
                       const std::vector<std::vector<Eigen::Matrix4d>> *tg_,
                       const std::vector<Eigen::Matrix4d> *viewPoint_ = nullptr)
        : pe(pe_), traj_group(tg_), viewPoint(viewPoint_) {}

    pagmo::vector_double fitness(const pagmo::vector_double &x) const {
        size_t N = traj_group->size();
        if(x.size() != 3 + N) {
            throw std::runtime_error("dimension mismatch in SingleLayerProblem");
        }

        PoseEstimation::Placement p{x[0], x[1], x[2]};

        std::vector<double> thetas;
        double worst_cost = -1e9;
        for(size_t i = 0; i < N; ++i) {
            double theta = x[3 + i];
            thetas.push_back(theta);
        }

        double c = pe->objective(thetas, p, *traj_group, *viewPoint);
        worst_cost = std::max(worst_cost, c);

        return {worst_cost};
    }

    // 变量边界
    std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const {
        size_t N = traj_group->size();

        pagmo::vector_double lb(3 + N);
        pagmo::vector_double ub(3 + N);

        // baseXYZ bounds
        lb[0] = -5000;  ub[0] = 5000;
        lb[1] = -1000;  ub[1] = 2000;
        lb[2] =  1400;  ub[2] = 2500;

        // θ bounds
        for(size_t i = 0; i < N; ++i) {
            lb[3 + i] = -M_PI;
            ub[3 + i] =  M_PI;
        }

        return {lb, ub};
    }
};

PoseEstimation::Result PoseEstimation::optimize(const std::vector<std::vector<Eigen::Matrix4d>> &trajectories, const std::vector<Eigen::Matrix4d> &viewPoint) const
{
    size_t N = trajectories.size();
    size_t DIM = 3 + N;

    pagmo::problem prob{ SingleLayerProblem(this, &trajectories, &viewPoint) };
    pagmo::algorithm algo{ pagmo::pso(gen_, inertia_, cognitive_, social_) };
    pagmo::population pop(prob, pop_size_);

    pop = algo.evolve(pop);

    auto best = pop.champion_x();
    if (best.size() != DIM) {
        throw std::runtime_error("❌ optimizeSingleLayer(): dimension mismatch: expected "
                                 + std::to_string(DIM) + " got "
                                 + std::to_string(best.size()));
    }

    double best_cost = pop.champion_f()[0];

    Placement best_p{best[0], best[1], best[2]};

    std::vector<double> best_angles(best.begin() + 3, best.end());

    Result result;
    result.placement = best_p;
    result.cost = best_cost;
    result.angles = best_angles;

    return result;
}

struct BaseOnlyProblem {
    const PoseEstimation *pe;
    const std::vector<std::vector<Eigen::Matrix4d>> *traj_group;
    const std::vector<Eigen::Matrix4d> *viewPoint;
    const std::vector<double> *fixed_angles;

    BaseOnlyProblem() = default;
    BaseOnlyProblem(const PoseEstimation *pe_,
                    const std::vector<std::vector<Eigen::Matrix4d>> *tg_,
                    const std::vector<Eigen::Matrix4d> *viewPoint_,
                    const std::vector<double> *fixed_angles_)
        : pe(pe_), traj_group(tg_), viewPoint(viewPoint_), fixed_angles(fixed_angles_) {}

    pagmo::vector_double fitness(const pagmo::vector_double &x) const {
        if(x.size() != 3) {
            throw std::runtime_error("dimension mismatch in BaseOnlyProblem: expected 3 got " + std::to_string(x.size()));
        }

        PoseEstimation::Placement p{x[0], x[1], x[2]};

        // 使用固定的角度
        std::vector<double> thetas = *fixed_angles;

        double c = pe->objective(thetas, p, *traj_group, *viewPoint);
        return {c};
    }

    // 变量边界（只优化基座位置）
    std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const {
        pagmo::vector_double lb(3);
        pagmo::vector_double ub(3);

        // 基座XYZ边界（与原始优化相同）
        lb[0] = -5000;  ub[0] = 5000;
        lb[1] = -1000;  ub[1] = 2000;
        lb[2] =  1400;  ub[2] = 2500;

        return {lb, ub};
    }
};

PoseEstimation::Result PoseEstimation::optimizeBaseOnly(const std::vector<std::vector<Matrix4d> > &trajectories, const std::vector<Matrix4d> &viewPoint, const std::vector<double> &fixed_angles) const
{
    size_t N = trajectories.size();

    // 验证固定角度数量是否匹配轨迹数量
    if (fixed_angles.size() != N) {
        throw std::runtime_error("Fixed angles count (" + std::to_string(fixed_angles.size()) +
                                ") does not match trajectories count (" + std::to_string(N) + ")");
    }

    pagmo::problem prob{ BaseOnlyProblem(this, &trajectories, &viewPoint, &fixed_angles) };
    pagmo::algorithm algo{ pagmo::pso(gen_, inertia_, cognitive_, social_) };
    pagmo::population pop(prob, pop_size_);

    pop = algo.evolve(pop);

    auto best = pop.champion_x();
    if (best.size() != 3) {
        throw std::runtime_error("❌ optimizeBaseOnly(): dimension mismatch: expected 3 got " + std::to_string(best.size()));
    }

    double best_cost = pop.champion_f()[0];

    Placement best_p{best[0], best[1], best[2]};

    Result result;
    result.placement = best_p;
    result.cost = best_cost;
    result.angles = fixed_angles;  // 使用固定的角度

    return result;
}


double PoseEstimation::objective(
    std::vector<double> angle,
    const Placement &placement,
    const std::vector<std::vector<Eigen::Matrix4d>> &trajectories, const std::vector<Eigen::Matrix4d> &viewPoint) const
{
    double rate = 0.0;
    for(size_t i = 0; i < trajectories.size(); i++){
        for (const auto& pose : trajectories[i]) {
            double target_x = pose(0, 3);
            double target_y = pose(1, 3);
            double target_z = pose(2, 3);

            double dx = target_x - placement.x;
            double dy = target_y - placement.y;
            double dz = target_z - placement.z;
            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            double denominator = 2000.0;
            double point_con1 = distance / denominator;

            rate = std::max(rate, point_con1);
        }
    }

    if (rate >= 1.0) {
        return 100 * rate;
    }


    std::vector<std::vector<Vector6d>> qs;
    bool ok = inverseKinematics_wrist_partitioned2(placement, trajectories, angle, qs);
    if (!ok) {
        return 80;
    }

    for (const auto& q : qs) {
    if (!q.empty()) {
        // 检查第一个点（使用isStateInCollision2）
        std::vector<double> q_vec_first;
        q_vec_first.reserve(9);

        for (int i = 0; i < 6; ++i) {
            q_vec_first.push_back(q.front()[i]);
        }

        q_vec_first.push_back(placement.x);
        q_vec_first.push_back(placement.y);
        q_vec_first.push_back(placement.z);

        if (isStateInCollision2(q_vec_first)) {
            return 50;
        }

        // 检查最后一个点（使用isStateInCollision2）
        if (q.size() > 1) {
            std::vector<double> q_vec_last;
            q_vec_last.reserve(9);

            for (int i = 0; i < 6; ++i) {
                q_vec_last.push_back(q.back()[i]);
            }

            q_vec_last.push_back(placement.x);
            q_vec_last.push_back(placement.y);
            q_vec_last.push_back(placement.z);

            if (isStateInCollision2(q_vec_last)) {
                return 50;
            }
        }
    }
    }

    double con1 = 0.0;
    double theta_s_star = 0.4;
    double theta_h_star = 0.2;
    double theta_h_star2 = 20;

    // 从DH参数表获取实际参数值
    double a1 = dh_params_[0].a;
    double a2 = dh_params_[1].a;
    double a3 = dh_params_[2].a;
    double d4 = dh_params_[3].d;

    for(size_t i = 0; i < qs.size(); i++) {
        for (const auto& q_point : qs[i]) {
            double theta2 = q_point[1];  // 关节2角度
            double theta3 = q_point[2];  // 关节3角度
            double theta5 = q_point[4];  // 关节5角度

            double sin_theta5 = std::sin(theta5);
            double term1 = (std::fabs(sin_theta5) < theta_s_star) ?
                           100 :
                           (theta_s_star / std::fabs(sin_theta5));

            double sin_shoulder1 = std::sin(theta3) - 1.26791189;
            double term2 = (std::fabs(sin_shoulder1) < theta_h_star) ?
                           100 :
                           (theta_h_star / std::fabs(sin_shoulder1));

            double sin_shoulder2 = std::abs(-a1 + a2*std::sin(theta2) + a3*std::sin(theta2+theta3) - d4*std::cos(theta2 + theta3));
            double term3 = (std::fabs(sin_shoulder2) < theta_h_star2) ?
                           100 :
                           (theta_h_star2 / std::fabs(sin_shoulder2));

            // 计算当前点的con6值
            double point_con2 = std::max({term1, term2, term3});

            // 更新整个轨迹的最大con6值
            con1 = std::max(con1, point_con2);
        }
    }
    if (con1 == 100) {
        return 50;
    }

    double con2 = 0.0;
    for(size_t j = 0; j < qs.size(); j++) {
    for (const auto& point : qs[j]) {
        double point_con3 = 0.0;
        for (size_t i = 0; i < 6; ++i) {
            double t = point[i]; // 当前关节角度
            double tmin = qlimits[i].first; // 关节下限
            double tmax = qlimits[i].second; // 关节上限

            double denom = (tmax - tmin) * 0.5 + 1e-12;
            double val = std::abs((t - 0.5*(tmin + tmax)) / denom); // 0 表示在中间，1 表示到边界

            point_con3 = std::max(point_con3, val);
        }

        con2 = std::max(con2, point_con3);
    }}

//    double min_manipulability = std::numeric_limits<double>::max();
//    for(size_t z = 0; z < qs.size(); z++) {
//        for (const auto& q_point : qs[z]) {
//            double manipulability = computeYoshikawaManipulability(q_point);
//            min_manipulability = std::min(min_manipulability, manipulability);
//        }
//    }
//    double max_reference = 4.67286e+08;
//    double manipulability_ratio = (max_reference > 1e-6) ? (min_manipulability / max_reference) : 0.0;
//    double con3 = 1.0 - std::min(1.0, manipulability_ratio);
    double con3 = 0;

    if (!viewPoint.empty()) {
        Matrix4d end2camMatrix;
        end2camMatrix <<
            1.0, 0.0, 0.0, -52.4276,
            0.0, 0.866, -0.5, -197.74489,
            0.0, 0.5,  0.866, 260.57368,
            0.0, 0.0,  0.0,   1.0;
        for (const auto& point : viewPoint) {
            Matrix4d baseTransformMatrix;
            baseTransformMatrix << 1.0, 0.0, 0.0, -placement.x,
                                   0.0, -1.0, 0.0, placement.y,
                                   0.0, 0.0, -1.0, placement.z,
                                   0.0, 0.0, 0.0, 1.0;

            Matrix4d transformedPoint = (baseTransformMatrix * point) * end2camMatrix;

            std::vector<Vector6d> solves = inverseKinematics(transformedPoint);
            if (solves.empty()) {
                return {50};
            } else {
                bool has_valid_solution = false;
                for (const auto& sol : solves) {
                    std::vector<double> q_vec;
                    q_vec.reserve(9);

                    for (int i = 0; i < 6; ++i) {
                        q_vec.push_back(sol[i]);
                    }

                    q_vec.push_back(placement.x);
                    q_vec.push_back(placement.y);
                    q_vec.push_back(placement.z);

                    // 检查同时满足碰撞检测和关节限制
                    if (!isStateInCollision2(q_vec) && isValidSolution(sol, qlimits)) {
                        has_valid_solution = true;
                        break;  // 找到一个有效解即可
                    }
                }

                if (!has_valid_solution) {
                    return {50};  // 没有找到同时满足两个条件的解
                }
            }
        }
    }

    const double k1 = 1.0, k2 = 1.0, k3 = 1.0;

    double v1 = k1 * con1;
    double v2 = k2 * con2;
    double v3 = k3 * con3;

    double Fr = 0.0;

    Fr = std::max({v1, v2, v3});

    return Fr;
}

Matrix4d PoseEstimation::forwardKinematics(const Vector6d &angles) const
{
    Matrix4d T = Matrix4d::Identity();

    for (int i = 0; i < 6; ++i) {
        T *=  dhTransform(
            dh_params_[i].a,
            dh_params_[i].alpha,
            dh_params_[i].d,
            dh_params_[i].theta + angles[i]
        );
    }

    Eigen::Matrix4d tool2endmatrix;
    tool2endmatrix << 0.0, -0.86083, 0.50889, 72.5,
              1.0, 0.0, 0.0, 0.0,
              0.0, 0.50889, 0.86083, 516.57368,
              0.0, 0.0, 0.0, 1.0;

    return T * tool2endmatrix;
}

std::vector<Vector6d> PoseEstimation::inverseKinematics(const Matrix4d &target_pose) const
{
    std::vector<Vector6d> solutions;

    Eigen::Matrix4d matrix;
    matrix << 0.0, -0.86083, 0.50889, 72.5,
              1.0, 0.0, 0.0, 0.0,
              0.0, 0.50889, 0.86083, 516.57368,
              0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4d matrix2 = matrix.inverse();

    Matrix4d targetPose = target_pose * matrix2;

    // 提取目标位置和z轴方向
    Vector3d p_06 = targetPose.block<3,1>(0,3);
    Vector3d z_06 = targetPose.block<3,1>(0,2);

    // 计算腕部中心位置
    double d6 = dh_params_[5].d;

    Vector3d p_03 = p_06 - d6 * z_06;

    // 求解前三个关节
    auto theta123_solutions = solveFirstThreeJoints(p_03);

    for (const auto& theta123 : theta123_solutions) {
        // 计算T03变换矩阵
        Matrix4d T_03 = computeT03(theta123);

        // 计算T36变换矩阵
        Matrix4d T_36 = T_03.inverse() * targetPose;

        // 求解后三个关节
        auto theta456_solutions = solveLastThreeJoints(T_36);

        for (const auto& theta456 : theta456_solutions) {
            Vector6d solution;
            solution << theta123, theta456;
            solutions.push_back(solution);
        }
    }

    return solutions;
}

double PoseEstimation::computeYoshikawaManipulability(const Vector6d &angles) const
{
    size_t n = angles.size();
    std::vector<Matrix4d> Ts(n+1);
    Ts[0] = Matrix4d::Identity();
    for (size_t i = 0; i < n; ++i) {
        Matrix4d A = dhTransform(dh_params_[i].a, dh_params_[i].alpha, dh_params_[i].d, dh_params_[i].theta + angles(i));
        Ts[i+1] = Ts[i] * A;
    }

    Vector3d o_n = Ts.back().block<3,1>(0,3);
    Matrix<double,6,6> J = Matrix<double,6,6>::Zero();
    for (size_t i = 0; i < n; ++i) {
        Eigen::Vector3d o_i = Ts[i].block<3,1>(0,3);
        Eigen::Vector3d z_i = Ts[i].block<3,1>(0,2);
        Eigen::Vector3d Jv = z_i.cross(o_n - o_i);
        Eigen::Vector3d Jw = z_i;
        J.block<3,1>(0,i) = Jv;
        J.block<3,1>(3,i) = Jw;
    }

    Eigen::JacobiSVD<Matrix<double,6,6>> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd sigma = svd.singularValues();

    double logsum = 0.0;
    for (int i = 0; i < sigma.size(); ++i) {
        if (sigma(i) <= 0) return 0.0;
        logsum += std::log(sigma(i));
    }

    double w = std::exp(logsum);

    return w;
}

Matrix4d PoseEstimation::dhTransform(double a, double alpha, double d, double theta) const
{
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    Matrix4d T;
    T << ct, -st * ca,  st * sa, a * ct,
         st,  ct * ca, -ct * sa, a * st,
          0,       sa,       ca,      d,
          0,        0,        0,      1;

    return T;
}

std::vector<Vector3d> PoseEstimation::solveFirstThreeJoints(const Vector3d &p_03) const
{
    std::vector<Vector3d> solutions;

    double a1 = dh_params_[0].a;
    double a2 = dh_params_[1].a;
    double a3 = dh_params_[2].a;
    double d1 = dh_params_[0].d;
    double d4 = dh_params_[3].d;

    // 求解theta1
    double r = std::sqrt(p_03.x() * p_03.x() + p_03.y() * p_03.y());
    double phi1 = atan2_shifted(p_03.y(), p_03.x());

    std::vector<double> theta1_candidates = {phi1, phi1 + M_PI};

    for (double theta1 : theta1_candidates) {
        Matrix4d T = dhTransform(dh_params_[0].a, dh_params_[0].alpha, dh_params_[0].d, dh_params_[0].theta + theta1);
        Vector4d p_03_hom;
        p_03_hom << p_03, 1.0;
        Matrix4d T_inverse = T.inverse();
        Vector4d p_13_hom = T_inverse * p_03_hom;
        Vector3d p_13 = p_13_hom.head<3>();
        double r_ = std::sqrt(p_13.x() * p_13.x() + p_13.z() * p_13.z());

        double A = std::abs(p_13.y());
        double B = r_;
        double C = (-(A * A + B * B - a2 * a2 - (d4 * d4 + a3 * a3))) / (2 * a2 * std::sqrt(d4 * d4 + a3 * a3));
        double D = (a3 * a3 + d4 * d4 + a2 * a2 - A * A - B * B) / (2 * a2 * std::sqrt(a3 * a3 + d4 * d4));

        if (std::abs(C) > 1.0) continue;
        if (std::abs(D) > 1.0) continue;
        double a = std::atan(d4/a3);

        double theta3_1 = -(M_PI - std::acos(C) - a);
        double theta3_2 = M_PI - std::acos(D) + a;

        for (double theta3 : {theta3_1, theta3_2}) {

            double theta2;
            double E = (a2 * a2 + p_13.y()*p_13.y() + p_13.x()*p_13.x() - a3 * a3 - d4 * d4) / (2 * a2 * std::sqrt(p_13.y()*p_13.y() + p_13.x()*p_13.x()));
            if (std::abs(E) > 1.0) continue;
            double alpha = std::acos(E);

            double beta = atan2_shifted(p_13.y(), p_13.x());
            if (theta3 >= a) {
                theta2 = beta - alpha;
            } else if (theta3 < a) {
                theta2 = beta + alpha;
            }

            Vector3d solution(
                normalizeAngle(theta1),
                normalizeAngle(theta2),
                normalizeAngle(theta3)
            );
            solutions.push_back(solution);
        }
    }

    return solutions;
}

std::vector<Vector3d> PoseEstimation::solveLastThreeJoints(const Matrix4d &T_36) const
{
    std::vector<Vector3d> solutions;

    Matrix3d R_36 = T_36.block<3,3>(0,0);

    // ZYZ欧拉角求解 (适用于大多数六轴机器人)
    double theta5 = std::atan2(
        std::sqrt(R_36(0,2) * R_36(0,2) + R_36(1,2) * R_36(1,2)),
        R_36(2,2)
    );

    double theta5_alt = -theta5;

    for (double theta5_val : {theta5, theta5_alt}) {
        if (std::abs(theta5_val) < 1e-10) {
            // 奇异位置处理
            double theta4 = 0.0;
            double theta6 = std::atan2(-R_36(1,0), R_36(0,0)) - theta4;
            solutions.push_back(Vector3d(
                normalizeAngle(theta4),
                normalizeAngle(theta5_val),
                normalizeAngle(theta6)
            ));
        } else {
            double theta4 = std::atan2(R_36(1,2) / std::sin(theta5_val),
                                      R_36(0,2) / std::sin(theta5_val));
            double theta6 = std::atan2(R_36(2,1) / std::sin(theta5_val),
                                      -R_36(2,0) / std::sin(theta5_val)) - M_PI;
            solutions.push_back(Vector3d(
                normalizeAngle(theta4),
                normalizeAngle(theta5_val),
                normalizeAngle(theta6)
            ));
        }
    }

    return solutions;
}

Matrix4d PoseEstimation::computeT03(const Vector3d &theta123) const
{
    Matrix4d T = Matrix4d::Identity();

    for (int i = 0; i < 3; ++i) {
        T *= dhTransform(
            dh_params_[i].a,
            dh_params_[i].alpha,
            dh_params_[i].d,
            dh_params_[i].theta + theta123[i]
        );
    }

    return T;
}

double PoseEstimation::normalizeAngle(double angle) const
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double PoseEstimation::atan2_shifted(double y, double x) const
{
    if (x == 0.0 && y == 0.0) return 0.0;

    double a = std::atan2(y, x) - M_PI / 2.0; // 左移 pi/2

    // 归一化到 (-pi, pi]
    while (a <= -M_PI) a += 2.0 * M_PI;
    while (a >  M_PI) a -= 2.0 * M_PI;

    return a;
}

double PoseEstimation::angleDistance(const Vector6d &q1, const Vector6d &q2) const
{
    double weighted_dist = 0.0;
    double max_possible_dist = 0.0; // 用于归一化的最大可能距离

    for (int i = 0; i < 6; ++i) {
        double diff = std::abs(q1[i] - q2[i]);
        double range = qlimits[i].second - qlimits[i].first;

        // 计算加权距离
        weighted_dist += diff;

        // 计算该关节的最大可能距离（用于归一化）
        max_possible_dist += range;
    }

    if (weighted_dist < 1e-5) {
        return 1e-5;  // 设置一个合理的最小值
    }

    // 归一化处理：将距离映射到[0,1]范围
    double normalized_dist = (max_possible_dist > 1e-5) ? (weighted_dist / max_possible_dist) : 0.0;

    return std::max(normalized_dist, 1e-5);
}

double PoseEstimation::angleWeightDistance(const Vector6d &q1, const Vector6d &q2) const
{
    Vector6d weights;
    weights << 2.0, 1.0, 1.0, 0.5, 0.5, 0.3; // 第2关节权重为2，第4、5关节权重为0.5

    double weighted_dist = 0.0;
    double max_possible_dist = 0.0; // 用于归一化的最大可能距离

    for (int i = 0; i < 6; ++i) {
        double diff = std::abs(q1[i] - q2[i]);
        double range = qlimits[i].second - qlimits[i].first;

        // 应用权重：权重越大，该关节的角度变化对总距离的贡献越大
        weighted_dist += weights[i] * diff;

        // 计算该关节的最大可能距离（用于归一化）
        max_possible_dist += weights[i] * range;
    }

    if (weighted_dist < 1e-5) {
        return 1e-5;  // 设置一个合理的最小值
    }

    // 归一化处理：将距离映射到[0,1]范围
    double normalized_dist = (max_possible_dist > 1e-5) ? (weighted_dist / max_possible_dist) : 0.0;

    return std::max(normalized_dist, 1e-5);
}

bool PoseEstimation::isValidSolution(const Vector6d &q, std::vector<std::pair<double, double> > qlimits) const
{
    for (int i = 0; i < 6; ++i) {
        if (q[i] < qlimits[i].first || q[i] > qlimits[i].second) {
            return false;
        }
    }
    return true;
}

bool PoseEstimation::inverseKinematics_wrist_partitioned(const Placement &placement, const std::vector<Matrix4d> &targetPose, double redundantAngle, std::vector<Vector6d> &q) const
{
    std::vector<Matrix4d> targetPoseNew;

    double c = std::cos(redundantAngle);
    double s = std::sin(redundantAngle);

    // 构造局部 Z 轴旋转矩阵
    Matrix4d Rz = Matrix4d::Identity();
    Rz(0,0) =  c;  Rz(0,1) = -s;
    Rz(1,0) =  s;  Rz(1,1) =  c;

    Matrix4d baseTransformMatrix;
    baseTransformMatrix << 1.0, 0.0, 0.0, -placement.x,
                           0.0, -1.0, 0.0, placement.y,
                           0.0, 0.0, -1.0, placement.z,
                           0.0, 0.0, 0.0, 1.0;

    for (const auto& pose : targetPose) {
        Matrix4d transformedPose = baseTransformMatrix * (pose * Rz);
        targetPoseNew.push_back(transformedPose);
    }

    if (!targetPoseNew.empty()) {
        q = selectBestTrajectory(targetPoseNew);
        if (q.empty()) {
            return false;
        }
        return true;
    }
    return false;
}

bool PoseEstimation::inverseKinematics_wrist_partitioned2(const PoseEstimation::Placement &placement, const std::vector<std::vector<Matrix4d> > &trajectories, std::vector<double> redundantAngle, std::vector<std::vector<Vector6d> > &qs) const
{
    std::vector<std::vector<Matrix4d>> targetPoseNew;
    for(size_t i = 0; i < trajectories.size(); i++) {
        double c = std::cos(redundantAngle[i]);
        double s = std::sin(redundantAngle[i]);

        // 构造局部 Z 轴旋转矩阵
        Matrix4d Rz = Matrix4d::Identity();
        Rz(0,0) =  c;  Rz(0,1) = -s;
        Rz(1,0) =  s;  Rz(1,1) =  c;

        Matrix4d baseTransformMatrix;
        baseTransformMatrix << 1.0, 0.0, 0.0, -placement.x,
                               0.0, -1.0, 0.0, placement.y,
                               0.0, 0.0, -1.0, placement.z,
                               0.0, 0.0, 0.0, 1.0;
        std::vector<Matrix4d> targetPoseTem;
        for (const auto& pose : trajectories[i]) {
            Matrix4d transformedPose = baseTransformMatrix * (pose * Rz);
            targetPoseTem.push_back(transformedPose);
        }
        targetPoseNew.push_back(targetPoseTem);
    }


    if (!targetPoseNew.empty()) {
        qs = cartesianPlanning2(targetPoseNew);
        if (qs.empty()) {
            return false;
        }
        return true;
    }
    return false;
}

bool PoseEstimation::isContinuous(const Vector6d &prev_q, const Vector6d &curr_q) const
{
    for (int j = 0; j < 6; ++j) {
        double angle_diff = std::abs(prev_q[j] - curr_q[j]);

        double joint_min = qlimits[j].first;
        double joint_max = qlimits[j].second;
        double joint_range = joint_max - joint_min;

        double normalized_diff = angle_diff / joint_range;

        if (normalized_diff > 0.08) {
            return false;
        }
    }
    return true;
}

bool PoseEstimation::findBestNextSolution(const Vector6d &prev_q, const std::vector<Vector6d> &candidates, Vector6d &out_best_q) const
{
    if (candidates.empty()) return false;

    double min_dist = std::numeric_limits<double>::max();
    bool found = false;

    for (const auto& q_curr : candidates) {
        // 检查限位
        if (!isValidSolution(q_curr, qlimits)) continue;

        // 计算距离
        double dist = angleDistance(prev_q, q_curr);

        // 检查连续性
        if (dist < min_dist && isContinuous(prev_q, q_curr)) {
            min_dist = dist;
            out_best_q = q_curr;
            found = true;
        }
    }

    return found;
}

std::vector<std::vector<Vector6d> > PoseEstimation::selectBestPaths(const std::vector<std::vector<std::vector<Vector6d> > > &all_segments_paths) const
{
    if (all_segments_paths.empty()) return {};

    size_t n_segs = all_segments_paths.size();
    // dp_table[段索引][该段解分支索引]
    std::vector<std::vector<State>> dp_table(n_segs);

    // ==========================================
    // 1. 初始化第一段轨迹 (起点状态)
    // ==========================================
    dp_table[0].resize(all_segments_paths[0].size());
    for (size_t j = 0; j < all_segments_paths[0].size(); ++j) {
        dp_table[0][j].total_dist = 0.0; // 起始段无跳转代价
        dp_table[0][j].total_manip = calculateManipulability(all_segments_paths[0][j]);
        dp_table[0][j].prev_idx = -1;
    }

    // ==========================================
    // 2. 动态规划过程 (多阶段决策优化)
    // ==========================================
    for (size_t i = 1; i < n_segs; ++i) {
        dp_table[i].resize(all_segments_paths[i].size());

        // 遍历当前段 i 的每一个候选分支 j
        for (size_t j = 0; j < all_segments_paths[i].size(); ++j) {
            const auto& curr_path = all_segments_paths[i][j];
            double curr_m = calculateManipulability(curr_path);

            // 临时存储所有从上一段 k 到当前 j 的可能跳转方案
            struct CandidateLink {
                int prev_k;
                double t_dist;
                double t_manip;
            };
            std::vector<CandidateLink> pool;
            double min_dist_to_j = std::numeric_limits<double>::max();

            // 第一遍扫描：计算所有上一段 k 跳转过来的代价，并找到最小距离
            for (size_t k = 0; k < all_segments_paths[i-1].size(); ++k) {
                // 计算上一段末尾到当前段开头的跳转距离
                double step_d = angleWeightDistance(all_segments_paths[i-1][k].back(), curr_path.front());
                double accum_dist = dp_table[i-1][k].total_dist + step_d;
                double accum_manip = dp_table[i-1][k].total_manip + curr_m;

                pool.push_back({static_cast<int>(k), accum_dist, accum_manip});

                if (accum_dist < min_dist_to_j) {
                    min_dist_to_j = accum_dist;
                }
            }

            // 第二遍扫描：建立 10% 冗余候选池，并根据可操作度择优
            double threshold = min_dist_to_j * 1.1; // 定义 10% 的冗余带宽
            double best_m = -1.0;
            int best_k = -1;
            double best_d = std::numeric_limits<double>::max();

            for (const auto& cand : pool) {
                // 仅在距离差异不大于最短距离 10% 的范围内竞争
                if (cand.t_dist <= threshold) {
                    if (cand.t_manip > best_m) {
                        best_m = cand.t_manip;
                        best_k = cand.prev_k;
                        best_d = cand.t_dist;
                    }
                }
            }

            // 更新 DP 表当前节点状态
            dp_table[i][j].total_dist = best_d;
            dp_table[i][j].total_manip = best_m;
            dp_table[i][j].prev_idx = best_k;
        }
    }

    // ==========================================
    // 3. 全局最优终点选择 (再次应用 10% 策略)
    // ==========================================
    double global_min_dist = std::numeric_limits<double>::max();
    for (const auto& state : dp_table[n_segs - 1]) {
        if (state.total_dist < global_min_dist) global_min_dist = state.total_dist;
    }

    double final_threshold = global_min_dist * 1.1;
    double max_final_manip = -1.0;
    int best_final_idx = 0;

    for (size_t j = 0; j < dp_table[n_segs - 1].size(); ++j) {
        if (dp_table[n_segs - 1][j].total_dist <= final_threshold) {
            if (dp_table[n_segs - 1][j].total_manip > max_final_manip) {
                max_final_manip = dp_table[n_segs - 1][j].total_manip;
                best_final_idx = static_cast<int>(j);
            }
        }
    }

    // ==========================================
    // 4. 回溯并组装结果路径
    // ==========================================
    std::vector<std::vector<Vector6d>> result(n_segs);
    int current_idx = best_final_idx;
    for (int i = n_segs - 1; i >= 0; --i) {
        result[i] = all_segments_paths[i][current_idx];
        current_idx = dp_table[i][current_idx].prev_idx;
    }

    return result;
}

double PoseEstimation::calculateManipulability(const std::vector<Vector6d> &path) const
{
    if (path.empty()) return 0.0;
    double worst_m = std::numeric_limits<double>::max();
    for (const auto& q : path) {
        double manipulability = computeYoshikawaManipulability(q);
        worst_m = std::min(worst_m, manipulability);
    }
    return worst_m;
}

std::vector<std::vector<Vector6d>> PoseEstimation::cartesianPlanning(const std::vector<std::vector<Matrix4d> > &trajectories) const
{
    if (trajectories.empty()) {
        return {};
    }

    // 为所有轨迹点求解逆运动学
    std::vector<std::vector<std::vector<Vector6d>>> all_solutions;

    // 遍历所有轨迹段的所有点
    for (size_t traj_idx = 0; traj_idx < trajectories.size(); ++traj_idx) {
        const auto& trajectory = trajectories[traj_idx];
        std::vector<std::vector<Vector6d>> traj_solutions;

        for (const auto& pose : trajectory) {
            std::vector<Vector6d> point_solutions = inverseKinematics(pose);
            traj_solutions.push_back(point_solutions);
        }

        // 检查当前轨迹段是否有解
        for (size_t i = 0; i < traj_solutions.size(); i++) {
            if (traj_solutions[i].empty()) {
                return {};
            }
        }

        all_solutions.push_back(traj_solutions);
    }

    // 创建有效解标记
    std::vector<std::vector<std::vector<bool>>> valid_solutions;
    for (size_t traj_idx = 0; traj_idx < all_solutions.size(); ++traj_idx) {
        const auto& traj_solutions = all_solutions[traj_idx];
        std::vector<std::vector<bool>> traj_valid;

        for (size_t point_idx = 0; point_idx < traj_solutions.size(); ++point_idx) {
            const auto& point_solutions = traj_solutions[point_idx];
            std::vector<bool> point_valid(point_solutions.size(), false);

            for (size_t sol_idx = 0; sol_idx < point_solutions.size(); ++sol_idx) {
                point_valid[sol_idx] = isValidSolution(point_solutions[sol_idx], qlimits);
            }

            // 检查当前点是否有有效解
            bool has_valid = false;
            for (bool valid : point_valid) {
                if (valid) {
                    has_valid = true;
                    break;
                }
            }
            if (!has_valid) {
                return {};
            }

            traj_valid.push_back(point_valid);
        }
        valid_solutions.push_back(traj_valid);
    }

    // 计算总点数
    size_t total_points = 0;
    std::vector<size_t> segment_start_indices;
    for (const auto& traj : trajectories) {
        segment_start_indices.push_back(total_points);
        total_points += traj.size();
    }

    // 初始化动态规划表
    std::vector<std::vector<double>> dp(total_points);
    std::vector<std::vector<int>> prev(total_points);
    std::vector<std::vector<std::pair<size_t, size_t>>> solution_indices(total_points);

    // 初始化第一段轨迹的第一个点
    size_t first_point_solutions = all_solutions[0][0].size();
    dp[0].resize(first_point_solutions, 0.0);
    prev[0].resize(first_point_solutions, -1);
    solution_indices[0].resize(first_point_solutions, {0, 0});

    // 全局动态规划计算最小代价路径
    size_t current_global_index = 0;

    for (size_t traj_idx = 0; traj_idx < trajectories.size(); ++traj_idx) {
        const auto& trajectory = trajectories[traj_idx];
        size_t points_in_segment = trajectory.size();

        // 处理当前轨迹段内的点
        for (size_t point_idx_in_segment = 0; point_idx_in_segment < points_in_segment; ++point_idx_in_segment) {
            size_t global_point_idx = segment_start_indices[traj_idx] + point_idx_in_segment;

            // 跳过第一点（已经初始化）
            if (global_point_idx == 0) continue;

            size_t current_solutions = all_solutions[traj_idx][point_idx_in_segment].size();
            dp[global_point_idx].resize(current_solutions, std::numeric_limits<double>::max());
            prev[global_point_idx].resize(current_solutions, -1);
            solution_indices[global_point_idx].resize(current_solutions, {traj_idx, point_idx_in_segment});

            // 找到前一个点的索引
            size_t prev_global_point_idx;
            size_t prev_traj_idx, prev_point_idx_in_segment;

            if (point_idx_in_segment > 0) {
                // 同一轨迹段内的前一个点
                prev_global_point_idx = global_point_idx - 1;
                prev_traj_idx = traj_idx;
                prev_point_idx_in_segment = point_idx_in_segment - 1;
            } else if (traj_idx > 0) {
                // 上一段轨迹的最后一个点
                prev_global_point_idx = segment_start_indices[traj_idx - 1] + trajectories[traj_idx - 1].size() - 1;
                prev_traj_idx = traj_idx - 1;
                prev_point_idx_in_segment = trajectories[traj_idx - 1].size() - 1;
            } else {
                continue; // 第一段轨迹的第一个点，已经处理过
            }

            size_t prev_solutions = all_solutions[prev_traj_idx][prev_point_idx_in_segment].size();

            // 计算代价
            for (size_t j = 0; j < current_solutions; ++j) {
                if (!valid_solutions[traj_idx][point_idx_in_segment][j]) continue;

                for (size_t k = 0; k < prev_solutions; ++k) {
                    if (!valid_solutions[prev_traj_idx][prev_point_idx_in_segment][k]) continue;

                    // 计算从上一解到当前解的代价
                    double distance_cost = angleDistance(
                        all_solutions[prev_traj_idx][prev_point_idx_in_segment][k],
                        all_solutions[traj_idx][point_idx_in_segment][j]);

                    double total_cost = dp[prev_global_point_idx][k] + distance_cost;

                    if (total_cost < dp[global_point_idx][j]) {
                        dp[global_point_idx][j] = total_cost;
                        prev[global_point_idx][j] = k;
                    }
                }
            }
        }
    }

    // 回溯找到全局最优路径
    size_t last_global_point_idx = total_points - 1;
    auto last_segment_info = solution_indices[last_global_point_idx][0];
    size_t last_traj_idx = last_segment_info.first;
    size_t last_point_idx_in_segment = last_segment_info.second;

    double min_cost = std::numeric_limits<double>::max();
    int best_last_index = -1;
    for (size_t j = 0; j < all_solutions[last_traj_idx][last_point_idx_in_segment].size(); ++j) {
        if (valid_solutions[last_traj_idx][last_point_idx_in_segment][j] &&
            dp[last_global_point_idx][j] < min_cost) {
            min_cost = dp[last_global_point_idx][j];
            best_last_index = j;
        }
    }

    if (best_last_index == -1) {
        return {};
    }

    // 回溯构建完整路径
    std::vector<std::vector<Vector6d>> result(trajectories.size());
    for (size_t i = 0; i < trajectories.size(); ++i) {
        result[i].resize(trajectories[i].size());
    }

    // 从最后一个点开始回溯
    int current_global_index_back = last_global_point_idx;
    int current_solution_index = best_last_index;

    while (current_global_index_back >= 0) {
        auto segment_info = solution_indices[current_global_index_back][current_solution_index];
        size_t traj_idx = segment_info.first;
        size_t point_idx_in_segment = segment_info.second;

        result[traj_idx][point_idx_in_segment] =
            all_solutions[traj_idx][point_idx_in_segment][current_solution_index];

        if (current_global_index_back == 0) break;

        int prev_solution_index = prev[current_global_index_back][current_solution_index];

        // 计算前一个点的全局索引
        if (point_idx_in_segment > 0) {
            current_global_index_back = segment_start_indices[traj_idx] + point_idx_in_segment - 1;
        } else if (traj_idx > 0) {
            current_global_index_back = segment_start_indices[traj_idx - 1] + trajectories[traj_idx - 1].size() - 1;
        } else {
            break;
        }

        current_solution_index = prev_solution_index;
    }

    return result;
}

bool PoseEstimation::isStateInCollision(const std::vector<double> &q) const
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
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        const auto& dh = dh_params_[i];
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
    collisionDetector.setEnvironmentMode(EnvDetectMask::STEP_ONLY);
    collisionDetector.updateRobotPose(joint_transforms);

    // 执行碰撞检测
    if(collisionDetector.checkCollision()) {
        return true;
    }
    return false;
}

bool PoseEstimation::isStateInCollision2(const std::vector<double> &q) const
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
    for (size_t i = 0; i < dh_params_.size(); ++i) {
        const auto& dh = dh_params_[i];
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

std::vector<std::vector<Vector6d> > PoseEstimation::cartesianPlanning2(const std::vector<std::vector<Matrix4d> > &trajectories) const
{
    if (trajectories.empty()) {
        return {};
    }
    std::vector<std::vector<std::vector<Vector6d>>> all_valid_joint_paths;

    for (const auto& path : trajectories) {
        if (path.empty()) {
            return {};
        }

        // 1. 计算第一段轨迹第一个点的所有逆解 (q11, q12, ..., q18)
        std::vector<Vector6d> start_solutions = inverseKinematics(path[0]);
        if (start_solutions.empty()) {
            return {};
        }

        std::vector<std::vector<Vector6d>> valid_joint_paths;

        // 2. 遍历每一个可能的起始解作为“种子”
        for (const auto& q_start : start_solutions) {

            // 验证起始解是否合法
            if (!isValidSolution(q_start, qlimits)) continue;

            std::vector<Vector6d> current_branch;
            current_branch.push_back(q_start);
            bool branch_valid = true;

            // 3. 以前一个点为基准，寻找后续点的最优解
            for (size_t i = 1; i < path.size(); ++i) {
                std::vector<Vector6d> candidates = inverseKinematics(path[i]);

                Vector6d best_q;
                if (findBestNextSolution(current_branch.back(), candidates, best_q)) {
                    current_branch.push_back(best_q);
                } else {
                    branch_valid = false;
                    break;
                }
            }

            // 4. 如果整条路径都顺利通过验证，存入结果
            if (branch_valid) {
                valid_joint_paths.push_back(current_branch);
            }
        }

        if(valid_joint_paths.empty()) {
            return {};
        }

        all_valid_joint_paths.push_back(valid_joint_paths);
    }

    std::vector<std::vector<Vector6d>> result = selectBestPaths(all_valid_joint_paths);

    return result;
}

std::vector<Vector6d> PoseEstimation::selectBestTrajectory(std::vector<Matrix4d> targetPose) const
{
    std::vector<std::vector<Vector6d>> solutions;

    for (const auto &target : targetPose) {
        std::vector<Vector6d> pointSolutions = inverseKinematics(target);
        solutions.push_back(pointSolutions);
    }
    for (size_t i = 0; i < solutions.size(); i++) {
        if (solutions[i].empty()) {
            return {};
        }
    }


    size_t points = solutions.size();
    std::vector<std::vector<double>> dp(points);
    std::vector<std::vector<int>> prev(points);

    size_t firstPoint = solutions[0].size();
    dp[0].resize(firstPoint, 0.0);
    prev[0].resize(firstPoint, -1);

    // 为每个点过滤有效解
    std::vector<std::vector<bool>> valid_solutions(points);
    for (size_t i = 0; i < points; ++i) {
        valid_solutions[i].resize(solutions[i].size(), false);
        for (size_t j = 0; j < solutions[i].size(); ++j) {
            valid_solutions[i][j] = isValidSolution(solutions[i][j], qlimits);
        }
    }
    for (size_t i = 0; i < valid_solutions.size(); i++) {
        bool allFalse = true;
        for (size_t j = 0; j < valid_solutions[i].size(); j++) {
            if (valid_solutions[i][j]) {
                allFalse = false;
                break;
            }
        }
        if (allFalse) {
            return {};
        }
    }

    // 动态规划计算最小代价路径
    for (size_t i = 1; i < points; ++i) {
        size_t current_solutions = solutions[i].size();
        size_t prev_solutions = solutions[i-1].size();

        dp[i].resize(current_solutions, std::numeric_limits<double>::max());
        prev[i].resize(current_solutions, -1);

        for (size_t j = 0; j < current_solutions; ++j) {
            if (!valid_solutions[i][j]) continue;

            for (size_t k = 0; k < prev_solutions; ++k) {
                if (!valid_solutions[i-1][k]) continue;

                // 计算从上一解到当前解的代价
                double distance_cost = angleDistance(
                    solutions[i-1][k],
                    solutions[i][j]);

                double total_cost = dp[i-1][k] + distance_cost;

                if (total_cost < dp[i][j]) {
                    dp[i][j] = total_cost;
                    prev[i][j] = k;
                }
            }
        }
    }

    std::vector<Vector6d> best_trajectory(points);

    double min_cost = std::numeric_limits<double>::max();
    int best_last_index = -1;
    for (size_t j = 0; j < solutions[points-1].size(); ++j) {
        if (valid_solutions[points-1][j] && dp[points-1][j] < min_cost) {
            min_cost = dp[points-1][j];
            best_last_index = j;
        }
    }

    best_trajectory[points-1] = solutions[points-1][best_last_index];
    int current_index = best_last_index;

    for (int i = points-2; i >= 0; --i) {
        current_index = prev[i+1][current_index];
        best_trajectory[i] = solutions[i][current_index];
    }

    return best_trajectory;
}




