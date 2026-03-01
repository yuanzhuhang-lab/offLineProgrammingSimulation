#ifndef IMPROVEDRRTSTAR_H
#define IMPROVEDRRTSTAR_H

#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>
#include <utility>
#include "collision_detection/CollisionDetection.h"
#include "model_processing/common.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/DiscreteMotionValidator.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class ImprovedRRTStar
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

    std::vector<DHParameters> dh_params = {
            {155.0, M_PI/2.0, 450.0, M_PI/2.0},
            {614.0, 0.0, 0.0, M_PI/2.0},
            {200.0, M_PI/2.0, 0.0, 0.0},
            {0.0, -M_PI/2.0, 640.0, 0.0},
            {0.0, -M_PI/2.0, 0.0, M_PI},
            {0.0, 0.0, 0.0, 0.0}
        };

    explicit ImprovedRRTStar();

    ~ImprovedRRTStar();

private:
    ompl::base::State* goal6_state_ = nullptr;


public:
    bool plan(const std::vector<double>& q_start, const std::vector<double>& q_goal, std::vector<std::vector<double>>& finalPath);

    bool isStateInCollision(const std::vector<double>& q);
};

#endif




