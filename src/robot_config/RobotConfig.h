#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <string>
#include <vector>
#include <utility>
#include <QJsonObject>
#include <QJsonArray>
#include <QJsonDocument>
#include <QFile>
#include <QDebug>
#include <QTextEdit>
#include "model_processing/common.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

struct JointConfig {
    double a;          // DH参数 - 连杆长度
    double alpha;      // DH参数 - 连杆扭角
    double d;          // DH参数 - 连杆偏距
    double theta;      // DH参数 - 关节角度初始值
    double minAngle;   // 关节角度下限（弧度）
    double maxAngle;   // 关节角度上限（弧度）
    std::string stlFile; // 模型文件路径
    std::vector<double> color; // 颜色 [R, G, B]

    JointConfig() : a(0), alpha(0), d(0), theta(0), minAngle(-M_PI), maxAngle(M_PI), color({1.0, 1.0, 1.0}) {}
};

class RobotConfig
{
public:
    RobotConfig();
    ~RobotConfig();

    // 从JSON文件加载配置
    bool loadFromFile(const std::string& filepath);

    // 保存配置到JSON文件
    bool saveToFile(const std::string& filepath) const;

    // 获取所有关节配置
    const std::vector<JointConfig>& getJointConfigs() const { return jointConfigs; }

    // 获取指定关节配置
    const JointConfig& getJointConfig(int index) const;

    // 获取关节数量
    int getJointCount() const { return jointConfigs.size(); }

    // 获取DH参数
    std::vector<DHParameters> getDHParameters() const;

    // 获取关节角度限制
    std::vector<std::pair<double, double>> getJointLimits() const;

    // 获取模型路径
    std::vector<std::string> getStlPaths() const;

    // 设置配置
    void setJointConfigs(const std::vector<JointConfig>& configs) { jointConfigs = configs; }

    // 获取基础模型路径
    const std::string& getBaseStlPath() const { return baseStlPath; }

    // 获取工具模型路径
    const std::string& getToolStlPath() const { return toolStlPath; }

    // 获取工具变换矩阵
    const Eigen::Matrix4d& getToolTransform() const { return toolTransform; }

    // 获取零件模型路径
    const std::string& getCameraStlPath() const { return cameraStlPath; }

    // 获取零件变换矩阵
    const Eigen::Matrix4d& getCameraTransform() const { return cameraTransform; }

private:
    std::vector<JointConfig> jointConfigs;
    std::string baseStlPath;
    std::string toolStlPath;
    std::string cameraStlPath;
    Eigen::Matrix4d toolTransform;
    Eigen::Matrix4d cameraTransform;

    // 内部辅助函数
    JointConfig parseJointConfig(const QJsonObject& jointObj) const;
    QJsonObject createJointConfigObject(const JointConfig& joint) const;
};

#endif // ROBOTCONFIG_H
