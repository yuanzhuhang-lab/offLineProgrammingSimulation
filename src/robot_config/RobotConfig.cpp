#include "RobotConfig.h"
#include <iostream>
#include <cmath>

RobotConfig::RobotConfig()
{
    // 初始化工具变换矩阵为单位矩阵
    toolTransform = Eigen::Matrix4d::Identity();
    cameraTransform = Eigen::Matrix4d::Identity();
}

RobotConfig::~RobotConfig()
{
}

bool RobotConfig::loadFromFile(const std::string& filepath)
{
    QFile file(QString::fromStdString(filepath));
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "Cannot open robot config file:" << QString::fromStdString(filepath);
        return false;
    }

    QByteArray data = file.readAll();
    QJsonDocument doc = QJsonDocument::fromJson(data);
    if (doc.isNull() || !doc.isObject()) {
        qDebug() << "Invalid JSON in robot config file:" << QString::fromStdString(filepath);
        return false;
    }

    QJsonObject rootObj = doc.object();

    // 读取基础模型路径
    if (rootObj.contains("baseStlPath") && rootObj["baseStlPath"].isString()) {
        baseStlPath = rootObj["baseStlPath"].toString().toStdString();
    }

    // 读取工具模型路径
    if (rootObj.contains("toolStlPath") && rootObj["toolStlPath"].isString()) {
        toolStlPath = rootObj["toolStlPath"].toString().toStdString();
    }

    // 读取零件模型路径
    if (rootObj.contains("cameraStlPath") && rootObj["cameraStlPath"].isString()) {
        cameraStlPath = rootObj["cameraStlPath"].toString().toStdString();
    }

    // 读取工具变换矩阵
    if (rootObj.contains("toolTransform") && rootObj["toolTransform"].isArray()) {
        QJsonArray transformArray = rootObj["toolTransform"].toArray();
        if (transformArray.size() == 16) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    int index = i * 4 + j;
                    if (transformArray[index].isDouble()) {
                        toolTransform(i, j) = transformArray[index].toDouble();
                    }
                }
            }
        }
    }

    // 读取零件变换矩阵
    if (rootObj.contains("cameraTransform") && rootObj["cameraTransform"].isArray()) {
        QJsonArray transformArray = rootObj["cameraTransform"].toArray();
        if (transformArray.size() == 16) {
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    int index = i * 4 + j;
                    if (transformArray[index].isDouble()) {
                        cameraTransform(i, j) = transformArray[index].toDouble();
                    }
                }
            }
        }
    }

    // 读取关节配置
    if (rootObj.contains("joints") && rootObj["joints"].isArray()) {
        QJsonArray jointsArray = rootObj["joints"].toArray();
        jointConfigs.clear();
        jointConfigs.reserve(jointsArray.size());

        for (const auto& jointValue : jointsArray) {
            if (jointValue.isObject()) {
                JointConfig joint = parseJointConfig(jointValue.toObject());
                jointConfigs.push_back(joint);
            }
        }
    }

    file.close();
    return true;
}

bool RobotConfig::saveToFile(const std::string& filepath) const
{
    QJsonObject rootObj;

    // 保存基础模型路径
    rootObj["baseStlPath"] = QString::fromStdString(baseStlPath);

    // 保存工具模型路径
    rootObj["toolStlPath"] = QString::fromStdString(toolStlPath);

    // 保存零件模型路径
    rootObj["cameraStlPath"] = QString::fromStdString(cameraStlPath);

    // 保存工具变换矩阵
    QJsonArray transformArray;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            transformArray.append(toolTransform(i, j));
        }
    }
    rootObj["toolTransform"] = transformArray;

    // 保存零件变换矩阵
    QJsonArray partTransformArray;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            partTransformArray.append(cameraTransform(i, j));
        }
    }
    rootObj["cameraTransform"] = partTransformArray;

    // 保存关节配置
    QJsonArray jointsArray;
    for (const auto& joint : jointConfigs) {
        jointsArray.append(createJointConfigObject(joint));
    }
    rootObj["joints"] = jointsArray;

    QJsonDocument doc(rootObj);
    QFile file(QString::fromStdString(filepath));
    if (!file.open(QIODevice::WriteOnly)) {
        qDebug() << "Cannot write to robot config file:" << QString::fromStdString(filepath);
        return false;
    }

    file.write(doc.toJson());
    file.close();
    return true;
}

const JointConfig& RobotConfig::getJointConfig(int index) const
{
    if (index >= 0 && index < static_cast<int>(jointConfigs.size())) {
        return jointConfigs[index];
    }
    static JointConfig emptyConfig;
    return emptyConfig;
}

std::vector<DHParameters> RobotConfig::getDHParameters() const
{
    std::vector<DHParameters> dhParams;
    dhParams.reserve(jointConfigs.size());

    for (const auto& joint : jointConfigs) {
        DHParameters dh;
        dh.a = joint.a;
        dh.alpha = joint.alpha;
        dh.d = joint.d;
        dh.theta = joint.theta;
        dhParams.push_back(dh);
    }

    return dhParams;
}

std::vector<std::pair<double, double>> RobotConfig::getJointLimits() const
{
    std::vector<std::pair<double, double>> limits;
    limits.reserve(jointConfigs.size());

    for (const auto& joint : jointConfigs) {
        limits.emplace_back(joint.minAngle, joint.maxAngle);
    }

    return limits;
}

std::vector<std::string> RobotConfig::getStlPaths() const
{
    std::vector<std::string> paths;
    paths.reserve(jointConfigs.size());

    for (const auto& joint : jointConfigs) {
        paths.push_back(joint.stlFile);
    }

    return paths;
}

JointConfig RobotConfig::parseJointConfig(const QJsonObject& jointObj) const
{
    JointConfig joint;

    // 读取DH参数
    if (jointObj.contains("a") && jointObj["a"].isDouble()) {
        joint.a = jointObj["a"].toDouble();
    }
    if (jointObj.contains("alpha") && jointObj["alpha"].isDouble()) {
        joint.alpha = jointObj["alpha"].toDouble();
    }
    if (jointObj.contains("d") && jointObj["d"].isDouble()) {
        joint.d = jointObj["d"].toDouble();
    }
    if (jointObj.contains("theta") && jointObj["theta"].isDouble()) {
        joint.theta = jointObj["theta"].toDouble();
    }

    // 读取关节角度限制
    if (jointObj.contains("minAngle") && jointObj["minAngle"].isDouble()) {
        joint.minAngle = jointObj["minAngle"].toDouble();
    }
    if (jointObj.contains("maxAngle") && jointObj["maxAngle"].isDouble()) {
        joint.maxAngle = jointObj["maxAngle"].toDouble();
    }

    // 读取模型文件路径
    if (jointObj.contains("stlFile") && jointObj["stlFile"].isString()) {
        joint.stlFile = jointObj["stlFile"].toString().toStdString();
    }

    // 读取颜色
    if (jointObj.contains("color") && jointObj["color"].isArray()) {
        QJsonArray colorArray = jointObj["color"].toArray();
        if (colorArray.size() >= 3) {
            joint.color.resize(3);
            for (int i = 0; i < 3 && i < colorArray.size(); ++i) {
                if (colorArray[i].isDouble()) {
                    joint.color[i] = colorArray[i].toDouble();
                }
            }
        }
    } else {
        joint.color = {1.0, 1.0, 1.0}; // 默认白色
    }

    return joint;
}

QJsonObject RobotConfig::createJointConfigObject(const JointConfig& joint) const
{
    QJsonObject jointObj;

    // 保存DH参数
    jointObj["a"] = joint.a;
    jointObj["alpha"] = joint.alpha;
    jointObj["d"] = joint.d;
    jointObj["theta"] = joint.theta;

    // 保存关节角度限制
    jointObj["minAngle"] = joint.minAngle;
    jointObj["maxAngle"] = joint.maxAngle;

    // 保存模型文件路径
    jointObj["stlFile"] = QString::fromStdString(joint.stlFile);

    // 保存颜色
    QJsonArray colorArray;
    for (double colorVal : joint.color) {
        colorArray.append(colorVal);
    }
    jointObj["color"] = colorArray;

    return jointObj;
}
