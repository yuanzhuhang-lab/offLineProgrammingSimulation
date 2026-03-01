#ifndef OFFLINEPROGRAMMINGSIMULATIONMAINWINDOW_H
#define OFFLINEPROGRAMMINGSIMULATIONMAINWINDOW_H

#include "ui/head/headOfMainWindow.h"
#include "ui/EdgeVisualizer.h"
#include "model_processing/ModelProcessing.h"
#include "pose_estimation/PoseEstimation.h"
#include "collision_detection/CollisionDetection.h"
#include "path_planning/ImprovedRRTStar.h"
#include "robot_config/RobotConfig.h"

#include <vtkCubeSource.h>        // 创建立方体
#include <vtkPolyDataMapper.h>    // 几何映射
#include <vtkActor.h>             // 可视化actor
#include <vtkProperty.h>          // actor属性
#include <vtkTransform.h>         // 变换操作
#include <vtkMatrix4x4.h>         // 矩阵操作
#include <vtkSmartPointer.h>      // 智能指针
#include <vtkRenderer.h>          // 渲染器
#include <vtkRenderWindow.h>      // 渲染窗口
#include <vtkCamera.h>            // 相机
#include <vtkLineSource.h>        // 创建线条
#include <vtkPolyLineSource.h>    // 创建多段线
#include <vtkTubeFilter.h>        // 创建管道
#include <fstream>                // 文件操作


#pragma execution_character_set("utf-8")

namespace Ui {
class OffLineProgrammingSimulationMainWindow;
}

VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)

enum class MotionMode {
    JOINT_ONLY,      // 只实现关节运动
    JOINT_AND_BASE   // 关节和基座同时运动
};

enum class WeldingType {
    PIPE_TO_PLATE,   // 管板
    PLATE_TO_PLATE,  // 板板
    PIPE_TO_PIPE     // 管管
};

struct MotionTarget {
    Vector6d jointAngles;  // 关节角度
    double baseX;          // 基座X坐标
    double baseY;          // 基座Y坐标
    double baseZ;          // 基座Z坐标
    MotionMode motionMode; // 运动模式

    MotionTarget() : baseX(0), baseY(0), baseZ(0), motionMode(MotionMode::JOINT_AND_BASE) {
        jointAngles.setZero();
    }

    MotionTarget(const Vector6d& angles, double x = 0, double y = 0, double z = 0, MotionMode mode = MotionMode::JOINT_AND_BASE)
        : jointAngles(angles), baseX(x), baseY(y), baseZ(z), motionMode(mode) {}
};


class OffLineProgrammingSimulationMainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit OffLineProgrammingSimulationMainWindow(QWidget *parent = nullptr);
    ~OffLineProgrammingSimulationMainWindow();

private slots:
    void on_horizontalSlider_4_valueChanged(int value);

    void on_horizontalSlider_5_valueChanged(int value);

    void on_horizontalSlider_6_valueChanged(int value);

    void on_horizontalSlider_7_valueChanged(int value);

    void on_horizontalSlider_8_valueChanged(int value);

    void on_horizontalSlider_9_valueChanged(int value);

    void on_toolButton_clicked();

    void on_toolButton_2_clicked();

    void on_toolButton_3_clicked();

    void on_toolButton_4_clicked();

    void on_toolButton_5_clicked();

    void on_toolButton_6_clicked();

    void on_toolButton_7_clicked();

    void on_toolButton_8_clicked();

    void on_toolButton_9_clicked();

    void on_toolButton_10_clicked();

    void on_toolButton_11_clicked();

    void on_radioButton_clicked();

    void on_radioButton_2_clicked();

    void on_radioButton_3_clicked();

    void on_toolButton_12_clicked();

private:
    Ui::OffLineProgrammingSimulationMainWindow *ui;

    TopoDS_Shape stepShape;

    WeldingType currentWeldingType_;

    vtkRenderer *renderer;
    vtkRenderer *renderer2;
    vtkSmartPointer<vtkCamera> camera;

    std::unique_ptr<EdgeVisualizer> edgeVisualizer;

    ModelProcessing modelprocess;

    std::vector<EdgeInfo> edgeInfo;
    std::unordered_set<int> selected_edge_indices_;
    std::vector<EdgeInfo> selected_edgeInfo;

    std::vector<vtkSmartPointer<vtkActor>> sample_point_actors_;
    std::vector<vtkSmartPointer<vtkActor>> normal_actors_;

    vtkSmartPointer<vtkActor> toolXAxisActor;
    vtkSmartPointer<vtkActor> toolYAxisActor;
    vtkSmartPointer<vtkActor> toolZAxisActor;
    std::vector<std::vector<CoordinateSystem>> all_coordinate_systems_B_;
    std::vector<Matrix4d> viewpoint;

    std::vector<vtkSmartPointer<vtkAssembly>> jointAssemblies;

    vtkSmartPointer<vtkAssembly> baseAssembly;
    vtkSmartPointer<vtkTransform> transform;
    Vector6d targetAnglesDeg_;
    Vector6d startAnglesDeg_;
    int interpolationStep_;
    int totalInterpolationSteps_;
    QTimer* smoothMotionTimer_;
    MotionMode currentMotionMode_;

    std::queue<MotionTarget> motionQueue_;  // 运动队列，存储关节角度和基座位置

    // 添加基座移动目标变量
    double targetBaseX_;  // 目标基座X坐标
    double targetBaseY_;  // 目标基座Y坐标
    double targetBaseZ_;  // 目标基座Z坐标
    double startBaseX_;   // 起始基座X坐标
    double startBaseY_;   // 起始基座Y坐标
    double startBaseZ_;   // 起始基座Z坐标

    // 添加baseAssembly移动相关的成员变量
    double baseAssemblyX_;  // X轴平移量
    double baseAssemblyY_;  // Y轴平移量
    double baseAssemblyZ_;  // Z轴平移量
    Matrix4d baseTransformMatrix_;
    Matrix4d link1TransformMatrix_;  // link1相对世界坐标系的齐次矩阵
    Matrix4d link2TransformMatrix_;  // link2相对世界坐标系的齐次矩阵
    Matrix4d link3TransformMatrix_;  // link3相对世界坐标系的齐次矩阵
    Matrix4d link4TransformMatrix_;  // link4相对世界坐标系的齐次矩阵
    Matrix4d link5TransformMatrix_;  // link5相对世界坐标系的齐次矩阵
    Matrix4d link6TransformMatrix_;
    Matrix4d toolTransformMatrix_;   // tool相对世界坐标系的齐次矩阵

    bool isProcessingQueue_;

    vtkSmartPointer<vtkAxesActor> baseAxes;
    vtkSmartPointer<vtkAxesActor> toolAxes;

    std::unique_ptr<PoseEstimation> solver;

    CollisionDetection* collisionDetector = nullptr;


    std::unique_ptr<RobotConfig> robotConfig;


    std::vector<vtkSmartPointer<vtkActor>> collisionBoxActors_;
    std::vector<vtkSmartPointer<vtkCubeSource>> collisionBoxSources_;
    bool showCollisionBoxes_;

    std::vector<Point3D> toolPathPoints_;
    vtkSmartPointer<vtkActor> toolPathActor_;
    bool showToolPath_;

public:
    void initVTKDisplay();

    void loadingModel(const QString &fileName);

    void VisualizeSelectedEdgePointsAndNormals();

    void ClearVisualization();

    const EdgeInfo* GetEdgeInfo(int edge_idx) const;

    CoordinateSystem createCoordinate(const Point3D &origin, const Normal3D &x_axis, const Normal3D &y_axis, const Normal3D &z_axis, double y_axis_angle_deg, double z_axis_angle_deg);

    vtkSmartPointer<vtkMatrix4x4> DH_to_Matrix(double a, double alpha, double d, double theta);

    void CreateJointMesh(int jointIndex);

    vtkSmartPointer<vtkAssembly> BuildKinematicChain();

    void UpdateJointTransforms(const Vector6d &anglesDeg);

    void loadingRobot();

    void calculateHanju();

    void SetJointAnglesDeg(const Vector6d &anglesDeg, double baseX = 0, double baseY = 0, double baseZ = 0, MotionMode mode = MotionMode::JOINT_ONLY);

    void onSmoothMotionTimer();

    void initQSliders();

    void on_horizontalSlider_valueChanged(int value);

    void on_horizontalSlider_2_valueChanged(int value);

    void on_horizontalSlider_3_valueChanged(int value);

    void updateBaseAssemblyTransform();

    void updateTransformMatrix();

    void updateTransformMatrixLabel();

    void processNextMotion();

    void updateJointAnglesLabel();

    void updateCollisionBoxes();
    void toggleCollisionBoxesVisibility(bool visible);
    void setCollisionBoxColor(int boxIndex, double r, double g, double b);
    void performCollisionDetection();

    // 焊枪路径记录和可视化相关函数
    void recordToolPosition();  // 记录当前焊枪位置
    void createToolPathVisualization();  // 创建焊枪路径可视化
    void updateToolPathVisualization();  // 更新焊枪路径可视化
    void clearToolPath();  // 清除焊枪路径
    void toggleToolPathVisibility(bool visible);  // 切换焊枪路径显示状态
    void saveToolPathToFile(const std::string& filename);  // 保存焊枪路径到文件

private:
    std::vector<std::vector<Vector6d>> recordedTrajectories_;  // 记录的轨迹数据（关节角度）
    std::vector<std::vector<Vector3d>> recordedBaseTrajectories_;  // 记录的基座轨迹数据
    std::vector<std::vector<Vector6d>> recordedMotions_;
    std::vector<bool> recordedPlanningSuccess_;  // 记录每段轨迹的规划成功状态
    bool isRecordingTrajectory_;  // 是否正在记录轨迹
    int currentTrajectoryIndex_;  // 当前轨迹索引

    bool isReplayingTrajectory_;  // 是否正在回放轨迹
    bool isPaused_;  // 是否暂停回放
    int currentReplaySegment_;  // 当前回放的轨迹段索引
    int currentReplayPoint_;  // 当前回放的轨迹点索引
    QTimer* replayTimer_;  // 轨迹回放定时器

    std::vector<Vector3d> saved_base_positions_;  // 保存的基座位置
    std::vector<std::vector<double>> saved_redundant_angles_;  // 保存的冗余角度
    std::vector<Vector3d> saved_centroids_;  // 保存的质心位置

    void startTrajectoryRecording();  // 开始记录轨迹
    void stopTrajectoryRecording();  // 停止记录轨迹
    void saveTrajectoryToFile(const QString& filename);  // 保存轨迹到文件
    void loadTrajectoryFromFile(const QString& filename);  // 从文件加载轨迹
    void replayTrajectory(int trajectoryIndex = -1);  // 复现轨迹
    void pauseReplayTrajectory();  // 暂停轨迹回放
    void resumeReplayTrajectory();  // 继续轨迹回放
    void stopReplayTrajectory();  // 停止轨迹回放
    void onReplayTimer();  // 轨迹回放定时器回调
    void clearRecordedTrajectories();  // 清除记录的轨迹
    void saveDataToJson(const QString& filename);  // 保存数据到JSON文件

    std::vector<vtkSmartPointer<vtkActor>> trajectoryPointActors_;  // 轨迹点可视化actors
    std::vector<vtkSmartPointer<vtkActor>> trajectoryLineActors_;

    void createCustomToolAxes();

    void updateCustomToolAxes();
};

#endif // OFFLINEPROGRAMMINGSIMULATIONMAINWINDOW_H
