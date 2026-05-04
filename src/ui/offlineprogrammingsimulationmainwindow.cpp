#include "offlineprogrammingsimulationmainwindow.h"
#include "ui_offlineprogrammingsimulationmainwindow.h"

namespace {
const char* motionTaskTypeToString(MotionTaskType taskType)
{
    switch (taskType) {
    case MotionTaskType::CAMERA_CAPTURE:
        return "CAMERA_CAPTURE";
    case MotionTaskType::WELDING:
        return "WELDING";
    case MotionTaskType::TRANSITION:
    default:
        return "TRANSITION";
    }
}

MotionTaskType motionTaskTypeFromString(const std::string& text)
{
    if (text.find("CAMERA_CAPTURE") != std::string::npos || text.find("PHOTO") != std::string::npos) {
        return MotionTaskType::CAMERA_CAPTURE;
    }

    if (text.find("WELDING") != std::string::npos) {
        return MotionTaskType::WELDING;
    }

    return MotionTaskType::TRANSITION;
}

std::vector<double> parseDoublesFromLine(const std::string& line)
{
    std::string dataLine = line;
    size_t colonPos = dataLine.find(":");
    if (colonPos != std::string::npos) {
        dataLine = dataLine.substr(colonPos + 1);
    }

    dataLine.erase(std::remove(dataLine.begin(), dataLine.end(), ' '), dataLine.end());

    std::vector<double> values;
    std::stringstream ss(dataLine);
    std::string token;

    while (std::getline(ss, token, ',')) {
        try {
            values.push_back(std::stod(token));
        } catch (const std::exception&) {
            continue;
        }
    }

    return values;
}

QString formatVector3dForInfo(const Vector3d& value)
{
    return QString("(%1, %2, %3)")
        .arg(value.x(), 0, 'f', 3)
        .arg(value.y(), 0, 'f', 3)
        .arg(value.z(), 0, 'f', 3);
}
}

OffLineProgrammingSimulationMainWindow::OffLineProgrammingSimulationMainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::OffLineProgrammingSimulationMainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle("离线编程仿真系统");

    initVTKDisplay();

    smoothMotionTimer_ = new QTimer(this);
    connect(smoothMotionTimer_, &QTimer::timeout, this, &OffLineProgrammingSimulationMainWindow::onSmoothMotionTimer);

    Vector6d angles;
    angles << 0, 0, 0, 0, 0, 0;
    startAnglesDeg_ = angles;
    targetAnglesDeg_ = angles;
    interpolationStep_ = 0;

    baseAssemblyX_ = -1000.0;
    baseAssemblyY_ = 1000.0;
    baseAssemblyZ_ = 2100.0;
    link1TransformMatrix_ = Matrix4d::Identity();
    link2TransformMatrix_ = Matrix4d::Identity();
    link3TransformMatrix_ = Matrix4d::Identity();
    link4TransformMatrix_ = Matrix4d::Identity();
    link5TransformMatrix_ = Matrix4d::Identity();
    link6TransformMatrix_ = Matrix4d::Identity();
    toolTransformMatrix_ = Matrix4d::Identity();


    // 初始化QSlider
    initQSliders();
    isProcessingQueue_ = false;
    currentMotionMode_ = MotionMode::JOINT_ONLY;
    currentMotionTaskType_ = MotionTaskType::TRANSITION;
    currentHasProjectorPose_ = false;
    currentProjectorPose_ = Matrix4d::Identity();
    currentHasStructuredLightGeometry_ = false;
    currentFrustumPoints_.fill(Vector3d::Zero());
    currentCoveredVertices_.clear();

    currentWeldingType_ = WeldingType::PLATE_TO_PLATE;

    showCollisionBoxes_ = true;
    structuredLightActorsAdded_ = false;
    hasLastStructuredLightProjectorPose_ = false;
    lastStructuredLightProjectorPose_ = Matrix4d::Identity();
    structuredLightFovY_ = 52.0;
    structuredLightAspectRatio_ = 0.8928;
    structuredLightNearDist_ = 350.0;
    structuredLightFarDist_ = 600.0;
    structuredLightCoverageSampleStep_ = 20.0;
    structuredLightProjectorCamera_ = vtkSmartPointer<vtkCamera>::New();
    structuredLightProjectorCamera_->SetViewAngle(structuredLightFovY_);
    structuredLightProjectorCamera_->SetClippingRange(structuredLightNearDist_, structuredLightFarDist_);

    isRecordingTrajectory_ = false;
    currentTrajectoryIndex_ = -1;
    isReplayingTrajectory_ = false;
    isPaused_ = false;
    currentReplaySegment_ = -1;
    currentReplayPoint_ = -1;
    replayTimer_ = nullptr;

    robotConfig = std::make_unique<RobotConfig>();
    std::string configPath = "D:/Qt_develop/offLineProgrammingSimulation/config/robotConfig.json";
    if (!robotConfig->loadFromFile(configPath)) {
        qDebug() << "Failed to load robot config from:" << QString::fromStdString(configPath);
    }

    jointAssemblies.clear();
}

OffLineProgrammingSimulationMainWindow::~OffLineProgrammingSimulationMainWindow()
{
    delete ui;
}

void OffLineProgrammingSimulationMainWindow::initVTKDisplay()
{
    showToolPath_ = false;

    auto renderer = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    ui->qvtkWidget->SetRenderWindow(renderWindow);
    camera = vtkSmartPointer<vtkCamera>::New();
    camera->ParallelProjectionOn();
}

void OffLineProgrammingSimulationMainWindow::loadingModel(const QString &fileName)
{
    STEPControl_Reader reader;
    std::string stpName = fileName.toStdString();

    IFSelect_ReturnStatus readStatus = reader.ReadFile(stpName.c_str());
    if (readStatus != IFSelect_RetDone) {
        throw std::runtime_error(u8"无法加载STEP文件。");
        return;
    }

    Standard_Integer nbRoots = reader.TransferRoots();
    if (nbRoots == 0) {
        throw std::runtime_error(u8"STEP文件中没有可转移的根对象。");
    }

    stepShape = reader.OneShape();

    collisionDetector = &CollisionDetection::getInstance();

    edgeInfo = modelprocess.ExtractEdgesFromStep(stepShape);

    BRepMesh_IncrementalMesh mesher(stepShape, 0.1, false, 0.1, true);
    mesher.Perform();

    vtkSmartPointer<vtkPolyData> stlPolyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();

    int pointIndex = 0;

    for (TopExp_Explorer faceExplorer(stepShape, TopAbs_FACE); faceExplorer.More(); faceExplorer.Next()) {
        TopoDS_Face face = TopoDS::Face(faceExplorer.Current());

        TopLoc_Location location;
        Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);

        if (triangulation.IsNull()) {
            continue;
        }

        int nbNodes = triangulation->NbNodes();
        int startIndex = pointIndex;

        for (int i = 1; i <= nbNodes; i++) {
            gp_Pnt point = triangulation->Node(i).Transformed(location);
            points->InsertNextPoint(point.X(), point.Y(), point.Z());
            pointIndex++;
        }

        int nbTriangles = triangulation->NbTriangles();
        for (int i = 1; i <= nbTriangles; i++) {
            Poly_Triangle triangle = triangulation->Triangle(i);
            int n1, n2, n3;
            triangle.Get(n1, n2, n3);

            n1--; n2--; n3--;

            vtkSmartPointer<vtkTriangle> vtkTri = vtkSmartPointer<vtkTriangle>::New();
            vtkTri->GetPointIds()->SetId(0, startIndex + n1);
            vtkTri->GetPointIds()->SetId(1, startIndex + n2);
            vtkTri->GetPointIds()->SetId(2, startIndex + n3);
            triangles->InsertNextCell(vtkTri);
        }
    }

    stlPolyData->SetPoints(points);
    stlPolyData->SetPolys(triangles);
    workpiecePolyData_ = stlPolyData;
    hasLastStructuredLightProjectorPose_ = false;

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(stlPolyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    actor->GetProperty()->SetColor(1, 1, 1);    

    ui->qvtkWidget->GetRenderWindow()->SetNumberOfLayers(2);

    renderer = vtkRenderer::New();
    renderer->AddActor(actor);
    renderer->SetLayer(0);
    renderer->SetBackground(0, 0, 0);

    vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
    actor->PickableOff();

    axesActor->SetTotalLength(100.0, 100.0, 100.0);
    axesActor->SetShaftTypeToLine();
    axesActor->SetConeResolution(30);
    axesActor->SetCylinderResolution(100);

    axesActor->SetXAxisLabelText("X");
    axesActor->SetYAxisLabelText("Y");
    axesActor->SetZAxisLabelText("Z");
    axesActor->AxisLabelsOn();

    renderer->AddActor(axesActor);

    renderer2 = vtkRenderer::New();
    actor->GetProperty()->SetOpacity(0.8);
    renderer2->AddActor(actor);
    renderer2->SetLayer(1);
    renderer2->SetBackground(0, 0, 0);
    renderer2->AddActor(axesActor);
    addStructuredLightActorsToRenderers();

    renderer2->SetActiveCamera(camera);
    renderer2->ResetCamera();

    vtkRenderWindowInteractor* interactor = ui->qvtkWidget->GetRenderWindow()->GetInteractor();
    if (!interactor) {
        ui->qvtkWidget->GetRenderWindow()->SetInteractor(vtkSmartPointer<vtkRenderWindowInteractor>::New());
        interactor = ui->qvtkWidget->GetRenderWindow()->GetInteractor();
    }
    ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer);
    ui->qvtkWidget->GetRenderWindow()->AddRenderer(renderer2);
    renderer2->SetDraw(0);

    edgeVisualizer.reset(new EdgeVisualizer(renderer, interactor));

    edgeVisualizer->AddEdges(edgeInfo);

    ui->qvtkWidget->update();

    renderer->SetActiveCamera(camera);
    renderer->ResetCamera();
    renderer2->SetActiveCamera(camera);
    renderer2->ResetCamera();
    renderer->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::VisualizeSelectedEdgePointsAndNormals()
{
    ClearVisualization();

    if (selected_edge_indices_.empty()) {
        return;
    }

    if (renderer) {
        renderer->SetDraw(0);
    }
    if (renderer2) {
        renderer2->SetDraw(1);
        renderer2->GetRenderWindow()->Render();
    }

    for (auto edgeInfo : selected_edgeInfo) {

        // 可视化离散点
        auto points = vtkSmartPointer<vtkPoints>::New();
        auto vertices = vtkSmartPointer<vtkCellArray>::New();

        for (size_t i = 0; i < edgeInfo.sample_points.size(); ++i) {
            const Point3D& p = edgeInfo.sample_points[i];
            vtkIdType pid = points->InsertNextPoint(p.x, p.y, p.z);
            vertices->InsertNextCell(1, &pid);
        }

        auto points_poly_data = vtkSmartPointer<vtkPolyData>::New();
        points_poly_data->SetPoints(points);
        points_poly_data->SetVerts(vertices);

        auto points_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        points_mapper->SetInputData(points_poly_data);

        auto points_actor = vtkSmartPointer<vtkActor>::New();
        points_actor->SetMapper(points_mapper);
        points_actor->GetProperty()->SetColor(0.0, 1.0, 0.0);
        points_actor->GetProperty()->SetPointSize(20.0);
        points_actor->GetProperty()->SetLighting(false);

        renderer2->AddActor(points_actor);
        sample_point_actors_.push_back(points_actor);

        if (edgeInfo.normals.size() > 0) {
            double colors[3][3] = {
                {1.0, 0.0, 0.0},
                {0.0, 1.0, 0.0},
                {0.0, 0.0, 1.0}
            };

            for (int vector_idx = 0; vector_idx < 3; ++vector_idx) {
                auto lines = vtkSmartPointer<vtkCellArray>::New();
                auto line_points = vtkSmartPointer<vtkPoints>::New();
                auto poly_lines = vtkSmartPointer<vtkPolyLine>::New();

                for (size_t i = 0; i < edgeInfo.sample_points.size(); ++i) {
                    const Point3D& p = edgeInfo.sample_points[i];
                    const auto& normal_tuple = edgeInfo.normals[i];

                    const Normal3D& vector = [&]() -> const Normal3D& {
                        switch (vector_idx) {
                            case 0: return std::get<0>(normal_tuple);
                            case 1: return std::get<1>(normal_tuple);
                            case 2: return std::get<2>(normal_tuple);
                            default: return std::get<0>(normal_tuple);
                        }
                    }();

                    double scale = 5.0;
                    Point3D end_point;
                    end_point.x = p.x + vector.x * scale;
                    end_point.y = p.y + vector.y * scale;
                    end_point.z = p.z + vector.z * scale;

                    vtkIdType start_id = line_points->InsertNextPoint(p.x, p.y, p.z);
                    vtkIdType end_id = line_points->InsertNextPoint(end_point.x, end_point.y, end_point.z);

                    auto line = vtkSmartPointer<vtkLine>::New();
                    line->GetPointIds()->SetId(0, start_id);
                    line->GetPointIds()->SetId(1, end_id);
                    lines->InsertNextCell(line);
                }

                auto lines_poly_data = vtkSmartPointer<vtkPolyData>::New();
                lines_poly_data->SetPoints(line_points);
                lines_poly_data->SetLines(lines);

                auto tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
                tubeFilter->SetInputData(lines_poly_data);
                tubeFilter->SetRadius(0.3);
                tubeFilter->SetNumberOfSides(8);
                tubeFilter->CappingOn();

                auto lines_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
                lines_mapper->SetInputConnection(tubeFilter->GetOutputPort());

                auto lines_actor = vtkSmartPointer<vtkActor>::New();
                lines_actor->SetMapper(lines_mapper);
                lines_actor->GetProperty()->SetColor(colors[vector_idx][0],
                                                    colors[vector_idx][1],
                                                    colors[vector_idx][2]);


                lines_actor->GetProperty()->SetLighting(true);
                lines_actor->GetProperty()->SetAmbient(0.3);
                lines_actor->GetProperty()->SetDiffuse(0.7);
                lines_actor->GetProperty()->SetSpecular(0.4);
                lines_actor->GetProperty()->SetSpecularPower(10);

                renderer2->AddActor(lines_actor);
                normal_actors_.push_back(lines_actor);

        }
    }}

    renderer2->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::ClearVisualization()
{
    // 清除离散点可视化
    for (auto& actor : sample_point_actors_) {
        renderer2->RemoveActor(actor);
    }
    sample_point_actors_.clear();

    // 清除法向量可视化
    for (auto& actor : normal_actors_) {
        renderer2->RemoveActor(actor);
    }
    normal_actors_.clear();

    renderer2->GetRenderWindow()->Render();
}

CoordinateSystem OffLineProgrammingSimulationMainWindow::createCoordinate(const Point3D &origin, const Normal3D &x_axis, const Normal3D &y_axis, const Normal3D &z_axis, double y_axis_angle_deg, double z_axis_angle_deg)
{
    CoordinateSystem csA;
    csA.origin = origin;
    csA.x_axis = x_axis;
    csA.y_axis = y_axis;
    csA.z_axis = z_axis;
    csA.transform_matrix <<
        x_axis.x, y_axis.x, z_axis.x, origin.x,
        x_axis.y, y_axis.y, z_axis.y, origin.y,
        x_axis.z, y_axis.z, z_axis.z, origin.z,
        0, 0, 0, 1;

    // 第一步：绕Y轴旋转（180度 + 用户指定的Y轴角度）
    double y_axis_angle_rad = y_axis_angle_deg * M_PI / 180.0;
    double total_y_angle_rad = (90.0 * M_PI / 180.0) - y_axis_angle_rad;
    double sin_y_alpha = std::sin(total_y_angle_rad);
    double cos_y_alpha = std::cos(total_y_angle_rad);

    Normal3D x_axis_intermediate = -cos_y_alpha * csA.x_axis + sin_y_alpha * csA.z_axis;
    Normal3D y_axis_intermediate = csA.y_axis;
    Normal3D z_axis_intermediate = (-sin_y_alpha) * csA.x_axis + (-cos_y_alpha) * csA.z_axis;

    // 第二步：绕Z轴旋转
    double z_axis_angle_rad = z_axis_angle_deg * M_PI / 180.0;
    double sin_z_alpha = std::sin(z_axis_angle_rad);
    double cos_z_alpha = std::cos(z_axis_angle_rad);

    Normal3D x_axis_B = cos_z_alpha * x_axis_intermediate + (-sin_z_alpha * y_axis_intermediate);
    Normal3D y_axis_B = sin_z_alpha * x_axis_intermediate + cos_z_alpha * y_axis_intermediate;
    Normal3D z_axis_B = z_axis_intermediate;

    // 归一化
    x_axis_B = modelprocess.NormalizeVec(x_axis_B);
    y_axis_B = modelprocess.NormalizeVec(y_axis_B);
    z_axis_B = modelprocess.NormalizeVec(z_axis_B);

    // 构建B系
    CoordinateSystem csB;
    csB.origin = csA.origin;
    csB.x_axis = x_axis_B;
    csB.y_axis = y_axis_B;
    csB.z_axis = z_axis_B;
    csB.transform_matrix <<
        x_axis_B.x, y_axis_B.x, z_axis_B.x, csB.origin.x,
        x_axis_B.y, y_axis_B.y, z_axis_B.y, csB.origin.y,
        x_axis_B.z, y_axis_B.z, z_axis_B.z, csB.origin.z,
        0, 0, 0, 1;

    return csB;
}

vtkSmartPointer<vtkMatrix4x4> OffLineProgrammingSimulationMainWindow::DH_to_Matrix(double a, double alpha, double d, double theta)
{
    double cth = std::cos(theta), sth = std::sin(theta);
    double cal = std::cos(alpha), sal = std::sin(alpha);

    // 矩阵按行填
    vtkSmartPointer<vtkMatrix4x4> M = vtkSmartPointer<vtkMatrix4x4>::New();
    M->Identity();

    // 第一行
    M->SetElement(0, 0,  cth);
    M->SetElement(0, 1, -sth * cal);
    M->SetElement(0, 2,  sth * sal);
    M->SetElement(0, 3,  a * cth);

    // 第二行
    M->SetElement(1, 0,  sth);
    M->SetElement(1, 1,  cth * cal);
    M->SetElement(1, 2, -cth * sal);
    M->SetElement(1, 3,  a * sth);

    // 第三行
    M->SetElement(2, 0,  0.0);
    M->SetElement(2, 1,  sal);
    M->SetElement(2, 2,  cal);
    M->SetElement(2, 3,  d);

    // 第四行
    M->SetElement(3, 0,  0.0);
    M->SetElement(3, 1,  0.0);
    M->SetElement(3, 2,  0.0);
    M->SetElement(3, 3,  1.0);

    return M;
}

void OffLineProgrammingSimulationMainWindow::CreateJointMesh(int jointIndex)
{
    const JointConfig& jointConfig = robotConfig->getJointConfig(jointIndex);

    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(jointConfig.stlFile.c_str());
    reader->Update();

    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(reader->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetDiffuse(0.8);
    if (jointConfig.color.size() >= 3) {
        actor->GetProperty()->SetDiffuseColor(jointConfig.color[0],
                                           jointConfig.color[1],
                                           jointConfig.color[2]);
    }
    actor->GetProperty()->SetSpecular(0.3);
    actor->GetProperty()->SetSpecularPower(60.0);
    actor->PickableOff();

    vtkSmartPointer<vtkAssembly> assembly = vtkSmartPointer<vtkAssembly>::New();
    assembly->AddPart(actor);

    if (jointAssemblies.size() <= jointIndex) {
        jointAssemblies.resize(jointIndex + 1);
    }
    jointAssemblies[jointIndex] = assembly;
}

vtkSmartPointer<vtkAssembly> OffLineProgrammingSimulationMainWindow::BuildKinematicChain()
{
    int jointCount = robotConfig->getJointCount();

    for (int i = 0; i < jointCount; ++i) {
        CreateJointMesh(i);
    }

    for (int i = 0; i < jointCount - 1; ++i) {
        jointAssemblies[i]->AddPart(jointAssemblies[i+1]);
    }

    return jointAssemblies[0];
}

void OffLineProgrammingSimulationMainWindow::UpdateJointTransforms(const Vector6d &anglesDeg)
{
    baseTransformMatrix_ << 1.0, 0.0, 0.0, baseAssemblyX_,
                            0.0, -1.0, 0.0, baseAssemblyY_,
                            0.0, 0.0, -1.0, baseAssemblyZ_,
                            0.0, 0.0, 0.0, 1.0;
    Matrix4d cumulativeTransform = baseTransformMatrix_;

    int jointCount = robotConfig->getJointCount();
    for (int i = 0; i < jointCount && i < 6; ++i) {
        const JointConfig& jointConfig = robotConfig->getJointConfig(i);

        double a = jointConfig.a;
        double alpha = jointConfig.alpha;
        double d = jointConfig.d;
        double theta = jointConfig.theta + anglesDeg(i);

        double ct = std::cos(theta);
        double st = std::sin(theta);
        double ca = std::cos(alpha);
        double sa = std::sin(alpha);

        Matrix4d T;
        T << ct, -st * ca,  st * sa, a * ct,
             st,  ct * ca, -ct * sa, a * st,
              0,       sa,       ca,      d,
              0,        0,        0,      1;

        cumulativeTransform = cumulativeTransform * T;

        switch(i) {
            case 0:
                link1TransformMatrix_ = cumulativeTransform;
                break;
            case 1:
                link2TransformMatrix_ = cumulativeTransform;
                break;
            case 2:
                link3TransformMatrix_ = cumulativeTransform;
                break;
            case 3:
                link4TransformMatrix_ = cumulativeTransform;
                break;
            case 4:
                link5TransformMatrix_ = cumulativeTransform;
                break;
            case 5:
                link6TransformMatrix_ = cumulativeTransform;
                Eigen::Matrix4d toolTransformMatrix = robotConfig->getToolTransform();

                toolTransformMatrix_ = cumulativeTransform * toolTransformMatrix;

                updateCustomToolAxes();
            break;
        }

        vtkSmartPointer<vtkMatrix4x4> M = DH_to_Matrix(jointConfig.a, jointConfig.alpha, jointConfig.d, jointConfig.theta + anglesDeg(i));

        vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
        t->SetMatrix(M);
        if (i < jointAssemblies.size()) {
            jointAssemblies[i]->SetUserTransform(t);
        }
    }
}

void OffLineProgrammingSimulationMainWindow::loadingRobot()
{
    if (!robotConfig) {
        qDebug() << "Robot config is not initialized!";
        return;
    }

    // 清空之前的数据
    jointAssemblies.clear();

    vtkSmartPointer<vtkAssembly> robotRoot = BuildKinematicChain();

    std::string toolPath = robotConfig->getToolStlPath();
    vtkSmartPointer<vtkActor> toolActor = nullptr;
    vtkSmartPointer<vtkAssembly> toolAssembly = vtkSmartPointer<vtkAssembly>::New();
    if (!toolPath.empty()) {
        vtkSmartPointer<vtkSTLReader> toolReader = vtkSmartPointer<vtkSTLReader>::New();
        toolReader->SetFileName(toolPath.c_str());
        toolReader->Update();
        vtkSmartPointer<vtkPolyDataMapper> toolMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        toolMapper->SetInputConnection(toolReader->GetOutputPort());

        toolActor = vtkSmartPointer<vtkActor>::New();
        toolActor->SetMapper(toolMapper);

        toolAssembly->AddPart(toolActor);

        if (!jointAssemblies.empty()) {
            jointAssemblies.back()->AddPart(toolAssembly);
        }

        vtkSmartPointer<vtkTransform> toolTransform = vtkSmartPointer<vtkTransform>::New();

        Eigen::Matrix4d toolTransformMatrix = robotConfig->getToolTransform();

        vtkSmartPointer<vtkMatrix4x4> vtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                vtkMatrix->SetElement(row, col, toolTransformMatrix(row, col));
            }
        }

        toolTransform->SetMatrix(vtkMatrix);
        toolAssembly->SetUserTransform(toolTransform);
    }

    createCustomToolAxes();

    std::string cameraPath = robotConfig->getCameraStlPath();
    vtkSmartPointer<vtkActor> cameraActor = nullptr;
    vtkSmartPointer<vtkAssembly> partAssembly = vtkSmartPointer<vtkAssembly>::New();
    if (!cameraPath.empty()) {
        vtkSmartPointer<vtkSTLReader> partReader = vtkSmartPointer<vtkSTLReader>::New();
        partReader->SetFileName(cameraPath.c_str());
        partReader->Update();
        vtkSmartPointer<vtkPolyDataMapper> partMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        partMapper->SetInputConnection(partReader->GetOutputPort());

        cameraActor = vtkSmartPointer<vtkActor>::New();
        cameraActor->SetMapper(partMapper);

        partAssembly->AddPart(cameraActor);

        toolAssembly->AddPart(partAssembly);

        vtkSmartPointer<vtkTransform> partTransform = vtkSmartPointer<vtkTransform>::New();

        Eigen::Matrix4d cameraTransformMatrix = robotConfig->getCameraTransform();

        vtkSmartPointer<vtkMatrix4x4> partVtkMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                partVtkMatrix->SetElement(row, col, cameraTransformMatrix(row, col));
            }
        }

        partTransform->SetMatrix(partVtkMatrix);
        partAssembly->SetUserTransform(partTransform);
    }

    vtkSmartPointer<vtkActor> baseActor = nullptr;
    std::string basePath = robotConfig->getBaseStlPath();
    if (!basePath.empty()) {
        vtkSmartPointer<vtkSTLReader> baseReader = vtkSmartPointer<vtkSTLReader>::New();
        baseReader->SetFileName(basePath.c_str());
        baseReader->Update();

        vtkSmartPointer<vtkPolyDataMapper> baseMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        baseMapper->SetInputConnection(baseReader->GetOutputPort());

        baseActor = vtkSmartPointer<vtkActor>::New();
        baseActor->SetMapper(baseMapper);
        baseActor->GetProperty()->SetDiffuse(0.8);
        baseActor->GetProperty()->SetSpecular(0.3);
        baseActor->GetProperty()->SetSpecularPower(60.0);
        baseActor->GetProperty()->SetColor(0.8, 0.8, 0.8);
    }

    baseAssembly = vtkSmartPointer<vtkAssembly>::New();

    if (baseActor) {
        baseAssembly->AddPart(baseActor);
    }

    if (robotRoot) {
        baseAssembly->AddPart(robotRoot);
    }

    renderer->AddActor(baseAssembly);
    renderer2->AddActor(baseAssembly);

    vtkSmartPointer<vtkTransform> baseTransform = vtkSmartPointer<vtkTransform>::New();
    baseTransform->Translate(baseAssemblyX_, 0, 0);
    baseTransform->Translate(0, baseAssemblyY_, 0);
    baseTransform->Translate(0, 0, baseAssemblyZ_);
    baseTransform->RotateX(180);
    baseAssembly->SetUserTransform(baseTransform);

    Vector6d angles;
    angles << 0, 0, 0, 0, 0, 0;
    UpdateJointTransforms(angles);

    baseAxes = vtkSmartPointer<vtkAxesActor>::New();
    baseAxes->PickableOff();

    baseAxes->SetTotalLength(100.0, 100.0, 100.0);
    baseAxes->SetShaftTypeToLine();
    baseAxes->SetConeResolution(30);
    baseAxes->SetCylinderResolution(100);

    baseAxes->SetXAxisLabelText("X");
    baseAxes->SetYAxisLabelText("Y");
    baseAxes->SetZAxisLabelText("Z");
    baseAxes->AxisLabelsOn();
    baseAxes->SetUserTransform(baseTransform);

    renderer->AddActor(baseAxes);
    renderer2->AddActor(baseAxes);

    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::calculateHanju()
{
    if (!all_coordinate_systems_B_.empty()){
        all_coordinate_systems_B_.clear();
    }

    for (const auto& edge : selected_edgeInfo) {
    // 判断是否为曲边
    if (edge.is_arc_or_spline && !edge.sample_points.empty()) {
        // 找到Z值最大的点的索引
        size_t max_z_idx = 0;
        double max_z = edge.sample_points[0].z;
        for (size_t i = 1; i < edge.sample_points.size(); ++i) {
            if (edge.sample_points[i].z > max_z) {
                max_z = edge.sample_points[i].z;
                max_z_idx = i;
            }
        }

        // 1. 从Z最大点向起点方向
        std::vector<CoordinateSystem> csB_to_start;
        for (int i = static_cast<int>(max_z_idx); i >= 0; --i) {
            const Point3D& point = edge.sample_points[i];
            const auto& normal_tuple = edge.normals[i];
            const Normal3D& tangent = -1 * std::get<0>(normal_tuple);
            const Normal3D& binormal = -1 * std::get<1>(normal_tuple);
            const Normal3D& normal = std::get<2>(normal_tuple);
            CoordinateSystem csB = createCoordinate(point, tangent, binormal, normal, 135.0, 180.0);
            csB_to_start.push_back(csB);
        }
        // 2. 从Z最大点向终点方向
        std::vector<CoordinateSystem> csB_to_end;
        for (size_t i = max_z_idx; i < edge.sample_points.size(); ++i) {
            const Point3D& point = edge.sample_points[i];
            const auto& normal_tuple = edge.normals[i];
            const Normal3D& tangent = std::get<0>(normal_tuple);
            const Normal3D& binormal = std::get<1>(normal_tuple);
            const Normal3D& normal = std::get<2>(normal_tuple);
            CoordinateSystem csB = createCoordinate(point, tangent, binormal, normal, 135.0, 0.0);
            csB_to_end.push_back(csB);
        }
        // 存储两组
        if (!csB_to_start.empty()) all_coordinate_systems_B_.push_back(csB_to_start);
        if (!csB_to_end.empty()) all_coordinate_systems_B_.push_back(csB_to_end);
    } else {
        // 非曲边，按原逻辑
        std::vector<CoordinateSystem> coordinate_systems_B;
        double z_start = edge.sample_points.empty() ? 0.0 : edge.sample_points.front().z;
        double z_end = edge.sample_points.empty() ? 0.0 : edge.sample_points.back().z;
        // 设定阈值，比如20mm
        double z_threshold = 20.0;
        if (z_end - z_start > z_threshold) {
            // 正向存储（起点到终点）
            for (int point_idx = static_cast<int>(edge.sample_points.size()) - 1; point_idx >= 0; --point_idx) {
                const Point3D& point = edge.sample_points[point_idx];
                const auto& normal_tuple = edge.normals[point_idx];
                const Normal3D& tangent = - 1 * std::get<0>(normal_tuple);
                const Normal3D& binormal = - 1 * std::get<1>(normal_tuple);
                const Normal3D& normal = std::get<2>(normal_tuple);
                CoordinateSystem csB = createCoordinate(point, tangent, binormal, normal, 135.0, -90.0);
                coordinate_systems_B.push_back(csB);
            }
        } else if (z_start - z_end > z_threshold) {
            // 反向存储（终点到起点）
            for (size_t point_idx = 0; point_idx < edge.sample_points.size(); ++point_idx) {
                const Point3D& point = edge.sample_points[point_idx];
                const auto& normal_tuple = edge.normals[point_idx];
                const Normal3D& tangent = std::get<0>(normal_tuple);
                const Normal3D& binormal = std::get<1>(normal_tuple);
                const Normal3D& normal = std::get<2>(normal_tuple);
                CoordinateSystem csB = createCoordinate(point, tangent, binormal, normal, 135.0, -90.0);
                coordinate_systems_B.push_back(csB);
            }
        } else {
            // Z值变化不大，默认正向
            for (size_t point_idx = 0; point_idx < edge.sample_points.size(); ++point_idx) {
                const Point3D& point = edge.sample_points[point_idx];
                const auto& normal_tuple = edge.normals[point_idx];
                const Normal3D& tangent = std::get<0>(normal_tuple);
                const Normal3D& binormal = std::get<1>(normal_tuple);
                const Normal3D& normal = std::get<2>(normal_tuple);
                CoordinateSystem csB = createCoordinate(point, tangent, binormal, normal, 90.0, 0.0);
                coordinate_systems_B.push_back(csB);
            }
        }
        if (!coordinate_systems_B.empty()) all_coordinate_systems_B_.push_back(coordinate_systems_B);
    }
    }
}

void OffLineProgrammingSimulationMainWindow::SetJointAnglesDeg(const Vector6d &anglesDeg, double baseX, double baseY, double baseZ, MotionMode mode, MotionTaskType taskType, const Matrix4d* projectorPose, const std::array<Vector3d, 5>* frustumPoints, const std::vector<Vector3d>* coveredVertices, const QString& planningInfo)
{
    MotionTarget target(anglesDeg, baseX, baseY, baseZ, mode, taskType, projectorPose, frustumPoints, coveredVertices, planningInfo);
    // 将运动请求加入队列
    motionQueue_.push(target);

    // 如果当前没有在处理队列，开始处理
    if (!isProcessingQueue_ && !smoothMotionTimer_->isActive()) {
        processNextMotion();
    }
}

void OffLineProgrammingSimulationMainWindow::onSmoothMotionTimer()
{
    if (isPaused_) {
        return;
    }

    if (interpolationStep_ >= totalInterpolationSteps_) {
        smoothMotionTimer_->stop();
        interpolationStep_ = 0;
        startAnglesDeg_ = targetAnglesDeg_;

        // 根据运动模式更新基座位置
        if (currentMotionMode_ == MotionMode::JOINT_AND_BASE) {
            baseAssemblyX_ = targetBaseX_;
            baseAssemblyY_ = targetBaseY_;
            baseAssemblyZ_ = targetBaseZ_;
        }

        processNextMotion();
        return;
    }

    double t = static_cast<double>(interpolationStep_) / totalInterpolationSteps_;

    // 使用更平滑的缓动函数
    t = t * t * t * (t * (t * 6 - 15) + 10); // 五次方缓动

    Vector6d currentAngles = startAnglesDeg_ + t * (targetAnglesDeg_ - startAnglesDeg_);

    if (currentMotionMode_ == MotionMode::JOINT_AND_BASE) {
        baseAssemblyX_ = startBaseX_ + t * (targetBaseX_ - startBaseX_);
        baseAssemblyY_ = startBaseY_ + t * (targetBaseY_ - startBaseY_);
        baseAssemblyZ_ = startBaseZ_ + t * (targetBaseZ_ - startBaseZ_);
    }

    UpdateJointTransforms(currentAngles);
    updateBaseAssemblyTransform();

    recordToolPosition();
    updateStructuredLightVisualization();

    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();

    interpolationStep_++;
}

void OffLineProgrammingSimulationMainWindow::initQSliders()
{
    ui->horizontalSlider->setRange(-5000, 5000);
    ui->horizontalSlider->setValue(-1000);
    ui->horizontalSlider_2->setRange(-1000, 2000);
    ui->horizontalSlider_2->setValue(1000);
    ui->horizontalSlider_3->setRange(500, 2500);
    ui->horizontalSlider_3->setValue(2000);

    connect(ui->horizontalSlider, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_valueChanged);
    connect(ui->horizontalSlider_2, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_2_valueChanged);
    connect(ui->horizontalSlider_3, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_3_valueChanged);

    updateTransformMatrixLabel();

    ui->horizontalSlider_4->setRange(-170, 170);
    ui->horizontalSlider_4->setValue(0);

    // 轴2: 关节角度范围 -150° 到 90°
    ui->horizontalSlider_5->setRange(-150, 90);
    ui->horizontalSlider_5->setValue(0);

    // 轴3: 关节角度范围 -85° 到 150°
    ui->horizontalSlider_6->setRange(-85, 150);
    ui->horizontalSlider_6->setValue(0);

    // 轴4: 关节角度范围 -180° 到 180°
    ui->horizontalSlider_7->setRange(-180, 180);
    ui->horizontalSlider_7->setValue(0);

    // 轴5: 关节角度范围 -150° 到 150°
    ui->horizontalSlider_8->setRange(-150, 150);
    ui->horizontalSlider_8->setValue(0);

    // 轴6: 关节角度范围 -180° 到 180°
    ui->horizontalSlider_9->setRange(-180, 180);
    ui->horizontalSlider_9->setValue(0);
    connect(ui->horizontalSlider_4, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_4_valueChanged);
    connect(ui->horizontalSlider_5, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_5_valueChanged);
    connect(ui->horizontalSlider_6, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_6_valueChanged);
    connect(ui->horizontalSlider_7, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_7_valueChanged);
    connect(ui->horizontalSlider_8, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_8_valueChanged);
    connect(ui->horizontalSlider_9, &QSlider::valueChanged, this, &OffLineProgrammingSimulationMainWindow::on_horizontalSlider_9_valueChanged);
    updateJointAnglesLabel();
}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_valueChanged(int value)
{
    baseAssemblyX_ = value;
    updateBaseAssemblyTransform();
    updateTransformMatrixLabel();
    performCollisionDetection();
}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_2_valueChanged(int value)
{
    baseAssemblyY_ = value;
    updateBaseAssemblyTransform();
    updateTransformMatrixLabel();
    performCollisionDetection();
}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_3_valueChanged(int value)
{
    baseAssemblyZ_ = value;
    updateBaseAssemblyTransform();
    updateTransformMatrixLabel();
    performCollisionDetection();
}

void OffLineProgrammingSimulationMainWindow::updateBaseAssemblyTransform()
{
    if (!baseAssembly) {
        return;
    }

//    UpdateJointTransforms(startAnglesDeg_);

    vtkSmartPointer<vtkTransform> newTransform = vtkSmartPointer<vtkTransform>::New();

    newTransform->Translate(baseAssemblyX_, baseAssemblyY_, baseAssemblyZ_);
    newTransform->RotateX(180);

    baseAssembly->SetUserTransform(newTransform);
    baseAxes->SetUserTransform(newTransform);

    updateTransformMatrix();

    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::updateTransformMatrix()
{
    if (!baseAssembly) {
        return;
    }

    baseTransformMatrix_ << 1.0, 0.0, 0.0, baseAssemblyX_,
                            0.0, -1.0, 0.0, baseAssemblyY_,
                            0.0, 0.0, -1.0, baseAssemblyZ_,
                            0.0, 0.0, 0.0, 1.0;
}

void OffLineProgrammingSimulationMainWindow::updateTransformMatrixLabel()
{
    ui->label_4->setText(QString::number(baseAssemblyX_, 'f', 1) + " mm");

    ui->label_5->setText(QString::number(baseAssemblyY_, 'f', 1) + " mm");

    ui->label_6->setText(QString::number(baseAssemblyZ_, 'f', 1) + " mm");
}

void OffLineProgrammingSimulationMainWindow::processNextMotion()
{
    // 如果队列为空，停止处理
    if (motionQueue_.empty()) {
        isProcessingQueue_ = false;
        isReplayingTrajectory_ = false;
        isPaused_ = false;
        currentMotionTaskType_ = MotionTaskType::TRANSITION;
        currentHasProjectorPose_ = false;
        currentHasStructuredLightGeometry_ = false;
        currentCoveredVertices_.clear();
        currentFrustumPoints_.fill(Vector3d::Zero());
        setStructuredLightActorsVisible(false);
        if (ui && ui->toolButton_13) {
            ui->toolButton_13->setText(u8"演示暂停");
        }
        return;
    }
    // 标记为正在处理队列
    isProcessingQueue_ = true;

    // 从队列中取出下一个运动目标
    MotionTarget nextTarget = motionQueue_.front();
    motionQueue_.pop();

    // 设置当前运动模式
    currentMotionMode_ = nextTarget.motionMode;
    currentMotionTaskType_ = nextTarget.taskType;
    appendPlanningInfo(nextTarget.planningInfo);
    currentHasProjectorPose_ = nextTarget.hasProjectorPose;
    if (currentHasProjectorPose_) {
        currentProjectorPose_ = nextTarget.projectorPose;
    }
    currentHasStructuredLightGeometry_ = nextTarget.hasStructuredLightGeometry;
    currentFrustumPoints_ = nextTarget.frustumPoints;
    currentCoveredVertices_ = nextTarget.coveredVertices;
    if (currentMotionTaskType_ == MotionTaskType::CAMERA_CAPTURE) {
        if (!currentHasProjectorPose_) {
            currentProjectorPose_ = toolTransformMatrix_;
            currentHasProjectorPose_ = true;
        }
        if (currentHasStructuredLightGeometry_) {
            applyHalfFrustumActors(currentFrustumPoints_, currentProjectorPose_);
            applyStructuredLightCoveredGeometry(currentCoveredVertices_);
            setStructuredLightActorsVisible(true);
        } else {
            std::array<Vector3d, 5> frustumPoints;
            buildHalfFrustumPoints(currentProjectorPose_, frustumPoints);
            applyHalfFrustumActors(frustumPoints, currentProjectorPose_);
            applyStructuredLightCoveredGeometry({});
            setStructuredLightActorsVisible(true);
        }
        if (renderer && renderer->GetRenderWindow()) {
            renderer->GetRenderWindow()->Render();
        }
        if (renderer2 && renderer2->GetRenderWindow()) {
            renderer2->GetRenderWindow()->Render();
        }
    } else {
        setStructuredLightActorsVisible(false);
    }

    // 设置运动参数
    targetAnglesDeg_ = nextTarget.jointAngles;
    interpolationStep_ = 0;

    if (currentMotionMode_ == MotionMode::JOINT_AND_BASE) {
        targetBaseX_ = nextTarget.baseX;
        targetBaseY_ = nextTarget.baseY;
        targetBaseZ_ = nextTarget.baseZ;
        startBaseX_ = baseAssemblyX_;
        startBaseY_ = baseAssemblyY_;
        startBaseZ_ = baseAssemblyZ_;
    } else {
        // 关节运动模式下，保持基座位置不变
        targetBaseX_ = baseAssemblyX_;
        targetBaseY_ = baseAssemblyY_;
        targetBaseZ_ = baseAssemblyZ_;
        startBaseX_ = baseAssemblyX_;
        startBaseY_ = baseAssemblyY_;
        startBaseZ_ = baseAssemblyZ_;
    }

    // 更精细的角度自适应调整
    Vector6d angleDiff = (targetAnglesDeg_ - startAnglesDeg_).cwiseAbs();

    double weightedMaxAngleDiff = angleDiff.maxCoeff();

    // 根据运动模式计算基座距离
    double baseDistance = 0.0;
    if (currentMotionMode_ == MotionMode::JOINT_AND_BASE) {
        baseDistance = std::sqrt(
            std::pow(targetBaseX_ - startBaseX_, 2) +
            std::pow(targetBaseY_ - startBaseY_, 2) +
            std::pow(targetBaseZ_ - startBaseZ_, 2)
        );
    }

    const int baseSteps = 20;
    const int maxSteps = 500;

    double angleFactor = std::min(weightedMaxAngleDiff / 100.0, 1.0);
    int angleSteps = static_cast<int>(angleFactor * 600);

    int baseStepsContribution = 0;
    if (currentMotionMode_ == MotionMode::JOINT_AND_BASE) {
        double baseFactor = std::min(baseDistance / 500.0, 1.0);
        baseStepsContribution = static_cast<int>(baseFactor * 200);
    }

    totalInterpolationSteps_ = std::max(baseSteps, std::min(baseSteps + angleSteps + baseStepsContribution, maxSteps));
    if (currentMotionTaskType_ == MotionTaskType::CAMERA_CAPTURE) {
        totalInterpolationSteps_ = std::max(totalInterpolationSteps_, 120);
    }


    // 启动定时器
    smoothMotionTimer_->start(17);
}

void OffLineProgrammingSimulationMainWindow::updateJointAnglesLabel()
{
    ui->label_13->setText(QString::number(startAnglesDeg_[0] * 180.0 / M_PI, 'f', 1) + "°");
    ui->label_14->setText(QString::number(startAnglesDeg_[1] * 180.0 / M_PI, 'f', 1) + "°");
    ui->label_15->setText(QString::number(startAnglesDeg_[2] * 180.0 / M_PI, 'f', 1) + "°");
    ui->label_16->setText(QString::number(startAnglesDeg_[3] * 180.0 / M_PI, 'f', 1) + "°");
    ui->label_17->setText(QString::number(startAnglesDeg_[4] * 180.0 / M_PI, 'f', 1) + "°");
    ui->label_18->setText(QString::number(startAnglesDeg_[5] * 180.0 / M_PI, 'f', 1) + "°");
}

void OffLineProgrammingSimulationMainWindow::performCollisionDetection()
{
    std::vector<Eigen::Matrix4d> jointTransforms;
    jointTransforms.reserve(7);
    jointTransforms.push_back(baseTransformMatrix_);
    jointTransforms.push_back(link1TransformMatrix_);
    jointTransforms.push_back(link2TransformMatrix_);
    jointTransforms.push_back(link3TransformMatrix_);
    jointTransforms.push_back(link4TransformMatrix_);
    jointTransforms.push_back(link5TransformMatrix_);
    jointTransforms.push_back(toolTransformMatrix_);

    collisionDetector->setEnvironmentMode(EnvDetectMask::STEP_ONLY);
    collisionDetector->updateRobotPose(jointTransforms);

    bool collision = collisionDetector->checkCollision();

     //显示碰撞检测结果
    if (collision) {
            ui->statusbar->showMessage("警告：检测到碰撞！", 3000); // 显示3秒
        } else {
            ui->statusbar->showMessage("安全：无碰撞", 1000); // 显示1秒
    }
}

void OffLineProgrammingSimulationMainWindow::recordToolPosition()
{
    // 从toolTransformMatrix_中提取焊枪位置
    Point3D toolPosition;
    toolPosition.x = toolTransformMatrix_(0, 3);
    toolPosition.y = toolTransformMatrix_(1, 3);
    toolPosition.z = toolTransformMatrix_(2, 3);

    // 添加焊枪位置到路径点集合
    toolPathPoints_.push_back(toolPosition);

    // 当路径点达到2个时，自动创建可视化（但保持隐藏状态）
    if (toolPathPoints_.size() >= 2 && !toolPathActor_) {
        createToolPathVisualization();
    }

    // 如果可视化已存在，更新它
    if (toolPathActor_) {
        updateToolPathVisualization();
    }
}

void OffLineProgrammingSimulationMainWindow::createToolPathVisualization()
{
    // 如果路径点不足，直接返回
    if (toolPathPoints_.size() < 2) {
        std::cout << "警告：路径点数量不足，无法创建路径可视化" << std::endl;
        return;
    }

    // 如果已经存在路径可视化，先清除
    if (toolPathActor_) {
        renderer->RemoveActor(toolPathActor_);
        renderer2->RemoveActor(toolPathActor_);
        toolPathActor_ = nullptr;
    }

    // 创建点集
    auto points = vtkSmartPointer<vtkPoints>::New();

    // 添加路径点到点集
    for (const auto& point : toolPathPoints_) {
        points->InsertNextPoint(point.x, point.y, point.z);
    }

    // 创建多段线
    auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
    for (int i = 0; i < toolPathPoints_.size(); ++i) {
        polyLine->GetPointIds()->InsertNextId(i);
    }

    auto lines = vtkSmartPointer<vtkCellArray>::New();
    lines->InsertNextCell(polyLine);

    // 创建多边形数据
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建管道过滤器使线条更明显
    auto tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    tubeFilter->SetInputData(polyData);
    tubeFilter->SetRadius(2.0);  // 管道半径
    tubeFilter->SetNumberOfSides(8);  // 管道边数

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(tubeFilter->GetOutputPort());

    // 创建actor
    toolPathActor_ = vtkSmartPointer<vtkActor>::New();
    toolPathActor_->SetMapper(mapper);

    // 设置路径颜色（蓝色）
    toolPathActor_->GetProperty()->SetColor(1.0, 0.0, 0.0);
    toolPathActor_->GetProperty()->SetLineWidth(3.0);

    // 添加到渲染器，但初始设置为隐藏状态
    renderer->AddActor(toolPathActor_);
    renderer2->AddActor(toolPathActor_);

    // 初始设置为隐藏状态，等待用户点击显示按钮
    toolPathActor_->SetVisibility(false);
}

void OffLineProgrammingSimulationMainWindow::updateToolPathVisualization()
{
    // 如果路径点不足，直接返回
    if (toolPathPoints_.size() < 2) {
        return;
    }

    // 如果路径可视化不存在，创建它
    if (!toolPathActor_) {
        createToolPathVisualization();
        return;
    }

    // 创建点集
    auto points = vtkSmartPointer<vtkPoints>::New();

    // 添加路径点到点集
    for (const auto& point : toolPathPoints_) {
        points->InsertNextPoint(point.x, point.y, point.z);
    }

    // 创建多段线
    auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
    for (int i = 0; i < toolPathPoints_.size(); ++i) {
        polyLine->GetPointIds()->InsertNextId(i);
    }

    auto lines = vtkSmartPointer<vtkCellArray>::New();
    lines->InsertNextCell(polyLine);

    // 创建多边形数据
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建管道过滤器
    auto tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    tubeFilter->SetInputData(polyData);
    tubeFilter->SetRadius(2.0);
    tubeFilter->SetNumberOfSides(8);

    // 更新映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(tubeFilter->GetOutputPort());

    toolPathActor_->SetMapper(mapper);

    // 重新渲染
    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::clearToolPath()
{
    toolPathPoints_.clear();

    if (toolPathActor_) {
        renderer->RemoveActor(toolPathActor_);
        renderer2->RemoveActor(toolPathActor_);
        toolPathActor_ = nullptr;
    }

    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::toggleToolPathVisibility(bool visible)
{
    showToolPath_ = visible;

    if (toolPathActor_) {
        toolPathActor_->SetVisibility(visible);
    }

    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::addStructuredLightActorsToRenderers()
{
    if (!halfFrustumActor_) {
        halfFrustumActor_ = vtkSmartPointer<vtkActor>::New();
        halfFrustumActor_->GetProperty()->SetColor(0.0, 0.8, 1.0);
        halfFrustumActor_->GetProperty()->SetOpacity(0.15);
        halfFrustumActor_->GetProperty()->SetRepresentationToSurface();
        halfFrustumActor_->GetProperty()->SetLighting(false);
    }

    if (!halfFrustumLineActor_) {
        halfFrustumLineActor_ = vtkSmartPointer<vtkActor>::New();
        halfFrustumLineActor_->GetProperty()->SetColor(0.0, 1.0, 1.0);
        halfFrustumLineActor_->GetProperty()->SetLineWidth(2.0);
        halfFrustumLineActor_->GetProperty()->SetLighting(false);
    }

    if (!structuredLightCoveredActor_) {
        structuredLightCoveredActor_ = vtkSmartPointer<vtkActor>::New();
        structuredLightCoveredActor_->GetProperty()->SetColor(1.0, 0.85, 0.0);
        structuredLightCoveredActor_->GetProperty()->SetOpacity(1.0);
        structuredLightCoveredActor_->GetProperty()->SetLighting(false);
    }

    if (!projectorPointActor_) {
        projectorPointActor_ = vtkSmartPointer<vtkActor>::New();
        projectorPointActor_->GetProperty()->SetColor(1.0, 0.2, 0.0);
        projectorPointActor_->GetProperty()->SetLighting(false);
    }

    if (!projectorDirectionActor_) {
        projectorDirectionActor_ = vtkSmartPointer<vtkActor>::New();
        projectorDirectionActor_->GetProperty()->SetColor(1.0, 0.2, 0.0);
        projectorDirectionActor_->GetProperty()->SetLineWidth(3.0);
        projectorDirectionActor_->GetProperty()->SetLighting(false);
    }

    if (!structuredLightActorsAdded_) {
        bool added = false;
        if (renderer) {
            renderer->AddActor(halfFrustumActor_);
            renderer->AddActor(halfFrustumLineActor_);
            renderer->AddActor(structuredLightCoveredActor_);
            renderer->AddActor(projectorPointActor_);
            renderer->AddActor(projectorDirectionActor_);
            added = true;
        }

        if (renderer2) {
            renderer2->AddActor(halfFrustumActor_);
            renderer2->AddActor(halfFrustumLineActor_);
            renderer2->AddActor(structuredLightCoveredActor_);
            renderer2->AddActor(projectorPointActor_);
            renderer2->AddActor(projectorDirectionActor_);
            added = true;
        }

        structuredLightActorsAdded_ = added;
    }
    setStructuredLightActorsVisible(false);
}

void OffLineProgrammingSimulationMainWindow::setStructuredLightActorsVisible(bool visible)
{
    if (halfFrustumActor_) halfFrustumActor_->SetVisibility(visible);
    if (halfFrustumLineActor_) halfFrustumLineActor_->SetVisibility(visible);
    if (structuredLightCoveredActor_) structuredLightCoveredActor_->SetVisibility(visible);
    if (projectorPointActor_) projectorPointActor_->SetVisibility(visible);
    if (projectorDirectionActor_) projectorDirectionActor_->SetVisibility(visible);
}

void OffLineProgrammingSimulationMainWindow::updateStructuredLightVisualization()
{
    if (!isCameraCaptureMotion()) {
        setStructuredLightActorsVisible(false);
        return;
    }

    Matrix4d projectorPose = currentHasProjectorPose_ ? currentProjectorPose_ : toolTransformMatrix_;
    bool poseChanged = !hasLastStructuredLightProjectorPose_ ||
                       (projectorPose - lastStructuredLightProjectorPose_).cwiseAbs().maxCoeff() > 1e-6;
    if (poseChanged) {
        updateHalfFrustumActors(projectorPose);
        if (currentHasStructuredLightGeometry_) {
            applyStructuredLightCoveredGeometry(currentCoveredVertices_);
        } else {
            applyStructuredLightCoveredGeometry({});
        }
        lastStructuredLightProjectorPose_ = projectorPose;
        hasLastStructuredLightProjectorPose_ = true;
    }
    setStructuredLightActorsVisible(true);
}

void OffLineProgrammingSimulationMainWindow::buildHalfFrustumPoints(const Matrix4d& projectorPose, std::array<Vector3d, 5>& frustumPoints) const
{
    Vector3d position = projectorPose.block<3,1>(0,3);
    Vector3d right = projectorPose.block<3,1>(0,0).normalized();
    Vector3d up = projectorPose.block<3,1>(0,1).normalized();
    Vector3d forward = projectorPose.block<3,1>(0,2).normalized();

    double fovYRad = vtkMath::RadiansFromDegrees(structuredLightFovY_);
    double halfHeight = structuredLightFarDist_ * std::tan(fovYRad * 0.5);
    double halfWidth = halfHeight * structuredLightAspectRatio_;

    Vector3d farCenter = position + forward * structuredLightFarDist_;
    frustumPoints[0] = position;
    frustumPoints[1] = farCenter - right * halfWidth;
    frustumPoints[2] = farCenter + right * halfWidth;
    frustumPoints[3] = farCenter + right * halfWidth + up * halfHeight;
    frustumPoints[4] = farCenter - right * halfWidth + up * halfHeight;
}

void OffLineProgrammingSimulationMainWindow::applyHalfFrustumActors(const std::array<Vector3d, 5>& frustumPoints, const Matrix4d& projectorPose)
{
    addStructuredLightActorsToRenderers();

    Vector3d position = projectorPose.block<3,1>(0,3);
    Vector3d up = projectorPose.block<3,1>(0,1).normalized();
    Vector3d forward = projectorPose.block<3,1>(0,2).normalized();

    if (!structuredLightProjectorCamera_) {
        structuredLightProjectorCamera_ = vtkSmartPointer<vtkCamera>::New();
    }
    structuredLightProjectorCamera_->SetPosition(position.x(), position.y(), position.z());
    Vector3d focalPoint = position + forward;
    structuredLightProjectorCamera_->SetFocalPoint(focalPoint.x(), focalPoint.y(), focalPoint.z());
    structuredLightProjectorCamera_->SetViewUp(up.x(), up.y(), up.z());
    structuredLightProjectorCamera_->SetViewAngle(structuredLightFovY_);
    structuredLightProjectorCamera_->SetClippingRange(structuredLightNearDist_, structuredLightFarDist_);

    auto points = vtkSmartPointer<vtkPoints>::New();
    for (const auto& point : frustumPoints) {
        points->InsertNextPoint(point.x(), point.y(), point.z());
    }

    auto polys = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType leftFace[3] = {0, 1, 4};
    vtkIdType rightFace[3] = {0, 2, 3};
    vtkIdType topFace[3] = {0, 4, 3};
    vtkIdType centerFace[3] = {0, 1, 2};
    vtkIdType farFace[4] = {1, 2, 3, 4};
    polys->InsertNextCell(3, leftFace);
    polys->InsertNextCell(3, rightFace);
    polys->InsertNextCell(3, topFace);
    polys->InsertNextCell(3, centerFace);
    polys->InsertNextCell(4, farFace);

    auto frustumPolyData = vtkSmartPointer<vtkPolyData>::New();
    frustumPolyData->SetPoints(points);
    frustumPolyData->SetPolys(polys);

    auto frustumMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    frustumMapper->SetInputData(frustumPolyData);
    halfFrustumActor_->SetMapper(frustumMapper);

    auto lineCells = vtkSmartPointer<vtkCellArray>::New();
    auto addLine = [&](vtkIdType p0, vtkIdType p1) {
        vtkIdType lineIds[2] = {p0, p1};
        lineCells->InsertNextCell(2, lineIds);
    };
    addLine(0, 1);
    addLine(0, 2);
    addLine(0, 3);
    addLine(0, 4);
    addLine(1, 2);
    addLine(2, 3);
    addLine(3, 4);
    addLine(4, 1);

    auto linePolyData = vtkSmartPointer<vtkPolyData>::New();
    linePolyData->SetPoints(points);
    linePolyData->SetLines(lineCells);

    auto lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    lineMapper->SetInputData(linePolyData);
    halfFrustumLineActor_->SetMapper(lineMapper);

    auto pointSource = vtkSmartPointer<vtkSphereSource>::New();
    pointSource->SetCenter(position.x(), position.y(), position.z());
    pointSource->SetRadius(8.0);
    pointSource->SetThetaResolution(16);
    pointSource->SetPhiResolution(16);

    auto pointMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    pointMapper->SetInputConnection(pointSource->GetOutputPort());
    projectorPointActor_->SetMapper(pointMapper);

    auto directionLine = vtkSmartPointer<vtkLineSource>::New();
    Vector3d directionEnd = position + forward * (structuredLightFarDist_ * 0.25);
    directionLine->SetPoint1(position.x(), position.y(), position.z());
    directionLine->SetPoint2(directionEnd.x(), directionEnd.y(), directionEnd.z());

    auto directionTube = vtkSmartPointer<vtkTubeFilter>::New();
    directionTube->SetInputConnection(directionLine->GetOutputPort());
    directionTube->SetRadius(2.0);
    directionTube->SetNumberOfSides(8);
    directionTube->CappingOn();

    auto directionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    directionMapper->SetInputConnection(directionTube->GetOutputPort());
    projectorDirectionActor_->SetMapper(directionMapper);
}

void OffLineProgrammingSimulationMainWindow::updateHalfFrustumActors(const Matrix4d& projectorPose)
{
    std::array<Vector3d, 5> frustumPoints;
    buildHalfFrustumPoints(projectorPose, frustumPoints);
    applyHalfFrustumActors(frustumPoints, projectorPose);
}

void OffLineProgrammingSimulationMainWindow::applyStructuredLightCoveredGeometry(const std::vector<Vector3d>& coveredVertices)
{
    auto coveredPoints = vtkSmartPointer<vtkPoints>::New();
    auto coveredPolys = vtkSmartPointer<vtkCellArray>::New();

    for (size_t i = 0; i + 2 < coveredVertices.size(); i += 3) {
        vtkIdType ids[3];
        ids[0] = coveredPoints->InsertNextPoint(coveredVertices[i + 0].x(), coveredVertices[i + 0].y(), coveredVertices[i + 0].z());
        ids[1] = coveredPoints->InsertNextPoint(coveredVertices[i + 1].x(), coveredVertices[i + 1].y(), coveredVertices[i + 1].z());
        ids[2] = coveredPoints->InsertNextPoint(coveredVertices[i + 2].x(), coveredVertices[i + 2].y(), coveredVertices[i + 2].z());
        coveredPolys->InsertNextCell(3, ids);
    }

    auto coveredPolyData = vtkSmartPointer<vtkPolyData>::New();
    coveredPolyData->SetPoints(coveredPoints);
    coveredPolyData->SetPolys(coveredPolys);

    auto coveredMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    coveredMapper->SetInputData(coveredPolyData);
    coveredMapper->SetResolveCoincidentTopologyToPolygonOffset();
    structuredLightCoveredActor_->SetMapper(coveredMapper);
}

bool OffLineProgrammingSimulationMainWindow::isPointInsideHalfFrustum(const Vector3d& point, const Vector3d& position, const Vector3d& right, const Vector3d& up, const Vector3d& forward) const
{
    Vector3d v = point - position;
    double z = v.dot(forward);
    if (z < structuredLightNearDist_ || z > structuredLightFarDist_) {
        return false;
    }

    double fovYRad = vtkMath::RadiansFromDegrees(structuredLightFovY_);
    double halfHeightAtZ = z * std::tan(fovYRad * 0.5);
    double halfWidthAtZ = halfHeightAtZ * structuredLightAspectRatio_;
    double x = v.dot(right);
    double y = v.dot(up);

    return y >= 0.0 &&
           y <= halfHeightAtZ &&
           x >= -halfWidthAtZ &&
           x <= halfWidthAtZ;
}

bool OffLineProgrammingSimulationMainWindow::doesTriangleAabbIntersectHalfFrustum(const Vector3d& bboxMin, const Vector3d& bboxMax, const std::array<Vector3d, 5>& frustumPoints) const
{
    Vector3d position = frustumPoints[0];
    Vector3d right = frustumPoints[2] - frustumPoints[1];
    Vector3d up = frustumPoints[4] - frustumPoints[1];
    Vector3d farCenter = (frustumPoints[1] + frustumPoints[2] + frustumPoints[3] + frustumPoints[4]) * 0.25;
    Vector3d forward = farCenter - position;
    if (right.norm() > 1e-9) {
        right.normalize();
    }
    if (up.norm() > 1e-9) {
        up.normalize();
    }
    if (forward.norm() > 1e-9) {
        forward.normalize();
    }

    for (int xi = 0; xi < 2; ++xi) {
        for (int yi = 0; yi < 2; ++yi) {
            for (int zi = 0; zi < 2; ++zi) {
                Vector3d corner(xi == 0 ? bboxMin.x() : bboxMax.x(),
                                yi == 0 ? bboxMin.y() : bboxMax.y(),
                                zi == 0 ? bboxMin.z() : bboxMax.z());
                if (isPointInsideHalfFrustum(corner, position, right, up, forward)) {
                    return true;
                }
            }
        }
    }

    for (const auto& point : frustumPoints) {
        if (point.x() >= bboxMin.x() && point.x() <= bboxMax.x() &&
            point.y() >= bboxMin.y() && point.y() <= bboxMax.y() &&
            point.z() >= bboxMin.z() && point.z() <= bboxMax.z()) {
            return true;
        }
    }

    Vector3d frustumMin = frustumPoints[0];
    Vector3d frustumMax = frustumPoints[0];
    for (const auto& point : frustumPoints) {
        frustumMin = frustumMin.cwiseMin(point);
        frustumMax = frustumMax.cwiseMax(point);
    }

    return !(bboxMax.x() < frustumMin.x() || bboxMin.x() > frustumMax.x() ||
             bboxMax.y() < frustumMin.y() || bboxMin.y() > frustumMax.y() ||
             bboxMax.z() < frustumMin.z() || bboxMin.z() > frustumMax.z());
}

void OffLineProgrammingSimulationMainWindow::buildStructuredLightCoveredVertices(const Matrix4d& projectorPose,
                                                                                 const std::array<Vector3d, 5>& frustumPoints,
                                                                                 std::vector<Vector3d>& coveredVertices) const
{
    coveredVertices.clear();
    if (!workpiecePolyData_) {
        return;
    }

    Vector3d position = projectorPose.block<3,1>(0,3);
    Vector3d right = projectorPose.block<3,1>(0,0).normalized();
    Vector3d up = projectorPose.block<3,1>(0,1).normalized();
    Vector3d forward = projectorPose.block<3,1>(0,2).normalized();
    coveredVertices.reserve(1024);
    std::vector<std::array<Vector3d, 3>> sampleTriangles;
    sampleTriangles.reserve(1024);
    std::vector<Vector3d> sampleNormals;
    sampleNormals.reserve(1024);

    for (vtkIdType cellId = 0; cellId < workpiecePolyData_->GetNumberOfCells(); ++cellId) {
        vtkCell* cell = workpiecePolyData_->GetCell(cellId);
        if (!cell || cell->GetNumberOfPoints() < 3) {
            continue;
        }

        double rawP0[3], rawP1[3], rawP2[3];
        workpiecePolyData_->GetPoint(cell->GetPointId(0), rawP0);
        workpiecePolyData_->GetPoint(cell->GetPointId(1), rawP1);
        workpiecePolyData_->GetPoint(cell->GetPointId(2), rawP2);

        Vector3d p0(rawP0[0], rawP0[1], rawP0[2]);
        Vector3d p1(rawP1[0], rawP1[1], rawP1[2]);
        Vector3d p2(rawP2[0], rawP2[1], rawP2[2]);
        Vector3d bboxMin = p0.cwiseMin(p1).cwiseMin(p2);
        Vector3d bboxMax = p0.cwiseMax(p1).cwiseMax(p2);
        if (!doesTriangleAabbIntersectHalfFrustum(bboxMin, bboxMax, frustumPoints)) {
            continue;
        }

        Vector3d normal = (p1 - p0).cross(p2 - p0);
        if (normal.norm() < 1e-9) {
            continue;
        }
        normal.normalize();

        double maxEdgeLength = std::max({
            (p1 - p0).norm(),
            (p2 - p1).norm(),
            (p0 - p2).norm()
        });
        if (maxEdgeLength <= structuredLightCoverageSampleStep_) {
            sampleTriangles.push_back({p0, p1, p2});
            sampleNormals.push_back(normal);
            continue;
        }

        int subdivisionCount = std::max(2, static_cast<int>(std::ceil(maxEdgeLength / structuredLightCoverageSampleStep_)));
        subdivisionCount = std::min(subdivisionCount, 300);

        auto pointOnTriangle = [&](int i, int j) {
            double u = static_cast<double>(i) / subdivisionCount;
            double v = static_cast<double>(j) / subdivisionCount;
            return p0 + (p1 - p0) * u + (p2 - p0) * v;
        };

        auto appendSampleTriangle = [&](const Vector3d& q0, const Vector3d& q1, const Vector3d& q2) {
            sampleTriangles.push_back({q0, q1, q2});
            sampleNormals.push_back(normal);
        };

        for (int i = 0; i < subdivisionCount; ++i) {
            for (int j = 0; j < subdivisionCount - i; ++j) {
                Vector3d q00 = pointOnTriangle(i, j);
                Vector3d q10 = pointOnTriangle(i + 1, j);
                Vector3d q01 = pointOnTriangle(i, j + 1);
                appendSampleTriangle(q00, q10, q01);

                if (i + j < subdivisionCount - 1) {
                    Vector3d q11 = pointOnTriangle(i + 1, j + 1);
                    appendSampleTriangle(q10, q11, q01);
                }
            }
        }
    }

    for (size_t i = 0; i < sampleTriangles.size(); ++i) {
        const auto& triangle = sampleTriangles[i];
        const Vector3d& normal = sampleNormals[i];
        if (!shouldShowCoveredSample(triangle[0], triangle[1], triangle[2], normal, position, right, up, forward)) {
            continue;
        }

        coveredVertices.push_back(triangle[0]);
        coveredVertices.push_back(triangle[1]);
        coveredVertices.push_back(triangle[2]);
    }
}

bool OffLineProgrammingSimulationMainWindow::shouldShowCoveredSample(const Vector3d& p0,
                                                                     const Vector3d& p1,
                                                                     const Vector3d& p2,
                                                                     const Vector3d& normal,
                                                                     const Vector3d& position,
                                                                     const Vector3d& right,
                                                                     const Vector3d& up,
                                                                     const Vector3d& forward) const
{
    Vector3d center = (p0 + p1 + p2) / 3.0;

    bool inside = isPointInsideHalfFrustum(center, position, right, up, forward) ||
                  isPointInsideHalfFrustum(p0, position, right, up, forward) ||
                  isPointInsideHalfFrustum(p1, position, right, up, forward) ||
                  isPointInsideHalfFrustum(p2, position, right, up, forward);
    if (!inside) {
        return false;
    }

    Vector3d viewDir = position - center;
    if (viewDir.norm() < 1e-9) {
        return false;
    }
    viewDir.normalize();

//    if (normal.dot(viewDir) <= 0.0) {
//        return false;
//    }

    return true;
}

void OffLineProgrammingSimulationMainWindow::updateStructuredLightCoverage(const Matrix4d& projectorPose)
{
    if (!workpiecePolyData_ || !structuredLightCoveredActor_) {
        return;
    }

    std::array<Vector3d, 5> frustumPoints;
    buildHalfFrustumPoints(projectorPose, frustumPoints);
    std::vector<Vector3d> coveredVertices;
    buildStructuredLightCoveredVertices(projectorPose, frustumPoints, coveredVertices);

    currentFrustumPoints_ = frustumPoints;
    currentCoveredVertices_ = coveredVertices;
    currentHasStructuredLightGeometry_ = true;
    applyStructuredLightCoveredGeometry(coveredVertices);
}

QString OffLineProgrammingSimulationMainWindow::makePlanningInfo(size_t segmentIndex,
                                                                 MotionTaskType segmentType,
                                                                 const Vector3d& basePosition,
                                                                 double manipulability,
                                                                 const Matrix4d* viewpointPose) const
{
    Q_UNUSED(segmentType);
    QString info;
    info += QString("Planning segment %1 information\n").arg(segmentIndex);
    info += QString("Base position (mm): %1\n").arg(formatVector3dForInfo(basePosition));
    info += QString("Minimum welding manipulability: %1\n").arg(manipulability, 0, 'g', 12);
    if (viewpointPose) {
        Vector3d viewpointPosition = viewpointPose->block<3,1>(0,3);
        Vector3d right = viewpointPose->block<3,1>(0,0);
        Vector3d up = viewpointPose->block<3,1>(0,1);
        Vector3d forward = viewpointPose->block<3,1>(0,2);
        info += QString("Viewpoint position (mm): %1\n").arg(formatVector3dForInfo(viewpointPosition));
        info += QString("Viewpoint orientation Right: %1\n").arg(formatVector3dForInfo(right));
        info += QString("Viewpoint orientation Up: %1\n").arg(formatVector3dForInfo(up));
        info += QString("Viewpoint orientation Forward: %1").arg(formatVector3dForInfo(forward));
    } else {
        info += "Viewpoint position (mm): N/A\n";
        info += "Viewpoint orientation Right: N/A\n";
        info += "Viewpoint orientation Up: N/A\n";
        info += "Viewpoint orientation Forward: N/A";
    }
    return info;
}

void OffLineProgrammingSimulationMainWindow::appendPlanningInfo(const QString& planningInfo)
{
    if (planningInfo.trimmed().isEmpty() || !ui || !ui->textEdit) {
        return;
    }

    ui->textEdit->append(planningInfo);
    ui->textEdit->append(QString());
}

void OffLineProgrammingSimulationMainWindow::recordTrajectorySegment(const std::vector<Vector6d>& trajectory,
                                                                     const std::vector<Vector3d>& baseTrajectory,
                                                                     bool planningSuccess,
                                                                     MotionTaskType trajectoryType,
                                                                     const std::vector<Vector6d>& motions,
                                                                     const Matrix4d* projectorPose,
                                                                     const std::array<Vector3d, 5>* frustumPoints,
                                                                     const std::vector<Vector3d>* coveredVertices,
                                                                     const QString& planningInfo)
{
    recordedTrajectories_.push_back(trajectory);
    recordedBaseTrajectories_.push_back(baseTrajectory);
    recordedPlanningSuccess_.push_back(planningSuccess);
    recordedTrajectoryTypes_.push_back(trajectoryType);
    recordedMotions_.push_back(motions);
    recordedHasProjectorPose_.push_back(projectorPose != nullptr);
    recordedProjectorPoses_.push_back(projectorPose ? *projectorPose : Matrix4d::Identity());
    recordedHasStructuredLightGeometry_.push_back(frustumPoints != nullptr && coveredVertices != nullptr);
    std::array<Vector3d, 5> storedFrustumPoints;
    storedFrustumPoints.fill(Vector3d::Zero());
    if (frustumPoints) {
        storedFrustumPoints = *frustumPoints;
    }
    recordedFrustumPoints_.push_back(storedFrustumPoints);
    recordedCoveredVertices_.push_back(coveredVertices ? *coveredVertices : std::vector<Vector3d>{});
    recordedPlanningInfos_.push_back(planningInfo);
}

void OffLineProgrammingSimulationMainWindow::startTrajectoryRecording()
{
    if (isRecordingTrajectory_) {
        qDebug() << u8"已经在记录轨迹中";
        return;
    }

    recordedTrajectories_.clear();
    recordedBaseTrajectories_.clear();
    recordedPlanningSuccess_.clear();
    recordedTrajectoryTypes_.clear();
    recordedProjectorPoses_.clear();
    recordedHasProjectorPose_.clear();
    recordedMotions_.clear();
    recordedFrustumPoints_.clear();
    recordedCoveredVertices_.clear();
    recordedHasStructuredLightGeometry_.clear();
    recordedPlanningInfos_.clear();
    isRecordingTrajectory_ = true;
    currentTrajectoryIndex_ = -1;

    qDebug() << u8"开始记录轨迹";
}

void OffLineProgrammingSimulationMainWindow::stopTrajectoryRecording()
{
    if (!isRecordingTrajectory_) {
        qDebug() << u8"当前没有在记录轨迹";
        return;
    }

    isRecordingTrajectory_ = false;
}

void OffLineProgrammingSimulationMainWindow::saveTrajectoryToFile(const QString &filename)
{
    std::ofstream outFile(filename.toStdString());

    if (!outFile.is_open()) {
        QMessageBox::warning(this, u8"保存失败", u8"无法创建文件: " + filename);
        return;
    }

    // 写入文件头部信息
    outFile << "======================================================" << std::endl;
    outFile << "          轨迹文件 - 离线编程仿真系统" << std::endl;
    outFile << "======================================================" << std::endl;
    outFile << std::endl;

    outFile << "统计信息:" << std::endl;
    outFile << "- 总轨迹段数: " << recordedTrajectories_.size() << std::endl;
    outFile << "- 生成时间: " << __DATE__ << " " << __TIME__ << std::endl;
    outFile << std::endl;

    outFile << "数据格式说明:" << std::endl;
    outFile << "- 轨迹段索引: 从0开始的段编号" << std::endl;
    outFile << "- 规划成功: 1表示成功，0表示失败" << std::endl;
    outFile << "- 轨迹段类型: TRANSITION表示过渡，CAMERA_CAPTURE表示拍照，WELDING表示焊接" << std::endl;
    outFile << "- 投影器位姿: 仅拍照段使用，4x4矩阵按行写入16个数" << std::endl;
    outFile << "- 视锥点数量: 缓存的半视锥关键点数量，按P/A/B/C/D顺序保存" << std::endl;
    outFile << "- 覆盖顶点数量: 缓存的覆盖区域三角网格顶点数量，每3个点组成1个三角形" << std::endl;
    outFile << "- 轨迹点数量: RRT规划路径点数量" << std::endl;
    outFile << "- 目标点数量: motions路径点数量" << std::endl;
    outFile << std::endl;

    outFile << "======================================================" << std::endl;
    outFile << "                    数据内容" << std::endl;
    outFile << "======================================================" << std::endl;
    outFile << std::endl;

    // 写入轨迹数据
    for (size_t i = 0; i < recordedTrajectories_.size(); ++i) {
        MotionTaskType segmentType = i < recordedTrajectoryTypes_.size()
            ? recordedTrajectoryTypes_[i]
            : MotionTaskType::TRANSITION;
        outFile << "轨迹段 " << i << ":" << std::endl;
        outFile << "- 规划成功: " << (recordedPlanningSuccess_[i] ? "是" : "否") << std::endl;
        outFile << "- 轨迹段类型: " << motionTaskTypeToString(segmentType) << std::endl;
        outFile << "- 目标路径类型: " << (recordedMotions_[i].empty() ? "NONE" : motionTaskTypeToString(MotionTaskType::WELDING)) << std::endl;
        if (i < recordedPlanningInfos_.size() && !recordedPlanningInfos_[i].trimmed().isEmpty()) {
            outFile << "规划输出信息:" << std::endl;
            const QStringList infoLines = recordedPlanningInfos_[i].split('\n');
            for (const QString& infoLine : infoLines) {
                outFile << "  " << infoLine.toStdString() << std::endl;
            }
            outFile << "规划输出信息结束" << std::endl;
        }
        if (i < recordedHasProjectorPose_.size() && recordedHasProjectorPose_[i] && i < recordedProjectorPoses_.size()) {
            outFile << "- 投影器位姿: ";
            for (int row = 0; row < 4; ++row) {
                for (int col = 0; col < 4; ++col) {
                    outFile << std::fixed << std::setprecision(6) << recordedProjectorPoses_[i](row, col);
                    if (!(row == 3 && col == 3)) outFile << ", ";
                }
            }
            outFile << std::endl;
        }
        bool hasStructuredLightGeometry = i < recordedHasStructuredLightGeometry_.size() &&
                                          recordedHasStructuredLightGeometry_[i] &&
                                          i < recordedFrustumPoints_.size() &&
                                          i < recordedCoveredVertices_.size();
        outFile << "- 结构光几何缓存: " << (hasStructuredLightGeometry ? "是" : "否") << std::endl;
        if (hasStructuredLightGeometry) {
            outFile << "- 视锥点数量: " << recordedFrustumPoints_[i].size() << std::endl;
            outFile << "视锥点数据 (P, A, B, C, D):" << std::endl;
            for (size_t j = 0; j < recordedFrustumPoints_[i].size(); ++j) {
                const Vector3d& point = recordedFrustumPoints_[i][j];
                outFile << "  点" << j << ": "
                        << std::fixed << std::setprecision(6)
                        << point.x() << ", "
                        << point.y() << ", "
                        << point.z() << std::endl;
            }
            outFile << "- 覆盖顶点数量: " << recordedCoveredVertices_[i].size() << std::endl;
            outFile << "覆盖顶点数据 (每3个点组成1个三角形):" << std::endl;
            for (size_t j = 0; j < recordedCoveredVertices_[i].size(); ++j) {
                const Vector3d& point = recordedCoveredVertices_[i][j];
                outFile << "  点" << j << ": "
                        << std::fixed << std::setprecision(6)
                        << point.x() << ", "
                        << point.y() << ", "
                        << point.z() << std::endl;
            }
        }
        outFile << "- RRT轨迹点数量: " << recordedTrajectories_[i].size() << std::endl;
        outFile << "- 基座轨迹点数量: " << recordedBaseTrajectories_[i].size() << std::endl;
        outFile << "- 目标路径点数量: " << recordedMotions_[i].size() << std::endl;
        outFile << std::endl;

        // 写入RRT轨迹数据
        if (recordedPlanningSuccess_[i] && !recordedTrajectories_[i].empty()) {
            outFile << "RRT轨迹数据 (关节角度，单位: 弧度):" << std::endl;
            for (size_t j = 0; j < recordedTrajectories_[i].size(); ++j) {
                outFile << "  点" << j << ": ";
                for (int k = 0; k < 6; ++k) {
                    outFile << std::fixed << std::setprecision(6) << recordedTrajectories_[i][j][k];
                    if (k < 5) outFile << ", ";
                }
                outFile << std::endl;
            }
            outFile << std::endl;

            // 写入基座轨迹数据
            outFile << "基座轨迹数据 (X, Y, Z坐标，单位: mm):" << std::endl;
            for (size_t j = 0; j < recordedBaseTrajectories_[i].size(); ++j) {
                outFile << "  点" << j << ": "
                        << std::fixed << std::setprecision(3) << recordedBaseTrajectories_[i][j][0] << ", "
                        << recordedBaseTrajectories_[i][j][1] << ", "
                        << recordedBaseTrajectories_[i][j][2] << std::endl;
            }
            outFile << std::endl;
        }

        // 写入目标路径数据
        if (!recordedMotions_[i].empty()) {
            outFile << "目标路径数据 (motions，关节角度，单位: 弧度):" << std::endl;
            for (size_t j = 0; j < recordedMotions_[i].size(); ++j) {
                outFile << "  点" << j << ": ";
                for (int k = 0; k < 6; ++k) {
                    outFile << std::fixed << std::setprecision(6) << recordedMotions_[i][j][k];
                    if (k < 5) outFile << ", ";
                }
                outFile << std::endl;
            }
        }
        outFile << "------------------------------------------------------" << std::endl;
        outFile << std::endl;
    }

    outFile.close();
    QMessageBox::information(this, u8"保存成功", u8"轨迹文件已保存到: " + filename);
    qDebug() << u8"轨迹文件已保存到: " << filename;
}

void OffLineProgrammingSimulationMainWindow::loadTrajectoryFromFile(const QString &filename)
{
    QFile file(filename);
        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            QMessageBox::warning(this, u8"加载失败", u8"无法打开文件: " + filename);
            return;
        }

        // 清空现有数据
        recordedTrajectories_.clear();
        recordedBaseTrajectories_.clear();
        recordedMotions_.clear();
        recordedPlanningSuccess_.clear();
        recordedTrajectoryTypes_.clear();
        recordedProjectorPoses_.clear();
        recordedHasProjectorPose_.clear();
        recordedFrustumPoints_.clear();
        recordedCoveredVertices_.clear();
        recordedHasStructuredLightGeometry_.clear();
        recordedPlanningInfos_.clear();

        QTextStream in(&file);
        in.setCodec("UTF-8"); // 设置编码为UTF-8

        std::string line;
        int currentSegment = -1;
        std::vector<Vector6d> currentTrajectory;
        std::vector<Vector3d> currentBaseTrajectory;
        std::vector<Vector6d> currentMotions;
        bool currentSuccess = false;
        MotionTaskType currentTrajectoryType = MotionTaskType::TRANSITION;
        Matrix4d currentProjectorPose = Matrix4d::Identity();
        bool currentHasProjectorPose = false;
        std::array<Vector3d, 5> currentFrustumPoints;
        currentFrustumPoints.fill(Vector3d::Zero());
        std::vector<Vector3d> currentCoveredVertices;
        bool currentHasStructuredLightGeometry = false;
        QString currentPlanningInfo;

        // 添加数据读取状态跟踪
        enum DataType { NONE, RRT_TRAJECTORY, BASE_TRAJECTORY, MOTIONS_TRAJECTORY, FRUSTUM_POINTS, COVERED_VERTICES, PLANNING_INFO };
        DataType currentDataType = NONE;

        while (!in.atEnd()) {
            QString qline = in.readLine();
            line = qline.toStdString();

            // 跳过空行和注释行
            if (line.empty() || line[0] == '=' || line.find_first_not_of("- ") == std::string::npos || line.find("轨迹文件") != std::string::npos) {
                continue;
            }

            // 检测轨迹段开始
            if (line.rfind("轨迹段 ", 0) == 0) {
                // 保存上一段轨迹
                if (currentSegment >= 0) {
                    recordedTrajectories_.push_back(currentTrajectory);
                    recordedBaseTrajectories_.push_back(currentBaseTrajectory);
                    recordedMotions_.push_back(currentMotions);
                    recordedPlanningSuccess_.push_back(currentSuccess);
                    recordedTrajectoryTypes_.push_back(currentTrajectoryType);
                    recordedProjectorPoses_.push_back(currentProjectorPose);
                    recordedHasProjectorPose_.push_back(currentHasProjectorPose);
                    recordedFrustumPoints_.push_back(currentFrustumPoints);
                    recordedCoveredVertices_.push_back(currentCoveredVertices);
                    recordedHasStructuredLightGeometry_.push_back(currentHasStructuredLightGeometry);
                    recordedPlanningInfos_.push_back(currentPlanningInfo.trimmed());
                }

                // 开始新段
                currentSegment++;
                currentTrajectory.clear();
                currentBaseTrajectory.clear();
                currentMotions.clear();
                currentSuccess = true;  // 旧格式未写明状态时默认按成功处理
                currentTrajectoryType = MotionTaskType::TRANSITION;
                currentProjectorPose = Matrix4d::Identity();
                currentHasProjectorPose = false;
                currentFrustumPoints.fill(Vector3d::Zero());
                currentCoveredVertices.clear();
                currentHasStructuredLightGeometry = false;
                currentPlanningInfo.clear();
                currentDataType = NONE;

                // 立即检测规划成功状态（在同一行中）
                if (line.find("规划成功: 是") != std::string::npos) {
                    currentSuccess = true;
                    qDebug() << u8"检测到第" << currentSegment << u8"段轨迹规划成功";
                } else if (line.find("规划成功: 否") != std::string::npos) {
                    currentSuccess = false;
                    qDebug() << u8"检测到第" << currentSegment << u8"段轨迹规划失败";
                }
            }

            // 检测规划成功状态（在后续行中）
            if (line.find("- 规划成功: 是") != std::string::npos) {
                currentSuccess = true;
                qDebug() << u8"检测到规划成功状态";
            } else if (line.find("- 规划成功: 否") != std::string::npos) {
                currentSuccess = false;
                qDebug() << u8"检测到规划失败状态";
            }

            if (line.find("- 轨迹段类型:") != std::string::npos) {
                currentTrajectoryType = motionTaskTypeFromString(line);
                qDebug() << u8"检测到轨迹段类型:" << motionTaskTypeToString(currentTrajectoryType);
            }

            if (line.find("- 投影器位姿:") != std::string::npos) {
                std::vector<double> values = parseDoublesFromLine(line);
                if (values.size() == 16) {
                    for (int row = 0; row < 4; ++row) {
                        for (int col = 0; col < 4; ++col) {
                            currentProjectorPose(row, col) = values[row * 4 + col];
                        }
                    }
                    currentHasProjectorPose = true;
                    qDebug() << u8"检测到投影器位姿";
                }
            }

            if (line.find("- 结构光几何缓存: 是") != std::string::npos) {
                currentHasStructuredLightGeometry = true;
                continue;
            } else if (line.find("- 结构光几何缓存: 否") != std::string::npos) {
                currentHasStructuredLightGeometry = false;
                continue;
            }

            if (line.find("规划输出信息:") != std::string::npos) {
                currentPlanningInfo.clear();
                currentDataType = PLANNING_INFO;
                continue;
            }

            if (line.find("规划输出信息结束") != std::string::npos) {
                currentDataType = NONE;
                continue;
            }

            if (currentDataType == PLANNING_INFO) {
                QString infoLine = qline;
                if (infoLine.startsWith("  ")) {
                    infoLine = infoLine.mid(2);
                }
                if (!currentPlanningInfo.isEmpty()) {
                    currentPlanningInfo += "\n";
                }
                currentPlanningInfo += infoLine;
                continue;
            }

            // 设置当前读取的数据类型
            if (line.find("RRT轨迹数据") != std::string::npos) {
                currentDataType = RRT_TRAJECTORY;
                continue;
            }

            if (line.find("基座轨迹数据") != std::string::npos) {
                currentDataType = BASE_TRAJECTORY;
                continue;
            }

            if (line.find("目标路径数据") != std::string::npos) {
                currentDataType = MOTIONS_TRAJECTORY;
                continue;
            }

            if (line.find("视锥点数据") != std::string::npos) {
                currentDataType = FRUSTUM_POINTS;
                continue;
            }

            if (line.find("覆盖顶点数据") != std::string::npos) {
                currentDataType = COVERED_VERTICES;
                continue;
            }

            // 解析数据行
            if (currentDataType != NONE && line.find("点") != std::string::npos) {
                size_t pointLabelPos = line.find("点");
                size_t colonMarkerPos = line.find(":");
                int pointIndex = -1;
                if (pointLabelPos != std::string::npos && colonMarkerPos != std::string::npos && colonMarkerPos > pointLabelPos + 1) {
                    try {
                        pointIndex = std::stoi(line.substr(pointLabelPos + 3, colonMarkerPos - (pointLabelPos + 3)));
                    } catch (const std::exception&) {
                        pointIndex = -1;
                    }
                }

                // 使用更健壮的数据解析方法
                std::string dataLine = line;
                // 移除"点X:"部分
                size_t dataColonPos = dataLine.find(":");
                if (dataColonPos != std::string::npos) {
                    dataLine = dataLine.substr(dataColonPos + 1);
                }

                // 移除空格
                dataLine.erase(std::remove(dataLine.begin(), dataLine.end(), ' '), dataLine.end());

                // 分割逗号分隔的数值
                std::vector<double> values;
                std::stringstream ss(dataLine);
                std::string token;

                while (std::getline(ss, token, ',')) {
                    try {
                        double value = std::stod(token);
                        values.push_back(value);
                    } catch (const std::exception& e) {
                        // 忽略转换错误
                        continue;
                    }
                }

                if (values.size() == 6) {
                    // 这是关节角度数据
                    Vector6d jointAngles;
                    for (int i = 0; i < 6; ++i) {
                        jointAngles[i] = values[i];
                    }

                    // 根据当前数据类型添加到相应的容器
                    if (currentDataType == RRT_TRAJECTORY) {
                        currentTrajectory.push_back(jointAngles);
                    } else if (currentDataType == MOTIONS_TRAJECTORY) {
                        currentMotions.push_back(jointAngles);
                    }
                } else if (values.size() == 3) {
                    // 这是基座坐标数据
                    Vector3d basePos;
                    for (int i = 0; i < 3; ++i) {
                        basePos[i] = values[i];
                    }
                    if (currentDataType == BASE_TRAJECTORY) {
                        currentBaseTrajectory.push_back(basePos);
                    } else if (currentDataType == FRUSTUM_POINTS) {
                        if (pointIndex >= 0 && static_cast<size_t>(pointIndex) < currentFrustumPoints.size()) {
                            currentFrustumPoints[pointIndex] = basePos;
                            currentHasStructuredLightGeometry = true;
                        }
                    } else if (currentDataType == COVERED_VERTICES) {
                        currentCoveredVertices.push_back(basePos);
                        currentHasStructuredLightGeometry = true;
                    }
                }
            }
        }

        // 保存最后一段轨迹
        if (currentSegment >= 0) {
            recordedTrajectories_.push_back(currentTrajectory);
            recordedBaseTrajectories_.push_back(currentBaseTrajectory);
            recordedMotions_.push_back(currentMotions);
            recordedPlanningSuccess_.push_back(currentSuccess);
            recordedTrajectoryTypes_.push_back(currentTrajectoryType);
            recordedProjectorPoses_.push_back(currentProjectorPose);
            recordedHasProjectorPose_.push_back(currentHasProjectorPose);
            recordedFrustumPoints_.push_back(currentFrustumPoints);
            recordedCoveredVertices_.push_back(currentCoveredVertices);
            recordedHasStructuredLightGeometry_.push_back(currentHasStructuredLightGeometry);
            recordedPlanningInfos_.push_back(currentPlanningInfo.trimmed());
        }

        file.close();

        // 添加调试信息
        qDebug() << u8"加载完成统计:";
        qDebug() << u8"轨迹段数:" << recordedTrajectories_.size();
        for (size_t i = 0; i < recordedTrajectories_.size(); ++i) {
            MotionTaskType segmentType = i < recordedTrajectoryTypes_.size()
                ? recordedTrajectoryTypes_[i]
                : MotionTaskType::TRANSITION;
            qDebug() << u8"段" << i << u8": RRT点数=" << recordedTrajectories_[i].size()
                     << u8", 基座点数=" << recordedBaseTrajectories_[i].size()
                     << u8", 目标点数=" << recordedMotions_[i].size()
                     << u8", 类型=" << motionTaskTypeToString(segmentType)
                     << u8", 成功=" << recordedPlanningSuccess_[i];
        }

        QMessageBox::information(this, u8"加载成功", u8"轨迹文件已加载，共" + QString::number(recordedTrajectories_.size()) + u8"段轨迹");
        qDebug() << u8"轨迹文件已加载，共" << recordedTrajectories_.size() << u8"段轨迹";
}

void OffLineProgrammingSimulationMainWindow::replayTrajectory(int trajectoryIndex)
{
    if (recordedTrajectories_.empty()) {
        QMessageBox::warning(this, u8"回放失败", u8"没有可回放的轨迹数据");
        return;
    }

    if (trajectoryIndex < 0) {
        // 回放所有轨迹
        for (size_t i = 0; i < recordedTrajectories_.size(); ++i) {
            if (recordedPlanningSuccess_[i] && !recordedTrajectories_[i].empty()) {
                qDebug() << u8"回放第" << i << u8"段轨迹";
                MotionTaskType segmentType = i < recordedTrajectoryTypes_.size()
                    ? recordedTrajectoryTypes_[i]
                    : MotionTaskType::TRANSITION;
                const Matrix4d* projectorPose = (i < recordedHasProjectorPose_.size() &&
                                                 recordedHasProjectorPose_[i] &&
                                                 i < recordedProjectorPoses_.size())
                    ? &recordedProjectorPoses_[i]
                    : nullptr;
                const std::array<Vector3d, 5>* frustumPoints = (i < recordedHasStructuredLightGeometry_.size() &&
                                                                recordedHasStructuredLightGeometry_[i] &&
                                                                i < recordedFrustumPoints_.size())
                    ? &recordedFrustumPoints_[i]
                    : nullptr;
                const std::vector<Vector3d>* coveredVertices = (i < recordedHasStructuredLightGeometry_.size() &&
                                                                recordedHasStructuredLightGeometry_[i] &&
                                                                i < recordedCoveredVertices_.size())
                    ? &recordedCoveredVertices_[i]
                    : nullptr;
                QString planningInfo = i < recordedPlanningInfos_.size()
                    ? recordedPlanningInfos_[i]
                    : QString();

                // 执行RRT规划路径
                for (size_t j = 0; j < recordedTrajectories_[i].size(); ++j) {
                    double baseX = 0, baseY = 0, baseZ = 0;
                    if (j < recordedBaseTrajectories_[i].size()) {
                        baseX = recordedBaseTrajectories_[i][j][0];
                        baseY = recordedBaseTrajectories_[i][j][1];
                        baseZ = recordedBaseTrajectories_[i][j][2];
                    }
                    SetJointAnglesDeg(recordedTrajectories_[i][j],
                                      baseX,
                                      baseY,
                                      baseZ,
                                      MotionMode::JOINT_AND_BASE,
                                      segmentType,
                                      projectorPose,
                                      frustumPoints,
                                      coveredVertices,
                                      j == 0 ? planningInfo : QString());

                }

                // 执行目标路径（如果存在）
                if (!recordedMotions_[i].empty()) {
                    for (size_t j = 0; j < recordedMotions_[i].size(); ++j) {
                        SetJointAnglesDeg(recordedMotions_[i][j], 0, 0, 0, MotionMode::JOINT_ONLY, MotionTaskType::WELDING);

                    }
                }
            } else {
                qDebug() << u8"跳过第" << i << u8"段失败轨迹";
            }
        }
    } else if (trajectoryIndex < recordedTrajectories_.size()) {
        // 回放指定轨迹段
        if (recordedPlanningSuccess_[trajectoryIndex] && !recordedTrajectories_[trajectoryIndex].empty()) {
            qDebug() << u8"回放第" << trajectoryIndex << u8"段轨迹";
            MotionTaskType segmentType = static_cast<size_t>(trajectoryIndex) < recordedTrajectoryTypes_.size()
                ? recordedTrajectoryTypes_[trajectoryIndex]
                : MotionTaskType::TRANSITION;
            const Matrix4d* projectorPose = (static_cast<size_t>(trajectoryIndex) < recordedHasProjectorPose_.size() &&
                                             recordedHasProjectorPose_[trajectoryIndex] &&
                                             static_cast<size_t>(trajectoryIndex) < recordedProjectorPoses_.size())
                ? &recordedProjectorPoses_[trajectoryIndex]
                : nullptr;
            const std::array<Vector3d, 5>* frustumPoints = (static_cast<size_t>(trajectoryIndex) < recordedHasStructuredLightGeometry_.size() &&
                                                            recordedHasStructuredLightGeometry_[trajectoryIndex] &&
                                                            static_cast<size_t>(trajectoryIndex) < recordedFrustumPoints_.size())
                ? &recordedFrustumPoints_[trajectoryIndex]
                : nullptr;
            const std::vector<Vector3d>* coveredVertices = (static_cast<size_t>(trajectoryIndex) < recordedHasStructuredLightGeometry_.size() &&
                                                            recordedHasStructuredLightGeometry_[trajectoryIndex] &&
                                                            static_cast<size_t>(trajectoryIndex) < recordedCoveredVertices_.size())
                ? &recordedCoveredVertices_[trajectoryIndex]
                : nullptr;
            QString planningInfo = static_cast<size_t>(trajectoryIndex) < recordedPlanningInfos_.size()
                ? recordedPlanningInfos_[trajectoryIndex]
                : QString();

            // 执行RRT规划路径
            for (size_t j = 0; j < recordedTrajectories_[trajectoryIndex].size(); ++j) {
                double baseX = 0, baseY = 0, baseZ = 0;
                if (j < recordedBaseTrajectories_[trajectoryIndex].size()) {
                    baseX = recordedBaseTrajectories_[trajectoryIndex][j][0];
                    baseY = recordedBaseTrajectories_[trajectoryIndex][j][1];
                    baseZ = recordedBaseTrajectories_[trajectoryIndex][j][2];
                }
                SetJointAnglesDeg(recordedTrajectories_[trajectoryIndex][j],
                                  baseX,
                                  baseY,
                                  baseZ,
                                  MotionMode::JOINT_AND_BASE,
                                  segmentType,
                                  projectorPose,
                                  frustumPoints,
                                  coveredVertices,
                                  j == 0 ? planningInfo : QString());


            }

            // 执行目标路径（如果存在）
            if (!recordedMotions_[trajectoryIndex].empty()) {
                for (size_t j = 0; j < recordedMotions_[trajectoryIndex].size(); ++j) {
                    SetJointAnglesDeg(recordedMotions_[trajectoryIndex][j], 0, 0, 0, MotionMode::JOINT_ONLY, MotionTaskType::WELDING);

                }
            }
        } else {
            QMessageBox::warning(this, u8"回放失败", u8"指定的轨迹段规划失败或为空");
        }
    } else {
        QMessageBox::warning(this, u8"回放失败", u8"指定的轨迹段索引超出范围");
    }

    isReplayingTrajectory_ = isProcessingQueue_ || smoothMotionTimer_->isActive() || !motionQueue_.empty();
    isPaused_ = false;
    qDebug() << u8"轨迹回放完成";
}

void OffLineProgrammingSimulationMainWindow::pauseReplayTrajectory()
{
    if (!isReplayingTrajectory_ && !isProcessingQueue_ && !smoothMotionTimer_->isActive()) {
        qDebug() << u8"当前没有正在演示的轨迹";
        return;
    }

    if (isPaused_) {
        return;
    }

    isPaused_ = true;
    if (smoothMotionTimer_->isActive()) {
        smoothMotionTimer_->stop();
    }
    qDebug() << u8"轨迹演示已暂停";
}

void OffLineProgrammingSimulationMainWindow::resumeReplayTrajectory()
{
    if (!isPaused_) {
        return;
    }

    isPaused_ = false;
    isReplayingTrajectory_ = true;

    if (!smoothMotionTimer_->isActive()) {
        if (isProcessingQueue_) {
            smoothMotionTimer_->start(17);
        } else if (!motionQueue_.empty()) {
            processNextMotion();
        }
    }
    qDebug() << u8"轨迹演示已继续";
}

void OffLineProgrammingSimulationMainWindow::stopReplayTrajectory()
{
    if (smoothMotionTimer_->isActive()) {
        smoothMotionTimer_->stop();
    }

    while (!motionQueue_.empty()) {
        motionQueue_.pop();
    }

    isProcessingQueue_ = false;
    isReplayingTrajectory_ = false;
    isPaused_ = false;
    currentMotionTaskType_ = MotionTaskType::TRANSITION;
    currentHasProjectorPose_ = false;
    setStructuredLightActorsVisible(false);
    qDebug() << u8"轨迹演示已停止";
}

void OffLineProgrammingSimulationMainWindow::onReplayTimer()
{
    onSmoothMotionTimer();
}

void OffLineProgrammingSimulationMainWindow::clearRecordedTrajectories()
{
    recordedTrajectories_.clear();
    recordedBaseTrajectories_.clear();
    recordedMotions_.clear();
    recordedPlanningSuccess_.clear();
    recordedTrajectoryTypes_.clear();
    recordedProjectorPoses_.clear();
    recordedHasProjectorPose_.clear();
    recordedFrustumPoints_.clear();
    recordedCoveredVertices_.clear();
    recordedHasStructuredLightGeometry_.clear();
    recordedPlanningInfos_.clear();
    currentTrajectoryIndex_ = -1;
    qDebug() << u8"已清除所有记录的轨迹";
}

void OffLineProgrammingSimulationMainWindow::saveDataToJson(const QString &filename)
{
    QJsonObject rootObj;

    // 保存基座位置
    QJsonArray basePositionsArray;
    for (const auto& pos : saved_base_positions_) {
        QJsonArray posArray;
        posArray.append(pos[0]);
        posArray.append(pos[1]);
        posArray.append(pos[2]);
        basePositionsArray.append(posArray);
    }
    rootObj["base_positions"] = basePositionsArray;

    // 保存冗余角度
    QJsonArray redundantAnglesArray;
    for (const auto& angles : saved_redundant_angles_) {
        QJsonArray angleArray;
        for (double angle : angles) {
            angleArray.append(angle);
        }
        redundantAnglesArray.append(angleArray);
    }
    rootObj["redundant_angles"] = redundantAnglesArray;

    // 保存质心位置
    QJsonArray centroidsArray;
    for (const auto& centroid : saved_centroids_) {
        QJsonArray centroidArray;
        centroidArray.append(centroid[0]);
        centroidArray.append(centroid[1]);
        centroidArray.append(centroid[2]);
        centroidsArray.append(centroidArray);
    }
    rootObj["centroids"] = centroidsArray;


    QJsonDocument doc(rootObj);
    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::warning(this, u8"保存失败", u8"无法写入文件: " + filename);
        return;
    }

    file.write(doc.toJson());
    file.close();
    QMessageBox::information(this, u8"保存成功", u8"数据已保存到JSON文件: " + filename);
    qDebug() << u8"数据已保存到JSON文件: " << filename;
}

void OffLineProgrammingSimulationMainWindow::createCustomToolAxes()
{
    // 轴长度
    double axisLength = 50.0;

    // 创建X轴（红色）
    vtkSmartPointer<vtkLineSource> xAxisLine = vtkSmartPointer<vtkLineSource>::New();
    xAxisLine->SetPoint1(0, 0, 0);
    xAxisLine->SetPoint2(axisLength, 0, 0);

    vtkSmartPointer<vtkPolyDataMapper> xAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    xAxisMapper->SetInputConnection(xAxisLine->GetOutputPort());

    toolXAxisActor = vtkSmartPointer<vtkActor>::New();
    toolXAxisActor->SetMapper(xAxisMapper);
    toolXAxisActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色
    toolXAxisActor->GetProperty()->SetLineWidth(3.0);
    toolXAxisActor->PickableOff();

    // 创建Y轴（绿色）
    vtkSmartPointer<vtkLineSource> yAxisLine = vtkSmartPointer<vtkLineSource>::New();
    yAxisLine->SetPoint1(0, 0, 0);
    yAxisLine->SetPoint2(0, axisLength, 0);

    vtkSmartPointer<vtkPolyDataMapper> yAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    yAxisMapper->SetInputConnection(yAxisLine->GetOutputPort());

    toolYAxisActor = vtkSmartPointer<vtkActor>::New();
    toolYAxisActor->SetMapper(yAxisMapper);
    toolYAxisActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色
    toolYAxisActor->GetProperty()->SetLineWidth(3.0);
    toolYAxisActor->PickableOff();

    // 创建Z轴（蓝色）
    vtkSmartPointer<vtkLineSource> zAxisLine = vtkSmartPointer<vtkLineSource>::New();
    zAxisLine->SetPoint1(0, 0, 0);
    zAxisLine->SetPoint2(0, 0, axisLength);

    vtkSmartPointer<vtkPolyDataMapper> zAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    zAxisMapper->SetInputConnection(zAxisLine->GetOutputPort());

    toolZAxisActor = vtkSmartPointer<vtkActor>::New();
    toolZAxisActor->SetMapper(zAxisMapper);
    toolZAxisActor->GetProperty()->SetColor(0.0, 0.0, 1.0); // 蓝色
    toolZAxisActor->GetProperty()->SetLineWidth(3.0);
    toolZAxisActor->PickableOff();

    // 添加到渲染器
    renderer->AddActor(toolXAxisActor);
    renderer->AddActor(toolYAxisActor);
    renderer->AddActor(toolZAxisActor);

    renderer2->AddActor(toolXAxisActor);
    renderer2->AddActor(toolYAxisActor);
    renderer2->AddActor(toolZAxisActor);

    // 初始更新坐标系位置
    updateCustomToolAxes();
}

void OffLineProgrammingSimulationMainWindow::updateCustomToolAxes()
{
    if (!toolXAxisActor || !toolYAxisActor || !toolZAxisActor) {
        return;
    }

    // 使用累积变换矩阵toolTransformMatrix_
    Eigen::Vector3d origin = toolTransformMatrix_.block<3,1>(0,3);
    Eigen::Matrix3d rotation = toolTransformMatrix_.block<3,3>(0,0);

    // 计算轴的方向向量
    Eigen::Vector3d xAxis = rotation.col(0) * 50.0; // X轴方向
    Eigen::Vector3d yAxis = rotation.col(1) * 50.0; // Y轴方向
    Eigen::Vector3d zAxis = rotation.col(2) * 50.0; // Z轴方向

    // 更新X轴
    vtkSmartPointer<vtkLineSource> xAxisLine = vtkSmartPointer<vtkLineSource>::New();
    xAxisLine->SetPoint1(origin.x(), origin.y(), origin.z());
    xAxisLine->SetPoint2(origin.x() + xAxis.x(), origin.y() + xAxis.y(), origin.z() + xAxis.z());

    vtkSmartPointer<vtkPolyDataMapper> xAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    xAxisMapper->SetInputConnection(xAxisLine->GetOutputPort());
    toolXAxisActor->SetMapper(xAxisMapper);

    // 更新Y轴
    vtkSmartPointer<vtkLineSource> yAxisLine = vtkSmartPointer<vtkLineSource>::New();
    yAxisLine->SetPoint1(origin.x(), origin.y(), origin.z());
    yAxisLine->SetPoint2(origin.x() + yAxis.x(), origin.y() + yAxis.y(), origin.z() + yAxis.z());

    vtkSmartPointer<vtkPolyDataMapper> yAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    yAxisMapper->SetInputConnection(yAxisLine->GetOutputPort());
    toolYAxisActor->SetMapper(yAxisMapper);

    // 更新Z轴
    vtkSmartPointer<vtkLineSource> zAxisLine = vtkSmartPointer<vtkLineSource>::New();
    zAxisLine->SetPoint1(origin.x(), origin.y(), origin.z());
    zAxisLine->SetPoint2(origin.x() + zAxis.x(), origin.y() + zAxis.y(), origin.z() + zAxis.z());

    vtkSmartPointer<vtkPolyDataMapper> zAxisMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    zAxisMapper->SetInputConnection(zAxisLine->GetOutputPort());
    toolZAxisActor->SetMapper(zAxisMapper);
}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_4_valueChanged(int value)
{
    startAnglesDeg_[0] = value * M_PI / 180.0;
    UpdateJointTransforms(startAnglesDeg_);
    updateJointAnglesLabel();
    performCollisionDetection();
    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_5_valueChanged(int value)
{
    startAnglesDeg_[1] = value * M_PI / 180.0;
    UpdateJointTransforms(startAnglesDeg_);
    updateJointAnglesLabel();
    performCollisionDetection();
    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();

}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_6_valueChanged(int value)
{
    startAnglesDeg_[2] = value * M_PI / 180.0;
    UpdateJointTransforms(startAnglesDeg_);
    updateJointAnglesLabel();
    performCollisionDetection();
    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();

}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_7_valueChanged(int value)
{
    startAnglesDeg_[3] = value * M_PI / 180.0;
    UpdateJointTransforms(startAnglesDeg_);
    updateJointAnglesLabel();
    performCollisionDetection();
    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();

}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_8_valueChanged(int value)
{
    startAnglesDeg_[4] = value * M_PI / 180.0;
    UpdateJointTransforms(startAnglesDeg_);
    updateJointAnglesLabel();
    performCollisionDetection();
    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();

}

void OffLineProgrammingSimulationMainWindow::on_horizontalSlider_9_valueChanged(int value)
{
    startAnglesDeg_[5] = value * M_PI / 180.0;
    UpdateJointTransforms(startAnglesDeg_);
    updateJointAnglesLabel();
    performCollisionDetection();
    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();

}

void OffLineProgrammingSimulationMainWindow::on_toolButton_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this,"Open STEP",".","Open STEP files(*.STEP)");
    if (!fileName.isEmpty()) {
        loadingModel(fileName);
        loadingRobot();
    }else{
        QMessageBox::information(this,u8"警告", u8"读取失败！");
        return;
    }
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_2_clicked()
{
    selected_edge_indices_ = edgeVisualizer->GetSelectedEdgeIndices();

    selected_edgeInfo.clear();

    for (int edge_idx : selected_edge_indices_) {
        if (edge_idx < 0 || edge_idx >= static_cast<int>(edgeInfo.size())) {
            continue;
        }

        EdgeInfo edge_info1;

        EdgeInfo original_edge = edgeInfo[edge_idx];
        edge_info1.edge = original_edge.edge;
        edge_info1.ends = original_edge.ends;
        edge_info1.is_arc_or_spline = original_edge.is_arc_or_spline;
        edge_info1.faces_info = original_edge.faces_info;
        edge_info1.normals = original_edge.normals;
        edge_info1.sample_points = original_edge.sample_points;
        selected_edgeInfo.push_back(edge_info1);
    }

    VisualizeSelectedEdgePointsAndNormals();
    calculateHanju();
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_3_clicked()
{
    if (renderer) {
        renderer->SetDraw(1);
        renderer2->GetRenderWindow()->Render();
    }
    if (renderer2) {
        renderer2->SetDraw(0);
    }
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_4_clicked()
{   
    ImprovedRRTStar rrt;

    std::vector<std::vector<Matrix4d>> trajectories;
    trajectories.reserve(all_coordinate_systems_B_.size());

    constexpr double TRANSITION_DIST = 30.0;
    constexpr double X_OFFSET = 10.0;
    constexpr double START_TARGET_Z = 120.0;  // 起点过渡点1的目标Z坐标
    constexpr double END_TARGET_Z = 120.0;    // 终点过渡点n的目标Z坐标

    for (const auto& seg : all_coordinate_systems_B_) {

        std::vector<Matrix4d> target;
        if (seg.empty()) {
            trajectories.push_back(target);
            continue;
        }

        // 生成起点过渡点
        {
            const Matrix4d& T0 = seg.front().transform_matrix;
            double start_z = T0(2, 3);  // 起点的Z坐标

            // 计算需要插入的过渡点数量
            int num_start_transitions = std::max(1, static_cast<int>((START_TARGET_Z - start_z) / X_OFFSET));

            // 第1个起点过渡点：起点沿Z轴负方向移动30mm
            Matrix4d T_start_base = T0;
            Vector3d z_axis = T0.block<3,1>(0,2);
            T_start_base.block<3,1>(0,3) -= TRANSITION_DIST * z_axis;

            // 生成起点过渡点序列
            std::vector<Matrix4d> start_transitions;
            start_transitions.reserve(num_start_transitions);

            Vector3d world_z_axis(0.0, 0.0, 1.0); // 世界坐标系Z轴正方向

            // 从基础点开始，沿Z轴正方向插入过渡点，直到达到目标Z坐标
            Matrix4d current_pose = T_start_base;
            for (int i = 0; i < num_start_transitions; ++i) {
                Matrix4d transition_pose = current_pose;
                transition_pose.block<3,1>(0,3) += X_OFFSET * world_z_axis;
                start_transitions.push_back(transition_pose);
                current_pose = transition_pose;
            }

            // 按照从高到低的顺序添加起点过渡点（先添加Z坐标最大的点）
            for (auto it = start_transitions.rbegin(); it != start_transitions.rend(); ++it) {
                target.push_back(*it);
            }
        }

        // 添加原始轨迹点（起点到终点）
        for (const auto& cs : seg) {
            target.push_back(cs.transform_matrix);
        }

        // 生成终点过渡点
        {
            const Matrix4d& Tn = seg.back().transform_matrix;
            double end_z = Tn(2, 3);  // 终点的Z坐标

            // 计算需要插入的过渡点数量
            int num_end_transitions = std::max(1, static_cast<int>((END_TARGET_Z - end_z) / X_OFFSET));

            // 第1个终点过渡点：终点沿Z轴负方向移动30mm
            Matrix4d T_end_base = Tn;
            Vector3d z_axis = Tn.block<3,1>(0,2);
            T_end_base.block<3,1>(0,3) -= TRANSITION_DIST * z_axis;

            // 生成终点过渡点序列
            std::vector<Matrix4d> end_transitions;
            end_transitions.reserve(num_end_transitions);

            Vector3d world_z_axis(0.0, 0.0, 1.0); // 世界坐标系Z轴正方向

            // 从基础点开始，沿Z轴正方向插入过渡点，直到达到目标Z坐标
            Matrix4d current_pose = T_end_base;
            for (int i = 0; i < num_end_transitions; ++i) {
                Matrix4d transition_pose = current_pose;
                transition_pose.block<3,1>(0,3) += X_OFFSET * world_z_axis;
                end_transitions.push_back(transition_pose);
                current_pose = transition_pose;
            }

            // 按照从低到高的顺序添加终点过渡点（先添加Z坐标最小的点）
            for (const auto& transition : end_transitions) {
                target.push_back(transition);
            }
        }

        trajectories.push_back(std::move(target));
    }

    auto makeOffsetViewpoint = [this](const CoordinateSystem& original_cs, double offset_distance, bool move_along_positive_y) {
        constexpr double Y_AXIS_OFFSET = 100.0;
        constexpr double AXIS_EPSILON = 1e-8;
        CoordinateSystem offset_cs = original_cs;

        Normal3D fixed_z_axis = modelprocess.NormalizeVec(offset_cs.z_axis);
        Normal3D world_down_axis(0.0, 0.0, -1.0);
        Normal3D projected_y_axis = modelprocess.ProjectVectorOntoPlane(world_down_axis, fixed_z_axis);

        double projected_y_norm = std::sqrt(projected_y_axis.x * projected_y_axis.x +
                                            projected_y_axis.y * projected_y_axis.y +
                                            projected_y_axis.z * projected_y_axis.z);

        if (projected_y_norm <= AXIS_EPSILON) {
            projected_y_axis = modelprocess.ProjectVectorOntoPlane(offset_cs.y_axis, fixed_z_axis);
            projected_y_norm = std::sqrt(projected_y_axis.x * projected_y_axis.x +
                                         projected_y_axis.y * projected_y_axis.y +
                                         projected_y_axis.z * projected_y_axis.z);
        }

        if (projected_y_norm <= AXIS_EPSILON) {
            projected_y_axis = modelprocess.ProjectVectorOntoPlane(Normal3D(1.0, 0.0, 0.0), fixed_z_axis);
            projected_y_norm = std::sqrt(projected_y_axis.x * projected_y_axis.x +
                                         projected_y_axis.y * projected_y_axis.y +
                                         projected_y_axis.z * projected_y_axis.z);
        }

        Normal3D fixed_y_axis = modelprocess.NormalizeVec(projected_y_axis);
        if (fixed_y_axis.z > 0.0) {
            fixed_y_axis = -1.0 * fixed_y_axis;
        }

        Normal3D fixed_x_axis = modelprocess.NormalizeVec(modelprocess.CrossProduct(fixed_y_axis, fixed_z_axis));
        fixed_y_axis = modelprocess.NormalizeVec(modelprocess.CrossProduct(fixed_z_axis, fixed_x_axis));

        offset_cs.x_axis = fixed_x_axis;
        offset_cs.y_axis = fixed_y_axis;
        offset_cs.z_axis = fixed_z_axis;

        // 先修正坐标系，再沿修正后的轴方向偏移
        offset_cs.origin.x += offset_distance * (-fixed_z_axis.x);
        offset_cs.origin.y += offset_distance * (-fixed_z_axis.y);
        offset_cs.origin.z += offset_distance * (-fixed_z_axis.z);

        const double y_offset_sign = move_along_positive_y ? 1.0 : -1.0;
        offset_cs.origin.x += y_offset_sign * Y_AXIS_OFFSET * fixed_y_axis.x;
        offset_cs.origin.y += y_offset_sign * Y_AXIS_OFFSET * fixed_y_axis.y;
        offset_cs.origin.z += y_offset_sign * Y_AXIS_OFFSET * fixed_y_axis.z;

        offset_cs.transform_matrix <<
            fixed_x_axis.x, fixed_y_axis.x, fixed_z_axis.x, offset_cs.origin.x,
            fixed_x_axis.y, fixed_y_axis.y, fixed_z_axis.y, offset_cs.origin.y,
            fixed_x_axis.z, fixed_y_axis.z, fixed_z_axis.z, offset_cs.origin.z,
            0, 0, 0, 1;

        return offset_cs.transform_matrix;
    };

    std::vector<Matrix4d> viewpoint;

    switch (currentWeldingType_) {
    case WeldingType::PIPE_TO_PIPE:
        if (!selected_edgeInfo.empty()) {
            const auto& edge = selected_edgeInfo.front();
            if (!edge.sample_points.empty() && !edge.normals.empty()) {
                size_t max_z_idx = 0;
                double max_z = edge.sample_points[0].z;
                for (size_t i = 1; i < edge.sample_points.size(); ++i) {
                    if (edge.sample_points[i].z > max_z) {
                        max_z = edge.sample_points[i].z;
                        max_z_idx = i;
                    }
                }

                if (max_z_idx >= (5 - 1)) {
                    const size_t idx_to_start = max_z_idx - (5 - 1);
                    const Point3D& point = edge.sample_points[idx_to_start];
                    const auto& normal_tuple = edge.normals[idx_to_start];
                    const Normal3D tangent = -1.0 * std::get<0>(normal_tuple);
                    const Normal3D binormal = -1.0 * std::get<1>(normal_tuple);
                    const Normal3D normal = std::get<2>(normal_tuple);
                    CoordinateSystem csB_view = createCoordinate(point, tangent, binormal, normal, 135.0, 180.0);
                    Matrix4d vp = csB_view.transform_matrix;
                    constexpr double PIPE_VIEW_OFFSET_Z = 500.0;
                    constexpr double PIPE_VIEW_OFFSET_Y = 100.0;
                    vp(0,3) += PIPE_VIEW_OFFSET_Z * (-csB_view.z_axis.x) + PIPE_VIEW_OFFSET_Y * (-csB_view.y_axis.x);
                    vp(1,3) += PIPE_VIEW_OFFSET_Z * (-csB_view.z_axis.y) + PIPE_VIEW_OFFSET_Y * (-csB_view.y_axis.y);
                    vp(2,3) += PIPE_VIEW_OFFSET_Z * (-csB_view.z_axis.z) + PIPE_VIEW_OFFSET_Y * (-csB_view.y_axis.z);
                    viewpoint.push_back(vp);
                }

                if (max_z_idx + (5 - 1) < edge.sample_points.size()) {
                    const size_t idx_to_end = max_z_idx + (5 - 1);
                    const Point3D& point = edge.sample_points[idx_to_end];
                    const auto& normal_tuple = edge.normals[idx_to_end];
                    const Normal3D tangent = std::get<0>(normal_tuple);
                    const Normal3D binormal = std::get<1>(normal_tuple);
                    const Normal3D normal = std::get<2>(normal_tuple);
                    CoordinateSystem csB_view = createCoordinate(point, tangent, binormal, normal, 135.0, 0.0);
                    Matrix4d vp = csB_view.transform_matrix;
                    constexpr double PIPE_VIEW_OFFSET_Z = 500.0;
                    constexpr double PIPE_VIEW_OFFSET_Y = 100.0;
                    vp(0,3) += PIPE_VIEW_OFFSET_Z * (-csB_view.z_axis.x) + PIPE_VIEW_OFFSET_Y * (-csB_view.y_axis.x);
                    vp(1,3) += PIPE_VIEW_OFFSET_Z * (-csB_view.z_axis.y) + PIPE_VIEW_OFFSET_Y * (-csB_view.y_axis.y);
                    vp(2,3) += PIPE_VIEW_OFFSET_Z * (-csB_view.z_axis.z) + PIPE_VIEW_OFFSET_Y * (-csB_view.y_axis.z);
                    viewpoint.push_back(vp);
                }
            }
        }
//        if (all_coordinate_systems_B_.size() > 1 &&
//            all_coordinate_systems_B_[0].size() >= 5 &&
//            all_coordinate_systems_B_[1].size() >= 5) {
//            viewpoint.push_back(
//                makeOffsetViewpoint(
//                    all_coordinate_systems_B_[0][all_coordinate_systems_B_[0].size() - 5], 500.0, false));
//            viewpoint.push_back(
//                makeOffsetViewpoint(
//                    all_coordinate_systems_B_[1][all_coordinate_systems_B_[1].size() - 5], 500.0, false));
//        }
        break;

    case WeldingType::PLATE_TO_PLATE:
        if (all_coordinate_systems_B_.size() > 1 && !all_coordinate_systems_B_[1].empty()) {
            size_t mid = all_coordinate_systems_B_[1].size() / 2;
            viewpoint.push_back(
                makeOffsetViewpoint(all_coordinate_systems_B_[1][mid], 500.0, false));
        }
        break;

    case WeldingType::PIPE_TO_PLATE:
        for (const auto& coordinateSystemGroup : all_coordinate_systems_B_) {
            if (coordinateSystemGroup.empty()) {
                continue;
            }

            size_t mid = coordinateSystemGroup.size() / 2;
            viewpoint.push_back(
                makeOffsetViewpoint(coordinateSystemGroup[mid], 500.0, false));
        }
        break;
    }

    if (trajectories.empty() || viewpoint.empty()) {
        QMessageBox::warning(this, u8"规划失败", u8"未生成有效焊缝路径，请先选择一条或两条有效焊缝");
        return;
    }

    /******************** 3. PSO 优化 ********************/
    int pop_size = 30, gen = 100;
    double inertia = 0.7, cognitive = 1.8, social = 1.8;

    std::vector<double> fixed_angles;
    fixed_angles.resize(trajectories.size(), 0.0);

    PoseEstimation solver(*robotConfig, pop_size, gen, inertia, cognitive, social);
    auto result = solver.optimizeBaseOnly(trajectories, viewpoint, fixed_angles);
//    auto result = solver.optimize(trajectories, viewpoint);

    qDebug() << "============ Optimization Result ============\n";
    qDebug() << "Placement:\n";
    qDebug() << "  X: " << result.placement.x << "\n";
    qDebug() << "  Y: " << result.placement.y << "\n";
    qDebug() << "  Z: " << result.placement.z << "\n";

    qDebug() << "Cost: " << result.cost << "\n";

    qDebug() << "Angles (per segment):\n";
    for (size_t i = 0; i < result.angles.size(); ++i) {
        qDebug() << "  theta[" << i << "] = " << result.angles[i] << "\n";
    }

    std::cout << "=============================================\n";

    Matrix4d baseMatrix;
    baseMatrix <<
        1.0,  0.0,  0.0, -result.placement.x,
        0.0, -1.0,  0.0,  result.placement.y,
        0.0,  0.0, -1.0,  result.placement.z,
        0.0,  0.0,  0.0,  1.0;

    auto makeRz = [](double theta) {
        Matrix4d R = Matrix4d::Identity();
        double c = std::cos(theta);
        double s = std::sin(theta);
        R(0,0) =  c;  R(0,1) = -s;
        R(1,0) =  s;  R(1,1) =  c;
        return R;
    };

    std::vector<std::vector<Matrix4d>> targetPose;

    for (size_t i = 0; i < trajectories.size(); i++) {
        Matrix4d Rz = makeRz(result.angles[i]);
        std::vector<Matrix4d> target;

        for (const auto& cs : trajectories[i]) {
            target.push_back(baseMatrix * (cs * Rz));
        }
        targetPose.push_back(std::move(target));
    }

    std::vector<std::vector<Vector6d>> motions = solver.cartesianPlanning2(targetPose);
    if (motions.empty()) {
        qDebug()<<u8"机器人无法运动"<<endl;
    }

    Matrix4d end2camMatrix;
    end2camMatrix <<
        1.0, 0.0, 0.0, -52.4276,
        0.0, 0.866, -0.5, -197.74489,
        0.0, 0.5,  0.866, 260.57368,
        0.0, 0.0,  0.0,   1.0;

    std::vector<Vector6d> view;
    std::vector<Matrix4d> projectorPoses;
    for (const auto& p : viewpoint) {
        Matrix4d projectorPose = p;
        Matrix4d q = (baseMatrix * p) * end2camMatrix;
        std::vector<Vector6d> solutions = solver.inverseKinematics(q);


        bool found_valid_solution = false;
        double min_distance = std::numeric_limits<double>::max();
        Vector6d selected_solution;

        for (const auto& sol : solutions) {
            // 检查关节角限制
            if (!solver.isValidSolution(sol, solver.qlimits)) {
                continue;
            }

            // 检查碰撞
            std::vector<double> q_vec;
            q_vec.reserve(9);
            for (int i = 0; i < 6; ++i) {
                q_vec.push_back(sol[i]);
            }
            q_vec.push_back(result.placement.x);
            q_vec.push_back(result.placement.y);
            q_vec.push_back(result.placement.z);

            if (!solver.isStateInCollision2(q_vec)) {
                // 计算与motions[0][0]的距离
                double distance = solver.angleDistance(sol, motions[0][0]);

                // 选择距离最小的解
                if (distance < min_distance) {
                    min_distance = distance;
                    selected_solution = sol;
                    found_valid_solution = true;
                }
            }
        }

        if (!found_valid_solution && !solutions.empty()) {
            selected_solution = solutions[0];
            qDebug()<<"no";
        }

        view.push_back(selected_solution);
        projectorPoses.push_back(projectorPose);
    }

    /******************** 8. RRT 规划 ********************/
    std::vector<std::vector<Vector6d>> traj;
    std::vector<std::vector<Vector3d>> base_traj;
    // 总段数：抬升段(1) + RRT规划段(1) + 基座移动段(1) + view段 + motions段
    std::vector<bool> planning_success(1 + 1 + 1 + motions.size() + view.size(), false);
    std::vector<MotionTaskType> trajectory_types(planning_success.size(), MotionTaskType::TRANSITION);

    auto fillBase = [&](std::vector<double>& q) {
        q[6] = result.placement.x;
        q[7] = result.placement.y;
        q[8] = result.placement.z;
    };

    /******************** 8.1 抬升段 ********************/
    std::vector<Vector6d> liftingAction;
    std::vector<Vector3d> liftingBaseAction;
    liftingAction.push_back(startAnglesDeg_);
    Vector3d currentBase(
        baseAssemblyX_,
        baseAssemblyY_,
        2100);
    liftingBaseAction.push_back(currentBase);
    traj.push_back(std::move(liftingAction));
    base_traj.push_back(std::move(liftingBaseAction));
    planning_success[0] = true;

    /******************** 8.2 RRT规划段 ********************/
    // 从抬升后的位置开始，进行第一次RRT规划
    std::vector<double> rrt_start(9, 0.0), rrt_goal(9, 0.0);

    // 起始状态：抬升后的关节角度和基座位置
    const Vector6d& lifting_joints = traj.back().back();
    const Vector3d& lifting_base = base_traj.back().back();

    for (int a = 0; a < 6; ++a)
        rrt_start[a] = lifting_joints[a];
    rrt_start[6] = lifting_base.x();
    rrt_start[7] = lifting_base.y();
    rrt_start[8] = lifting_base.z();

    // 目标状态：使用第一个view点的关节角度，基座位置保持抬升后的位置
    for (int a = 0; a < 6; ++a)
        rrt_goal[a] = view[0](a);
    rrt_goal[6] = lifting_base.x();
    rrt_goal[7] = lifting_base.y();
    rrt_goal[8] = lifting_base.z();

    std::vector<std::vector<double>> rrt_path;
    if (rrt.plan(rrt_start, rrt_goal, rrt_path) && !rrt_path.empty()) {
        std::vector<Vector6d> rrt_motion;
        std::vector<Vector3d> rrt_base_motion;

        for (const auto& q : rrt_path) {
            Vector6d j;
            for (int k = 0; k < 6; ++k) j[k] = q[k];
            rrt_motion.push_back(j);

            Vector3d b(q[6], q[7], q[8]);
            rrt_base_motion.push_back(b);
        }

        traj.push_back(std::move(rrt_motion));
        base_traj.push_back(std::move(rrt_base_motion));
        planning_success[1] = true;
    } else {
        // RRT规划失败，添加空段
        traj.emplace_back();
        base_traj.emplace_back();
        planning_success[1] = false;
    }

    /******************** 8.3 基座移动段 ********************/
    // 从RRT规划后的位置开始，直接移动基座到目标位置，关节角度保持不变
    std::vector<Vector6d> base_move_motion;
    std::vector<Vector3d> base_move_base_motion;

    // 起始状态：RRT规划后的关节角度和基座位置
    const Vector6d& rrt_joints = traj.back().back();

    // 目标状态：保持关节角度不变，基座移动到目标位置
    Vector3d target_base(result.placement.x, result.placement.y, result.placement.z);

    base_move_motion.push_back(rrt_joints);
    base_move_base_motion.push_back(target_base);

    traj.push_back(std::move(base_move_motion));
    base_traj.push_back(std::move(base_move_base_motion));
    planning_success[2] = true;

    /******************** 8.4 后续RRT规划段 ********************/
    // 从基座移动后的位置开始，依次进行view和motions的RRT规划
    for (size_t i = 0; i < view.size() + motions.size(); i++) {
        std::vector<double> q_start(9, 0.0), q_goal(9, 0.0);

        {
            const auto& last_traj = traj.back();
            const auto& last_base = base_traj.back();

            const Vector6d& j = last_traj.back();
            const Vector3d& b = last_base.back();

            for (int a = 0; a < 6; ++a)
                q_start[a] = j[a];

            q_start[6] = b.x();
            q_start[7] = b.y();
            q_start[8] = b.z();
        }

        /*************** 正确的终点定义 ***************/
        if (i < view.size()) {
            for (int a = 0; a < 6; ++a)
                q_goal[a] = view[i](a);

            fillBase(q_goal);
            trajectory_types[3 + i] = MotionTaskType::TRANSITION;
        } else {
            size_t m = i - view.size();
            for (int a = 0; a < 6; ++a)
                q_goal[a] = motions[m][0](a);

            fillBase(q_goal);
            trajectory_types[3 + i] = MotionTaskType::TRANSITION;
        }

        std::vector<std::vector<double>> path;
        if (!rrt.plan(q_start, q_goal, path) || path.empty()) {
            traj.emplace_back();
            base_traj.emplace_back();
            continue;
        }

        std::vector<Vector6d> motion;
        std::vector<Vector3d> base_motion;

        for (const auto& q : path) {
            Vector6d j;
            for (int k = 0; k < 6; ++k) j[k] = q[k];
            motion.push_back(j);

            Vector3d b(q[6], q[7], q[8]);
            base_motion.push_back(b);
        }

        traj.push_back(std::move(motion));
        base_traj.push_back(std::move(base_motion));
        planning_success[3 + i] = true; // 3 = 抬升段(0) + RRT规划段(1) + 基座移动段(2)
    }

    double weldingManipulability = std::numeric_limits<double>::max();
    for (const auto& weldingPath : motions) {
        if (!weldingPath.empty()) {
            weldingManipulability = std::min(weldingManipulability, solver.calculateManipulability(weldingPath));
        }
    }
    if (weldingManipulability == std::numeric_limits<double>::max()) {
        weldingManipulability = 0.0;
    }

    std::vector<QString> trajectoryPlanningInfos(traj.size());
    for (size_t i = 0; i < traj.size(); ++i) {
        if (i >= planning_success.size() || !planning_success[i] || traj[i].empty()) {
            continue;
        }

        bool is_view_segment = i >= 3 && (i - 3) < view.size();
        if (!is_view_segment) {
            continue;
        }

        size_t viewIndex = i - 3;
        const Matrix4d* viewpointPose = viewIndex < projectorPoses.size() ? &projectorPoses[viewIndex] : nullptr;
        if (!viewpointPose) {
            continue;
        }

        Vector3d basePosition = i < base_traj.size() && !base_traj[i].empty()
            ? base_traj[i].back()
            : Vector3d(result.placement.x, result.placement.y, result.placement.z);
        trajectoryPlanningInfos[i] = makePlanningInfo(i, MotionTaskType::CAMERA_CAPTURE, basePosition, weldingManipulability, viewpointPose);
        appendPlanningInfo(trajectoryPlanningInfos[i]);
    }

    /******************** 9. 执行 + 正确记录 ********************/
    bool has_successful_plan = false;

    for (size_t i = 0; i < traj.size(); i++) {
        if (!planning_success[i] || traj[i].empty()) {
            if (isRecordingTrajectory_) {
                recordTrajectorySegment({},
                                        {},
                                        false,
                                        i < trajectory_types.size() ? trajectory_types[i] : MotionTaskType::TRANSITION,
                                        {});
            }
            continue;
        }

        has_successful_plan = true;
        bool is_view_segment = i >= 3 && (i - 3) < view.size();
        const Matrix4d* captureProjectorPose = nullptr;
        std::array<Vector3d, 5> captureFrustumPoints;
        captureFrustumPoints.fill(Vector3d::Zero());
        std::vector<Vector3d> captureCoveredVertices;
        const std::array<Vector3d, 5>* captureFrustumPointsPtr = nullptr;
        const std::vector<Vector3d>* captureCoveredVerticesPtr = nullptr;
        QString capturePlanningInfo;
        if (is_view_segment) {
            size_t viewIndex = i - 3;
            captureProjectorPose = viewIndex < projectorPoses.size() ? &projectorPoses[viewIndex] : nullptr;
            if (captureProjectorPose) {
                buildHalfFrustumPoints(*captureProjectorPose, captureFrustumPoints);
                buildStructuredLightCoveredVertices(*captureProjectorPose, captureFrustumPoints, captureCoveredVertices);
                captureFrustumPointsPtr = &captureFrustumPoints;
                captureCoveredVerticesPtr = &captureCoveredVertices;
                Vector3d captureBase = i < base_traj.size() && !base_traj[i].empty()
                    ? base_traj[i].back()
                    : Vector3d(result.placement.x, result.placement.y, result.placement.z);
                capturePlanningInfo = makePlanningInfo(i, MotionTaskType::CAMERA_CAPTURE, captureBase, weldingManipulability, captureProjectorPose);
            }
        }

        for (size_t z = 0; z < traj[i].size(); z++) {
            SetJointAnglesDeg(traj[i][z],
                              base_traj[i][z][0],
                              base_traj[i][z][1],
                              base_traj[i][z][2],
                              MotionMode::JOINT_AND_BASE,
                              i < trajectory_types.size() ? trajectory_types[i] : MotionTaskType::TRANSITION,
                              nullptr,
                              nullptr,
                              nullptr,
                              QString());
        }

        if (is_view_segment) {
            const Vector6d& capture_joints = traj[i].back();
            const Vector3d& capture_base = base_traj[i].back();
            SetJointAnglesDeg(capture_joints,
                              capture_base[0],
                              capture_base[1],
                              capture_base[2],
                              MotionMode::JOINT_AND_BASE,
                              MotionTaskType::CAMERA_CAPTURE,
                              captureProjectorPose,
                              captureFrustumPointsPtr,
                              captureCoveredVerticesPtr,
                              capturePlanningInfo);
        }

        // 执行目标路径（如果存在）
        if (i >= 3) { // 从第3段开始（抬升段0 + RRT规划段1 + 基座移动段2）
            size_t logical_i = i - 3; // 减去抬升段、RRT规划段和基座移动段

            if (logical_i >= view.size()) {
                size_t m = logical_i - view.size();
                for (const auto& q : motions[m]) {
                    SetJointAnglesDeg(q, 0, 0, 0, MotionMode::JOINT_ONLY, MotionTaskType::WELDING);
                }
            }
        }

        if (isRecordingTrajectory_) {
            std::vector<Vector6d> recordedSegmentMotions;
            if (i < 3) { // 抬升段、RRT规划段和基座移动段没有目标路径
                recordedSegmentMotions = {};
            } else {
                size_t logical_i = i - 3;
                if (logical_i >= view.size()) {
                    recordedSegmentMotions = motions[logical_i - view.size()];
                } else {
                    recordedSegmentMotions = {};
                }
            }

            recordTrajectorySegment(traj[i],
                                    base_traj[i],
                                    true,
                                    i < trajectory_types.size() ? trajectory_types[i] : MotionTaskType::TRANSITION,
                                    recordedSegmentMotions,
                                    nullptr,
                                    nullptr,
                                    nullptr,
                                    QString());

            if (is_view_segment) {
                recordTrajectorySegment({traj[i].back()},
                                        {base_traj[i].back()},
                                        true,
                                        MotionTaskType::CAMERA_CAPTURE,
                                        {},
                                        captureProjectorPose,
                                        captureFrustumPointsPtr,
                                        captureCoveredVerticesPtr,
                                        capturePlanningInfo);
            }
        }
    }

    if (!has_successful_plan) {
        QMessageBox::warning(this, u8"规划失败", u8"所有路径段规划都失败");
        return;
    }
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_5_clicked()
{
    showToolPath_ = !showToolPath_;

    // 根据当前状态更新按钮文本
    if (showToolPath_) {
        ui->toolButton_5->setText("隐藏路径");

        // 如果路径可视化不存在但路径点足够，创建它
        if (!toolPathActor_ && toolPathPoints_.size() >= 2) {
            createToolPathVisualization();
        }

        // 如果路径可视化存在，显示它
        if (toolPathActor_) {
            toolPathActor_->SetVisibility(true);
        } else {
            std::cout << "警告：没有可显示的路径数据" << std::endl;
            showToolPath_ = false;
            ui->toolButton_5->setText("显示路径");
        }
    } else {
        ui->toolButton_5->setText("显示路径");

        // 隐藏路径
        if (toolPathActor_) {
            toolPathActor_->SetVisibility(false);
        }
    }

    // 重新渲染
    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_6_clicked()
{
    toolPathPoints_.clear();

    if (toolPathActor_) {
        renderer->RemoveActor(toolPathActor_);
        renderer2->RemoveActor(toolPathActor_);
        toolPathActor_ = nullptr;
    }

    // 重置显示状态
    showToolPath_ = false;
    ui->toolButton_5->setText("显示路径");

    renderer->GetRenderWindow()->Render();
    renderer2->GetRenderWindow()->Render();

    std::cout << "焊枪路径已清除" << std::endl;
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_7_clicked()
{
    startTrajectoryRecording();
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_8_clicked()
{
    stopTrajectoryRecording();
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_9_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                   u8"保存轨迹文件",
                                                   QDir::currentPath(),
                                                   u8"轨迹文件 (*.traj)");
    if (!filename.isEmpty()) {
        saveTrajectoryToFile(filename);
    }
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_10_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                   u8"加载轨迹文件",
                                                   QDir::currentPath(),
                                                   u8"轨迹文件 (*.traj)");
    if (!filename.isEmpty()) {
        loadTrajectoryFromFile(filename);
    }
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_11_clicked()
{
    replayTrajectory();
    ui->toolButton_13->setText(u8"演示暂停");
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_13_clicked()
{
    if (isPaused_) {
        resumeReplayTrajectory();
        ui->toolButton_13->setText(u8"演示暂停");
        return;
    }

    pauseReplayTrajectory();
    if (isPaused_) {
        ui->toolButton_13->setText(u8"继续演示");
    }
}

void OffLineProgrammingSimulationMainWindow::on_radioButton_clicked()
{
    currentWeldingType_ = WeldingType::PIPE_TO_PLATE;
}

void OffLineProgrammingSimulationMainWindow::on_radioButton_2_clicked()
{
    currentWeldingType_ = WeldingType::PLATE_TO_PLATE;
}

void OffLineProgrammingSimulationMainWindow::on_radioButton_3_clicked()
{
    currentWeldingType_ = WeldingType::PIPE_TO_PIPE;
}

void OffLineProgrammingSimulationMainWindow::on_toolButton_12_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                   u8"保存JSON数据文件",
                                                   QDir::currentPath(),
                                                   u8"JSON文件 (*.json)");
    if (!filename.isEmpty()) {
        if (!filename.endsWith(".json", Qt::CaseInsensitive)) {
            filename += ".json";
        }
        saveDataToJson(filename);
    }
}
