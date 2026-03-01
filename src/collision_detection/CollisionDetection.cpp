#include "CollisionDetection.h"

CollisionDetection& CollisionDetection::getInstance()
{
    static CollisionDetection instance;
    return instance;
}

CollisionDetection::CollisionDetection()
{
    static_manager  = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    dynamic_manager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
    init();

}

CollisionDetection::~CollisionDetection()
{
    // 释放userData分配的内存
    if (base_obj && base_obj->getUserData()) {
        delete static_cast<int*>(base_obj->getUserData());
    }
    if (link1_obj && link1_obj->getUserData()) {
        delete static_cast<int*>(link1_obj->getUserData());
    }
    if (link2_obj && link2_obj->getUserData()) {
        delete static_cast<int*>(link2_obj->getUserData());
    }
    if (link3_obj && link3_obj->getUserData()) {
        delete static_cast<int*>(link3_obj->getUserData());
    }
    if (link4_obj && link4_obj->getUserData()) {
        delete static_cast<int*>(link4_obj->getUserData());
    }
    if (link5_obj && link5_obj->getUserData()) {
        delete static_cast<int*>(link5_obj->getUserData());
    }
    if (torch_obj && torch_obj->getUserData()) {
        delete static_cast<int*>(torch_obj->getUserData());
    }
    if (camera_obj && camera_obj->getUserData()) {
        delete static_cast<int*>(camera_obj->getUserData());
    }
    if (workpiece_obj && workpiece_obj->getUserData()) {
        delete static_cast<int*>(workpiece_obj->getUserData());
    }
}

void CollisionDetection::updateRobotPose(const std::vector<Eigen::Matrix4d>& matrix)
{   
    if (matrix.size() < 7) return;

    std::vector<Eigen::Matrix4d> localTransforms = {
        // base
        (Eigen::Matrix4d() << 1.0, 0.0, 0.0, -1.5,
                             0.0, 0.0, 1.0, -60.85,
                             0.0, -1.0, 0.0, 82.5,
                             0.0, 0.0, 0.0, 1.0).finished(),
        // link1
        (Eigen::Matrix4d() << 0.0, 0.0, -1.0, -135.0,
                             0.0, 1.0, 0.0, -97.25,
                             1.0, 0.0, 0.0, -0.5,
                             0.0, 0.0, 0.0, 1.0).finished(),
        // link2
        (Eigen::Matrix4d() << 0.0, -1.0, 0.0, -314.75,
                             0.0, 0.0, -1.0, 7.5,
                             1.0, 0.0, 0.0, 118.25,
                             0.0, 0.0, 0.0, 1.0).finished(),
        // link3
        (Eigen::Matrix4d() << 0.0, 1.0, 0.0, -115.75,
                             0.0, 0.0, -1.0, 22.75,
                             -1.0, 0.0, 0.0, 97.25,
                             0.0, 0.0, 0.0, 1.0).finished(),
        // link4
        (Eigen::Matrix4d() << 0.0, 1.0, 0.0, -0.5,
                             -1.0, 0.0, 0.0, 155.0,
                             0.0, 0.0, 1.0, -5.0,
                             0.0, 0.0, 0.0, 1.0).finished(),
        // link5
        (Eigen::Matrix4d() << 0.0, 0.0, -1.0, 0.0,
                             0.0, 1.0, 0.0, 0.0,
                             1.0, 0.0, 0.0, 19.5,
                             0.0, 0.0, 0.0, 1.0).finished(),
        // camera2tool
        (Eigen::Matrix4d() << 1.0, 0.0, 0.0, 52.4276,
                             0.0, 0.866, 0.5, 40.955234,
                             0.0, -0.5, 0.866, -324.531252,
                             0.0, 0.0, 0.0, 1.0).finished()
    };

    if (base_obj) {
         base_obj->setTransform(matrix4dToTransform3f(matrix[0] * localTransforms[0]));
         base_obj->computeAABB();
     }

     if (link1_obj) {
         link1_obj->setTransform(matrix4dToTransform3f(matrix[1] * localTransforms[1]));
         link1_obj->computeAABB();
     }

     if (link2_obj) {
         link2_obj->setTransform(matrix4dToTransform3f(matrix[2] * localTransforms[2]));
         link2_obj->computeAABB();
     }

     if (link3_obj) {
         link3_obj->setTransform(matrix4dToTransform3f(matrix[3] * localTransforms[3]));
         link3_obj->computeAABB();
     }

     if (link4_obj) {
         link4_obj->setTransform(matrix4dToTransform3f(matrix[4] * localTransforms[4]));
         link4_obj->computeAABB();
     }

     if (link5_obj) {
         link5_obj->setTransform(matrix4dToTransform3f(matrix[5] * localTransforms[5]));
         link5_obj->computeAABB();
     }

     if (torch_obj) {
         torch_obj->setTransform(matrix4dToTransform3f(matrix[6]));
         torch_obj->computeAABB();
     }

     if (camera_obj) {
         camera_obj->setTransform(matrix4dToTransform3f(matrix[6] * localTransforms[6]));
         camera_obj->computeAABB();
     }

     dynamic_manager->update();
}

bool CollisionDetection::checkCollision()
{
    CollisionData cdata;
    cdata.self = this;

    if (workpiece_obj) {
        static_manager->collide(
            dynamic_manager.get(),
            &cdata,
            &CollisionDetection::collisionCallback);
    }

    if (cdata.collision)
        return true;

    dynamic_manager->collide(
        &cdata,
        &CollisionDetection::collisionCallback);

    return cdata.collision;
}

void CollisionDetection::setEnvironmentMode(EnvDetectMask mode)
{
    env_detect_mask_ = mode;
}

int CollisionDetection::getPartIndex(fcl::CollisionObjectf *obj)
{
    if (!obj->getUserData()) return -1;
    return *static_cast<int*>(obj->getUserData());
}

bool CollisionDetection::isAdjacent(fcl::CollisionObjectf *o1, fcl::CollisionObjectf *o2)
{
    int idx1 = getPartIndex(o1);
    int idx2 = getPartIndex(o2);

    if (idx1 == -1 || idx2 == -1) return false;

    if (adjacentPairs.count({std::min(idx1, idx2), std::max(idx1, idx2)}) > 0) {
        return true;
    }

    return false;
}

void CollisionDetection::init()
{
    adjacentPairs.insert({PART_BASE, PART_LINK1});
    adjacentPairs.insert({PART_LINK1, PART_LINK2});
    adjacentPairs.insert({PART_LINK2, PART_LINK3});
    adjacentPairs.insert({PART_LINK3, PART_LINK4});
    adjacentPairs.insert({PART_LINK4, PART_LINK5});
    adjacentPairs.insert({PART_LINK5, PART_TORCH});
    adjacentPairs.insert({PART_TORCH, PART_CAMERA});

    fcl::Vector3f side1(303.0f, 165.0f, 428.0f);
    auto box1 = std::make_shared<fcl::Boxf>(side1); 
    fcl::Transform3f pose1 = fcl::Transform3f::Identity();
    base_obj = std::make_shared<fcl::CollisionObjectf>(box1, pose1);
    int* index1 = new int(PART_BASE);
    base_obj->setUserData(index1);

    fcl::Vector3f side2(359.0f, 375.5f, 467.0f);
    auto box2 = std::make_shared<fcl::Boxf>(side2); 
    fcl::Transform3f pose2 = fcl::Transform3f::Identity();
    link1_obj = std::make_shared<fcl::CollisionObjectf>(box2, pose2);
    int* index2 = new int(PART_LINK1);
    link1_obj->setUserData(index2);

    fcl::Vector3f side3(157.5f, 781.5f, 197.0f);
    auto box3 = std::make_shared<fcl::Boxf>(side3);
    fcl::Transform3f pose3 = fcl::Transform3f::Identity();
    link2_obj = std::make_shared<fcl::CollisionObjectf>(box3, pose3);
    int* index3 = new int(PART_LINK2);
    link2_obj->setUserData(index3);

    fcl::Vector3f side4(335.5f, 393.5f, 340.5f);
    auto box4 = std::make_shared<fcl::Boxf>(side4);
    fcl::Transform3f pose4 = fcl::Transform3f::Identity();
    link3_obj = std::make_shared<fcl::CollisionObjectf>(box4, pose4);
    int* index4 = new int(PART_LINK3);
    link3_obj->setUserData(index4);

    fcl::Vector3f side5(435.0f, 155.0f, 177.0f);
    auto box5 = std::make_shared<fcl::Boxf>(side5);
    fcl::Transform3f pose5 = fcl::Transform3f::Identity();
    link4_obj = std::make_shared<fcl::CollisionObjectf>(box5, pose5);
    int* index5 = new int(PART_LINK4);
    link4_obj->setUserData(index5);

    fcl::Vector3f side6(149.0f, 128.0f, 128.0f);
    auto box6 = std::make_shared<fcl::Boxf>(side6);
    fcl::Transform3f pose6 = fcl::Transform3f::Identity();
    link5_obj = std::make_shared<fcl::CollisionObjectf>(box6, pose6);
    int* index6 = new int(PART_LINK5);
    link5_obj->setUserData(index6);

    {
        auto model = createBVHFromSTL(
            "D:/Qt_develop/offLineProgrammingSimulation/model/weldingRobot/tool.STL");

        if (model) {
            torch_obj = std::make_shared<fcl::CollisionObjectf>(
                model, fcl::Transform3f::Identity());
            torch_obj->setUserData(new int(PART_TORCH));
            torch_obj->computeAABB();
        } else {
            torch_obj.reset(); // ⭐ 明确失败
        }
    }

    {
        auto model = createBVHFromSTL(
            "D:/Qt_develop/offLineProgrammingSimulation/model/weldingRobot/camera.STL");

        if (model) {
            camera_obj = std::make_shared<fcl::CollisionObjectf>(
                model, fcl::Transform3f::Identity());
            camera_obj->setUserData(new int(PART_CAMERA));
            camera_obj->computeAABB();
        } else {
            camera_obj.reset();
        }
    }

    {
            auto box = std::make_shared<fcl::Boxf>(15000.0f,15000.0f,230.0f);
            ground_obj = std::make_shared<fcl::CollisionObjectf>(
                box, fcl::Transform3f::Identity());
            ground_obj->setUserData(new int(PART_WORKPIECE));

        }

        {
            auto model = createBVHFromSTEP(
                "D:/Qt_develop/offLineProgrammingSimulation/build/hanjie.STEP");
            if (model) {
                workpiece_obj = std::make_shared<fcl::CollisionObjectf>(
                    model, fcl::Transform3f::Identity());
                workpiece_obj->setUserData(new int(PART_WORKPIECE));
                workpiece_obj->computeAABB();

            } else {
                workpiece_obj.reset();
            }
        }

    if (workpiece_obj && ground_obj) {
        static_manager->registerObject(ground_obj.get());
        static_manager->registerObject(workpiece_obj.get());
        static_manager->setup();
    }

    // 动态物体
    if (base_obj)  dynamic_manager->registerObject(base_obj.get());
    if (link1_obj) dynamic_manager->registerObject(link1_obj.get());
    if (link2_obj) dynamic_manager->registerObject(link2_obj.get());
    if (link3_obj) dynamic_manager->registerObject(link3_obj.get());
    if (link4_obj) dynamic_manager->registerObject(link4_obj.get());
    if (link5_obj) dynamic_manager->registerObject(link5_obj.get());
    if (torch_obj) dynamic_manager->registerObject(torch_obj.get());
    if (camera_obj) dynamic_manager->registerObject(camera_obj.get());

    env_detect_mask_ = EnvDetectMask::STEP_ONLY;

    dynamic_manager->setup();

}

std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf> > CollisionDetection::createBVHFromSTL(const std::string &filename)
{
    vtkSmartPointer<vtkSTLReader> reader =
        vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();

    vtkPolyData* polyData = reader->GetOutput();
    if (!polyData || polyData->GetNumberOfPoints() == 0) {
        std::cerr << "[BVH] STL empty." << std::endl;
        return nullptr;
    }

    /* ========= 2. 三角化 ========= */
    vtkSmartPointer<vtkTriangleFilter> triFilter =
        vtkSmartPointer<vtkTriangleFilter>::New();
    triFilter->SetInputData(polyData);
    triFilter->PassLinesOff();
    triFilter->PassVertsOff();
    triFilter->Update();

    vtkPolyData* mesh = triFilter->GetOutput();
    if (!mesh || mesh->GetNumberOfCells() == 0) {
        std::cerr << "[BVH] No triangles." << std::endl;
        return nullptr;
    }

    vtkIdType numPoints = mesh->GetNumberOfPoints();
    vtkIdType numCells  = mesh->GetNumberOfCells();

    /* ========= 3. 顶点 ========= */
    std::vector<fcl::Vector3f> vertices;
    vertices.reserve(numPoints);

    for (vtkIdType i = 0; i < numPoints; ++i) {
        double p[3];
        mesh->GetPoint(i, p);
        vertices.emplace_back(
            static_cast<float>(p[0]),
            static_cast<float>(p[1]),
            static_cast<float>(p[2])
        );
    }

    /* ========= 4. 三角形索引 ========= */
    std::vector<fcl::Triangle> triangles;
    triangles.reserve(numCells);

    for (vtkIdType i = 0; i < numCells; ++i) {
        vtkCell* cell = mesh->GetCell(i);
        if (!cell || cell->GetCellType() != VTK_TRIANGLE)
            continue;

        vtkIdType i0 = cell->GetPointId(0);
        vtkIdType i1 = cell->GetPointId(1);
        vtkIdType i2 = cell->GetPointId(2);

        const auto& a = vertices[i0];
        const auto& b = vertices[i1];
        const auto& c = vertices[i2];

        // 退化三角形检查
        if ((b - a).cross(c - a).norm() < 1e-8f)
            continue;

        triangles.emplace_back(i0, i1, i2);
    }

    if (triangles.empty()) {
        std::cerr << "[BVH] All triangles degenerate." << std::endl;
        return nullptr;
    }

    /* ========= 5. BVH 构建（关键） ========= */
    auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSf>>();

    model->beginModel();
    model->addSubModel(vertices, triangles);
    model->endModel();
    model->computeLocalAABB();

    std::cout << "[BVH] Build success: "
              << triangles.size() << " triangles" << std::endl;

    return model;
}

std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf> > CollisionDetection::createBVHFromSTEP(const std::string &filename)
{
    /* ==================== 1. 读取 STEP ==================== */
        STEPControl_Reader stpReader;
        IFSelect_ReturnStatus status = stpReader.ReadFile(filename.c_str());
        if (status != IFSelect_RetDone) {
            std::cerr << "[STEP] Read failed: " << filename << std::endl;
            return nullptr;
        }

        if (stpReader.TransferRoots() <= 0) {
            std::cerr << "[STEP] No transferable roots." << std::endl;
            return nullptr;
        }

        TopoDS_Shape shape = stpReader.OneShape();

        BRepMesh_IncrementalMesh mesher(shape, 0.1);
        mesher.Perform();

        std::vector<fcl::Vector3f> vertices;
        std::vector<fcl::Triangle> triangles;

        TopExp_Explorer faceExp(shape, TopAbs_FACE);
        int vertexOffset = 0;

        for (; faceExp.More(); faceExp.Next()) {
            TopoDS_Face face = TopoDS::Face(faceExp.Current());

            TopLoc_Location loc;
            Handle(Poly_Triangulation) triangulation =
                BRep_Tool::Triangulation(face, loc);

            if (triangulation.IsNull())
                continue;

            const gp_Trsf& T = loc.Transformation();
            bool reverse = (face.Orientation() == TopAbs_REVERSED);

            int nbNodes = triangulation->NbNodes();
            int baseIndex = vertexOffset;

            /* ---- 顶点 ---- */
            for (int i = 1; i <= nbNodes; ++i) {
                gp_Pnt p = triangulation->Node(i).Transformed(T);
                vertices.emplace_back(
                    static_cast<float>(p.X()),
                    static_cast<float>(p.Y()),
                    static_cast<float>(p.Z())
                );
                ++vertexOffset;
            }

            /* ---- 三角形 ---- */
            int nbTriangles = triangulation->NbTriangles();
            for (int i = 1; i <= nbTriangles; ++i) {
                int n1, n2, n3;
                triangulation->Triangle(i).Get(n1, n2, n3);

                int i0 = baseIndex + n1 - 1;
                int i1 = baseIndex + n2 - 1;
                int i2 = baseIndex + n3 - 1;

                const auto& a = vertices[i0];
                const auto& b = vertices[i1];
                const auto& c = vertices[i2];

                // 退化三角形过滤
                if ((b - a).cross(c - a).norm() < 1e-8f)
                    continue;

                if (reverse)
                    triangles.emplace_back(i0, i2, i1);
                else
                    triangles.emplace_back(i0, i1, i2);
            }
        }

        if (vertices.empty() || triangles.empty()) {
            std::cerr << "[STEP] Empty mesh after triangulation." << std::endl;
            return nullptr;
        }

        /* ==================== 5. 构建 BVH（关键） ==================== */
        auto model = std::make_shared<fcl::BVHModel<fcl::OBBRSSf>>();

        model->beginModel();
        model->addSubModel(vertices, triangles);
        model->endModel();
        model->computeLocalAABB();

        std::cout << "[STEP BVH] Build success: "
                  << vertices.size() << " vertices, "
                  << triangles.size() << " triangles." << std::endl;

        return model;
}

bool CollisionDetection::collisionCallback(fcl::CollisionObjectf *o1, fcl::CollisionObjectf *o2, void *data)
{
    auto* cdata = static_cast<CollisionData*>(data);
    if (cdata->done)
        return true;

    CollisionDetection* self = cdata->self;

    int idx1 = self ? self->getPartIndex(o1) : -1;
    int idx2 = self ? self->getPartIndex(o2) : -1;

    bool o1_is_ground = (o1 == self->ground_obj.get());
    bool o2_is_ground = (o2 == self->ground_obj.get());
    bool o1_is_step   = (o1 == self->workpiece_obj.get());
    bool o2_is_step   = (o2 == self->workpiece_obj.get());

    if (self->env_detect_mask_ == EnvDetectMask::GROUND_ONLY)
    {
        if (o1_is_step || o2_is_step)
            return false;
    }
    else if (self->env_detect_mask_ == EnvDetectMask::STEP_ONLY)
    {
        if (o1_is_ground || o2_is_ground)
            return false;
    }

    // ⭐ adjacency 只用于机器人内部自碰撞
    bool bothRobotParts =
        idx1 != PART_WORKPIECE &&
        idx2 != PART_WORKPIECE &&
        idx1 >= 0 && idx2 >= 0;

    if (bothRobotParts && self->isAdjacent(o1, o2))
        return false;

    bool torchWithLink4Or5 =
        (idx1 == PART_TORCH && (idx2 == PART_LINK4 || idx2 == PART_LINK5)) ||
        (idx2 == PART_TORCH && (idx1 == PART_LINK4 || idx1 == PART_LINK5));

    if (torchWithLink4Or5) {
        return false; // 跳过焊枪与link4、link5的碰撞检测
    }

    bool hasDetectObj =
        (idx1 == PART_CAMERA) || (idx2 == PART_CAMERA);

    if (hasDetectObj)
    {
        int other =
            (idx1 == PART_CAMERA) ? idx2 : idx1;

        if (other == PART_LINK5)
            return false;
    }

    fcl::collide(o1, o2, cdata->request, cdata->result);

    if (cdata->result.isCollision()) {
        cdata->collision = true;
        cdata->done = true;
        return true;
    }

    return false;
}

fcl::Transform3f CollisionDetection::matrix4dToTransform3f(const Eigen::Matrix4d &matrix)
{
    fcl::Transform3f transform;

    // 设置旋转部分
    Eigen::Matrix3f rotation = matrix.block<3,3>(0,0).cast<float>();
    transform.linear() = rotation;

    // 设置平移部分
    Eigen::Vector3f translation = matrix.block<3,1>(0,3).cast<float>();
    transform.translation() = translation;

    return transform;
}




