#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <fcl/fcl.h>
#include <memory>
#include <vector>
#include <set>

#include <vtkSmartPointer.h>
#include <vtkSTLReader.h>
#include <vtkPolyData.h>
#include <vtkTriangleFilter.h>
#include <vtkDataArray.h>

#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <IVtkVTK_ShapeData.hxx>
#include <IVtkOCC_ShapeMesher.hxx>
#include <BRepBuilderAPI_Transform.hxx>

enum class EnvDetectMask
{
    GROUND_ONLY,
    STEP_ONLY
};

class CollisionDetection
{
public:
    static CollisionDetection& getInstance();

    CollisionDetection(const CollisionDetection&) = delete;
    CollisionDetection& operator=(const CollisionDetection&) = delete;

    void updateRobotPose(const std::vector<Eigen::Matrix4d>& matrix);

    bool checkCollision();

    void setEnvironmentMode(EnvDetectMask mode);

private:
    CollisionDetection();
    ~CollisionDetection();

    struct CollisionData
    {
        bool done = false;
        bool collision = false;
        fcl::CollisionRequestf request;
        fcl::CollisionResultf result;
        CollisionDetection* self = nullptr;

        CollisionData()
        {
            request.enable_contact = true;
            request.num_max_contacts = 1;
        }
    };

    enum PartIndex {
        PART_BASE = 0,
        PART_LINK1,
        PART_LINK2,
        PART_LINK3,
        PART_LINK4,
        PART_LINK5,
        PART_TORCH,
        PART_CAMERA,
        PART_WORKPIECE
    };

    EnvDetectMask env_detect_mask_ = EnvDetectMask::STEP_ONLY;

    std::set<std::pair<int, int>> adjacentPairs;

    int getPartIndex(fcl::CollisionObjectf* obj);

    bool isAdjacent(fcl::CollisionObjectf* o1, fcl::CollisionObjectf* o2);

    std::shared_ptr<fcl::CollisionObjectf> base_obj;
    std::shared_ptr<fcl::CollisionObjectf> link1_obj;
    std::shared_ptr<fcl::CollisionObjectf> link2_obj;
    std::shared_ptr<fcl::CollisionObjectf> link3_obj;
    std::shared_ptr<fcl::CollisionObjectf> link4_obj;
    std::shared_ptr<fcl::CollisionObjectf> link5_obj;
    std::shared_ptr<fcl::CollisionObjectf> torch_obj;
    std::shared_ptr<fcl::CollisionObjectf> camera_obj;
    std::shared_ptr<fcl::CollisionObjectf> workpiece_obj;
    std::shared_ptr<fcl::CollisionObjectf> ground_obj;

    std::shared_ptr<fcl::DynamicAABBTreeCollisionManagerf> static_manager;
    std::shared_ptr<fcl::DynamicAABBTreeCollisionManagerf> dynamic_manager;

    void init();

    static std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>> createBVHFromSTL(const std::string& filename);

    static std::shared_ptr<fcl::BVHModel<fcl::OBBRSSf>> createBVHFromSTEP(const std::string& filename);

    static bool collisionCallback(fcl::CollisionObjectf* o1, fcl::CollisionObjectf* o2, void* data);

    fcl::Transform3f matrix4dToTransform3f(const Eigen::Matrix4d& matrix);
};

#endif // COLLISIONDETECTION_H
