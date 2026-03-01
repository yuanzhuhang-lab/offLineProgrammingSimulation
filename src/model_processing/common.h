#ifndef COMMON_H
#define COMMON_H

#include <TopoDS_Face.hxx>
#include <TopoDS_Edge.hxx>
#include <Eigen/Dense>
#include <vtkAssembly.h>
#include <vtkSmartPointer.h>

using Matrix4x4 = Eigen::Matrix<double, 4, 4>;
struct Point3D {
    double x, y, z;
    Point3D() : x(0), y(0), z(0) {}
    Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
};
struct Bounds3D {
    double xmin, xmax;
    double ymin, ymax;
    double zmin, zmax;
    Bounds3D() {
        xmin = ymin = zmin = std::numeric_limits<double>::quiet_NaN();
        xmax = ymax = zmax = std::numeric_limits<double>::quiet_NaN();
    }
};
struct Normal3D {
    double x, y, z;
    Normal3D() : x(0), y(0), z(0) {}
    Normal3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    // 向量加法
    Normal3D operator+(const Normal3D& other) const {
        return Normal3D(x + other.x, y + other.y, z + other.z);
    }

    // 标量乘法
    friend Normal3D operator*(double scalar, const Normal3D& vec) {
        return Normal3D(scalar * vec.x, scalar * vec.y, scalar * vec.z);
    }
};
struct UVParams {
    double u, v;
    UVParams() : u(0), v(0) {}
    UVParams(double u_, double v_) : u(u_), v(v_) {}
};
struct FacesInfo {
    std::vector<TopoDS_Face> faces;
    int face_count;
    FacesInfo() : face_count(0) {}
};
struct EdgeInfo {
    TopoDS_Edge edge;
    std::pair<Point3D, Point3D> ends;
    bool is_arc_or_spline;
    int idx;
    FacesInfo faces_info;
    std::vector<Point3D> sample_points;
    std::vector<std::tuple<Normal3D, Normal3D, Normal3D>> normals;

    EdgeInfo() : is_arc_or_spline(false), idx(0) {}
};

struct CoordinateSystem {
    Point3D origin;     // 原点坐标
    Normal3D x_axis;     // X轴归一化向量
    Normal3D y_axis;     // Y轴归一化向量
    Normal3D z_axis;     // Z轴归一化向量
    Matrix4x4 transform_matrix; // 相对于世界坐标系的变换矩阵
};

struct DHJoint {
    double a;      // link length
    double alpha;  // link twist (radians)
    double d;      // link offset
    double theta;  // joint angle (radians) -- 初始值
    std::string stlFile;
    double color[3];
    vtkSmartPointer<vtkAssembly> assembly; // 用于该关节的 assembly（相对父）
    vtkSmartPointer<vtkActor> meshActor;   // 仅 mesh
};

struct DHParameters {
    double a;     // 连杆长度
    double alpha; // 连杆扭角
    double d;     // 连杆偏距
    double theta; // 关节角度
};

struct Bounds { // 定义 Bounds 结构体用于保存每维上下界
    std::vector<double> lb, ub; // 成员：每维下界和上界向量
    Bounds() {} // 默认构造函数（空）
    Bounds(size_t dim, double lo, double hi) : lb(dim, lo), ub(dim, hi) {} // 按维度与范围初始化构造函数
};

struct Particle { // 定义粒子结构体
    std::vector<double> x;    // 位置向量 x
    std::vector<double> v;    // 速度向量 v
    std::vector<double> pbest; // 个体最优位置 pbest
    double pbest_val; // 个体最优值 pbest_val

    Particle() : pbest_val(std::numeric_limits<double>::infinity()) {} // 构造函数：初始化 pbest_val 为 +inf
};

#endif // COMMON_H
