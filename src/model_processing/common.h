#ifndef COMMON_H
#define COMMON_H

#include <cstddef>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <Eigen/Dense>
#include <vtkActor.h>
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

    Normal3D operator+(const Normal3D& other) const {
        return Normal3D(x + other.x, y + other.y, z + other.z);
    }

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
    int source_edge_idx;
    double parameter_begin;
    double parameter_end;
    Point3D segment_start_point;
    Point3D segment_end_point;
    std::vector<int> adjacent_face_owner_ids_a;
    std::vector<int> adjacent_face_owner_ids_b;
    std::vector<Point3D> sample_points;
    std::vector<std::tuple<Normal3D, Normal3D, Normal3D>> normals;

    EdgeInfo()
        : is_arc_or_spline(false),
          idx(0),
          source_edge_idx(-1),
          parameter_begin(0.0),
          parameter_end(0.0)
    {
    }
};

struct CoordinateSystem {
    Point3D origin;
    Normal3D x_axis;
    Normal3D y_axis;
    Normal3D z_axis;
    Matrix4x4 transform_matrix;
};

struct DHJoint {
    double a;
    double alpha;
    double d;
    double theta;
    std::string stlFile;
    double color[3];
    vtkSmartPointer<vtkAssembly> assembly;
    vtkSmartPointer<vtkActor> meshActor;
};

struct DHParameters {
    double a;
    double alpha;
    double d;
    double theta;
};

struct Bounds {
    std::vector<double> lb, ub;
    Bounds() {}
    Bounds(size_t dim, double lo, double hi) : lb(dim, lo), ub(dim, hi) {}
};

struct Particle {
    std::vector<double> x;
    std::vector<double> v;
    std::vector<double> pbest;
    double pbest_val;

    Particle() : pbest_val(std::numeric_limits<double>::infinity()) {}
};

#endif // COMMON_H
