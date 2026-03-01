#ifndef MODELPROCESSING_H
#define MODELPROCESSING_H

#include <TopoDS_Shape.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <vector>
#include <tuple>
#include <string>
#include <BRep_Tool.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <TopoDS.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRepAdaptor_Surface.hxx>

#include "common.h"

class ModelProcessing
{
public:

    Point3D Point2Tuple(const gp_Pnt& pnt);

    bool PtsEqual(const Point3D& p1, const Point3D& p2, double tol = 1e-7);

    Normal3D CrossProduct(const Normal3D& a, const Normal3D& b);

    double DotProduct(const Normal3D& a, const Normal3D& b);

    Normal3D VectorSubtract(const Normal3D& a, const Normal3D& b);

    Normal3D VectorScale(const Normal3D& v, double scale);

    Normal3D ProjectVectorOntoPlane(const Normal3D& v, const Normal3D& plane_normal);

    std::unique_ptr<Normal3D> TangentAtParameter(const Handle(Geom_Curve)& curve_handle, double u);

    std::pair<std::vector<Point3D>, std::vector<double>> UniformSampleEdge(const TopoDS_Edge& edge, double step);

    std::vector<Point3D> ChordHeightSampleCurve(const Handle(Geom_Curve)& curve_handle, double f, double l, double max_dev);

    std::pair<std::vector<Point3D>, std::vector<double>> EqualChordLengthSampleCurve(const Handle(Geom_Curve)& curve_handle, double f, double l, double chord_length);

    std::pair<std::vector<Point3D>, std::vector<double>> DiscretizeEdgeByType(const TopoDS_Edge& edge, bool is_arc_or_spline, double linear_step = 1.0);

    std::tuple<Normal3D, bool, UVParams> NormalOnFaceAtPoint(const TopoDS_Face& face, const Point3D& point_xyz);

    Normal3D NormalizeVec(const Normal3D& v);

    Normal3D AddVecs(const Normal3D& a, const Normal3D& b);

    TopoDS_Edge MakeVectorEdge(const Point3D& p, const Normal3D& vec, double scale = 1.0);

    void GenerateDiscretePointsNormals(std::vector<EdgeInfo>& edges_info, double step);

    FacesInfo GetFacesInfo(const TopoDS_Edge& edge, const TopoDS_Shape& full_shape);

    std::pair<Point3D, Point3D> EdgeEndpoints(const TopoDS_Edge& edge);

    bool IsArcOrSpline(const TopoDS_Edge& edge);

    std::vector<EdgeInfo> ExtractEdgesFromStep(const TopoDS_Shape& shape);

    bool IsEdgeConvex(const TopoDS_Edge& edge, const FacesInfo &faces_info);

};

#endif // MODELPROCESSING_H
