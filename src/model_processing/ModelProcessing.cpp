#include "ModelProcessing.h"

Point3D ModelProcessing::Point2Tuple(const gp_Pnt &pnt)
{
    return Point3D(pnt.X(), pnt.Y(), pnt.Z());
}

bool ModelProcessing::PtsEqual(const Point3D &p1, const Point3D &p2, double tol)
{
    return ((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z)) <= tol*tol;
}

Normal3D ModelProcessing::CrossProduct(const Normal3D &a, const Normal3D &b)
{
    return Normal3D(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
                );
}

double ModelProcessing::DotProduct(const Normal3D &a, const Normal3D &b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Normal3D ModelProcessing::VectorSubtract(const Normal3D &a, const Normal3D &b)
{
    return Normal3D(a.x - b.x, a.y - b.y, a.z - b.z);
}

Normal3D ModelProcessing::VectorScale(const Normal3D &v, double scale)
{
    return Normal3D(v.x * scale, v.y * scale, v.z * scale);
}

Normal3D ModelProcessing::ProjectVectorOntoPlane(const Normal3D &v, const Normal3D &plane_normal)
{
    // 计算向量在平面法向量方向上的分量
    double dot = DotProduct(v, plane_normal);

    // 减去法向量方向的分量，得到平面上的投影
    Normal3D normal_component = VectorScale(plane_normal, dot);
    return VectorSubtract(v, normal_component);
}

std::unique_ptr<Normal3D> ModelProcessing::TangentAtParameter(const Handle(Geom_Curve)& curve_handle, double u)
{
    try {
        // 计算切线向量
        gp_Pnt point;
        gp_Vec deriv;
        curve_handle->D1(u, point, deriv);

        Normal3D tangent(deriv.X(), deriv.Y(), deriv.Z());

        // 归一化切线向量
        double mag = std::sqrt(tangent.x*tangent.x + tangent.y*tangent.y + tangent.z*tangent.z);

        if (mag < 1e-9) {
            return nullptr;
        }

        return std::make_unique<Normal3D>(
            tangent.x / mag,
            tangent.y / mag,
            tangent.z / mag
        );

    } catch (...) {
        return nullptr;
    }
}

std::pair<std::vector<Point3D>, std::vector<double>> ModelProcessing::UniformSampleEdge(const TopoDS_Edge &edge, double step)
{
    double f, l;
    Handle(Geom_Curve) curve_handle = BRep_Tool::Curve(edge, f, l);
    if (curve_handle.IsNull()) {
        return {};
    }

    BRepAdaptor_Curve adaptor(edge);
    std::vector<Point3D> pts;
    std::vector<double> params;

    try {
        GCPnts_UniformAbscissa length_estimator(adaptor, step);
        int n_pts = length_estimator.NbPoints();
        for (int i = 1; i <= n_pts; ++i) {
            double u = length_estimator.Parameter(i);
            gp_Pnt p;
            curve_handle->D0(u, p);
            pts.push_back(Point2Tuple(p));
            params.push_back(u);
        }
    } catch (...) {
        // 如果UniformAbscissa失败，使用均匀采样
        int n = std::max(2, static_cast<int>((l - f)/step) + 1);
        for (int i = 0; i <= n; ++i) {
            double u = f + (l - f) * (static_cast<double>(i) / n);
            gp_Pnt p;
            curve_handle->D0(u, p);
            pts.push_back(Point2Tuple(p));
            params.push_back(u);;
        }
    }

    if (params.size() > 0) {
        // 检查起点
        if (std::abs(params[0] - f) > 1e-10) {
            gp_Pnt p_start;
            curve_handle->D0(f, p_start);
            pts.insert(pts.begin(), Point2Tuple(p_start));
            params.insert(params.begin(), f);
        }

        // 检查终点
        if (std::abs(params.back() - l) > 1e-10) {
            gp_Pnt p_end;
            curve_handle->D0(l, p_end);
            pts.push_back(Point2Tuple(p_end));
            params.push_back(l);
        }
    }

    return std::make_pair(pts, params);
}

std::vector<Point3D> ModelProcessing::ChordHeightSampleCurve(const Handle(Geom_Curve)& curve_handle, double f, double l, double max_dev)
{
    std::vector<Point3D> pts;

    // 使用lambda表达式定义递归函数
    std::function<void(double, double, const gp_Pnt&, const gp_Pnt&)> subdiv;
    subdiv = [&](double u0, double u1, const gp_Pnt& p0, const gp_Pnt& p1) {
        double umid = 0.5 * (u0 + u1);
        gp_Pnt pmid;
        curve_handle->D0(umid, pmid);

        Point3D v0 = Point2Tuple(p0);
        Point3D v1 = Point2Tuple(p1);
        Point3D vm = Point2Tuple(pmid);

        double ux = v1.x - v0.x, uy = v1.y - v0.y, uz = v1.z - v0.z;
        double wx = vm.x - v0.x, wy = vm.y - v0.y, wz = vm.z - v0.z;
        double denom = ux*ux + uy*uy + uz*uz;

        double dist;
        if (denom == 0) {
            dist = std::sqrt((vm.x-v0.x)*(vm.x-v0.x) + (vm.y-v0.y)*(vm.y-v0.y) + (vm.z-v0.z)*(vm.z-v0.z));
        } else {
            double t = (ux*wx + uy*wy + uz*wz) / denom;
            double proj_x, proj_y, proj_z;
            if (t < 0) {
                proj_x = v0.x; proj_y = v0.y; proj_z = v0.z;
            } else if (t > 1) {
                proj_x = v1.x; proj_y = v1.y; proj_z = v1.z;
            } else {
                proj_x = v0.x + t*ux; proj_y = v0.y + t*uy; proj_z = v0.z + t*uz;
            }
            double dx = vm.x - proj_x, dy = vm.y - proj_y, dz = vm.z - proj_z;
            dist = std::sqrt(dx*dx + dy*dy + dz*dz);
        }

        if (dist <= max_dev) {
            pts.push_back(v0);
        } else {
            gp_Pnt pmid_copy(pmid.X(), pmid.Y(), pmid.Z());
            subdiv(u0, umid, p0, pmid_copy);
            subdiv(umid, u1, pmid_copy, p1);
        }
    };

    gp_Pnt p0, p1;
    curve_handle->D0(f, p0);
    curve_handle->D0(l, p1);
    subdiv(f, l, p0, p1);
    pts.push_back(Point2Tuple(p1));

    return pts;
}

std::pair<std::vector<Point3D>, std::vector<double>> ModelProcessing::EqualChordLengthSampleCurve(const Handle(Geom_Curve)& curve_handle, double f, double l, double chord_length)
{
    std::vector<Point3D> pts;
    std::vector<double> params;  // 存储对应的参数值

    // 获取曲线适配器来计算弧长
    BRepBuilderAPI_MakeEdge edge_maker(curve_handle, f, l);
    TopoDS_Edge edge = edge_maker.Edge();
    BRepAdaptor_Curve adaptor(edge);

    try {
        // 使用GCPnts_UniformAbscissa进行等弧长采样
        GCPnts_UniformAbscissa length_estimator(adaptor, chord_length);
        int n_pts = length_estimator.NbPoints();

        for (int i = 1; i <= n_pts; ++i) {
            double u = length_estimator.Parameter(i);
            gp_Pnt p;
            curve_handle->D0(u, p);
            pts.push_back(Point2Tuple(p));
            params.push_back(u);
        }

    } catch (...) {
        // 如果GCPnts_UniformAbscissa失败，使用手动计算方法
        // 首先估算曲线总长度
        double total_length = 0.0;
        int n_segments = std::max(10, static_cast<int>((l - f) / chord_length) * 10);
        gp_Pnt prev_point;
        curve_handle->D0(f, prev_point);

        for (int i = 1; i <= n_segments; ++i) {
            double u = f + (l - f) * (static_cast<double>(i) / n_segments);
            gp_Pnt current_point;
            curve_handle->D0(u, current_point);
            double dx = current_point.X() - prev_point.X();
            double dy = current_point.Y() - prev_point.Y();
            double dz = current_point.Z() - prev_point.Z();
            double segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);
            total_length += segment_length;
            prev_point = current_point;
        }

        // 根据总长度和弦长计算采样点数
        int n_samples = std::max(2, static_cast<int>(total_length / chord_length) + 1);
        double actual_chord_length = total_length / (n_samples - 1);

        // 重新采样
        pts.clear();
        params.clear();
        double accumulated_length = 0.0;
        gp_Pnt prev_point2;
        curve_handle->D0(f, prev_point2);
        pts.push_back(Point2Tuple(prev_point2));
        params.push_back(f);

        for (int i = 1; i <= n_segments; ++i) {
            double u = f + (l - f) * (static_cast<double>(i) / n_segments);
            gp_Pnt current_point;
            curve_handle->D0(u, current_point);

            double dx = current_point.X() - prev_point2.X();
            double dy = current_point.Y() - prev_point2.Y();
            double dz = current_point.Z() - prev_point2.Z();
            double segment_length = std::sqrt(dx*dx + dy*dy + dz*dz);
            accumulated_length += segment_length;

            // 检查是否达到采样点
            while (accumulated_length >= actual_chord_length && pts.size() < static_cast<size_t>(n_samples)) {
                // 线性插值找到精确的采样点
                double overshoot = accumulated_length - actual_chord_length;
                if (segment_length > 0) {
                    double ratio = 1.0 - (overshoot / segment_length);
                    double interp_u = f + (l - f) * ((i-1 + ratio) / n_segments);
                    gp_Pnt interp_point;
                    curve_handle->D0(interp_u, interp_point);
                    pts.push_back(Point2Tuple(interp_point));
                    params.push_back(interp_u);
                    accumulated_length -= actual_chord_length;
                } else {
                    break;
                }
            }

            prev_point2 = current_point;
        }

        // 确保包含终点
        if (pts.size() < static_cast<size_t>(n_samples)) {
            gp_Pnt end_point;
            curve_handle->D0(l, end_point);
            pts.push_back(Point2Tuple(end_point));
            params.push_back(l);
        }
    }

    return std::make_pair(pts, params);
}

std::pair<std::vector<Point3D>, std::vector<double>> ModelProcessing::DiscretizeEdgeByType(const TopoDS_Edge &edge, bool is_arc_or_spline, double linear_step)
{
    double f, l;
    Handle(Geom_Curve) curve_handle = BRep_Tool::Curve(edge, f, l);
    if (curve_handle.IsNull()) {
        return std::make_pair(std::vector<Point3D>(), std::vector<double>());
    }

    // 使用auto和decltype
    auto get_curve_type_name = [&]() -> std::string {
        try {
            return std::string(curve_handle->DynamicType()->Name());
        } catch (...) {
            return "";
        }
    };

    std::string tname = get_curve_type_name();
    if (!is_arc_or_spline || tname.find("Geom_Line") != std::string::npos) {
        return UniformSampleEdge(edge, linear_step);
    } else {
        return EqualChordLengthSampleCurve(curve_handle, f, l, linear_step);
    }
}

std::tuple<Normal3D, bool, UVParams> ModelProcessing::NormalOnFaceAtPoint(const TopoDS_Face &face, const Point3D &point_xyz)
{
    Handle(Geom_Surface) surf = BRep_Tool::Surface(face);
    gp_Pnt pnt(point_xyz.x, point_xyz.y, point_xyz.z);
    if (surf.IsNull()) {
        std::cerr << "Error: surf is null (BRep_Tool::Surface returned null)\n";
        return {Normal3D(), false, UVParams()};
    }

    GeomAPI_ProjectPointOnSurf proj(pnt, surf);

    double u, v;
    proj.Parameters(1, u, v);

    try {
        gp_Pnt P;
        gp_Vec du, dv;
        surf->D1(u, v, P, du, dv);

        double cx = du.Y()*dv.Z() - du.Z()*dv.Y();
        double cy = du.Z()*dv.X() - du.X()*dv.Z();
        double cz = du.X()*dv.Y() - du.Y()*dv.X();
        double mag = std::sqrt(cx*cx + cy*cy + cz*cz);

        if (mag < 1e-9) {
            return {Normal3D(), false, UVParams()};
        }

        try {
            TopAbs_Orientation orient = face.Orientation();
            if (orient == TopAbs_REVERSED || orient == TopAbs_INTERNAL) {
                cx = -cx; cy = -cy; cz = -cz;
            }
        } catch (...) {
            // 忽略方向异常
        }

        return {Normal3D(cx, cy, cz), true, UVParams(u, v)};
    } catch (...) {
        return {Normal3D(), false, UVParams()};
    }
    return {Normal3D(), false, UVParams()};
}

Normal3D ModelProcessing::NormalizeVec(const Normal3D &v)
{
    double mag = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (mag == 0) {
        return Normal3D(0.0, 0.0, 0.0);
    }
    return Normal3D(v.x/mag, v.y/mag, v.z/mag);
}

Normal3D ModelProcessing::AddVecs(const Normal3D &a, const Normal3D &b)
{
    return Normal3D(a.x + b.x, a.y + b.y, a.z + b.z);
}

TopoDS_Edge ModelProcessing::MakeVectorEdge(const Point3D &p, const Normal3D &vec, double scale)
{
    gp_Pnt start(p.x, p.y, p.z);
    gp_Pnt end(p.x + vec.x*scale, p.y + vec.y*scale, p.z + vec.z*scale);
    return BRepBuilderAPI_MakeEdge(start, end).Edge();
}

void ModelProcessing::GenerateDiscretePointsNormals(std::vector<EdgeInfo> &edges_info, double step)
{
    // 使用range-based for loop和auto
    for (auto& einfo : edges_info) {
        std::vector<Point3D> sample_points;
        std::vector<std::tuple<Normal3D, Normal3D, Normal3D>> normals_list;

        std::vector<Point3D> pts;
        std::vector<double> params;
        std::tie(pts, params) = DiscretizeEdgeByType(einfo.edge, einfo.is_arc_or_spline, step);
        sample_points = pts;

        auto& faces = einfo.faces_info.faces;
        if (faces.size() < 2) {
            continue;
        }

        double f, l;
        Handle(Geom_Curve) curve_handle = BRep_Tool::Curve(einfo.edge, f, l);


        // 使用auto和结构化绑定
        for (size_t idx = 0; idx < pts.size(); ++idx) {
            const auto& p = pts[idx];
            double u = params[idx];

            // 1. 计算切线向量
            auto tangent_ptr = TangentAtParameter(curve_handle, u);
            if (!tangent_ptr) {
                normals_list.push_back(std::make_tuple(Normal3D(), Normal3D(), Normal3D()));
                continue;
            }
            Normal3D tangent = *tangent_ptr;

            Normal3D n1, n2;
            bool ok1, ok2;
            UVParams uv1, uv2;

            std::tie(n1, ok1, uv1) = NormalOnFaceAtPoint(faces[0], p);
            std::tie(n2, ok2, uv2) = NormalOnFaceAtPoint(faces[1], p);

            if (!ok2 || !ok1) {
                normals_list.push_back(std::make_tuple(Normal3D(), Normal3D(), Normal3D()));
                continue;
            }

            auto n1_norm = NormalizeVec(n1);
            auto n2_norm = NormalizeVec(n2);
            auto sum_vec = AddVecs(n1_norm, n2_norm);

            double sum_mag = std::sqrt(sum_vec.x*sum_vec.x + sum_vec.y*sum_vec.y + sum_vec.z*sum_vec.z);

            Normal3D n_comb;
            if (sum_mag < 1e-8) {
                n_comb = n1_norm;
            } else {
                n_comb = NormalizeVec(sum_vec);
            }

            // 4. 将合成向量投影到与切线向量垂直的平面上
            Normal3D n_comb_proj = ProjectVectorOntoPlane(n_comb, tangent);
            double n_comb_proj_mag = std::sqrt(n_comb_proj.x*n_comb_proj.x +
                                               n_comb_proj.y*n_comb_proj.y +
                                               n_comb_proj.z*n_comb_proj.z);
            Normal3D n_comb_proj_norm;

            if (n_comb_proj_mag < 1e-8) {
                n_comb_proj_norm = Normal3D(0.0, 0.0, 0.0);
            } else {
                n_comb_proj_norm = NormalizeVec(n_comb_proj);
            }

            // 5. 计算垂直于切线向量和投影向量的向量（满足右手定则）
            Normal3D n_perp = CrossProduct(n_comb_proj_norm, tangent);
            double n_perp_mag = std::sqrt(n_perp.x*n_perp.x + n_perp.y*n_perp.y + n_perp.z*n_perp.z);
            Normal3D n_perp_norm;

            if (n_perp_mag < 1e-8) {
                n_perp_norm = Normal3D(0.0, 0.0, 0.0);
            } else {
                n_perp_norm = NormalizeVec(n_perp);
            }

            // 6. 归一化所有向量并存储三个向量：切线、投影、垂直
            Normal3D tangent_norm = NormalizeVec(tangent);
            Normal3D n_comb_proj_norm_final = NormalizeVec(n_comb_proj_norm);
            Normal3D n_perp_norm_final = NormalizeVec(n_perp_norm);

            normals_list.push_back(std::make_tuple(tangent_norm, n_perp_norm_final, n_comb_proj_norm_final));
        }

        einfo.sample_points = sample_points;
        einfo.normals = normals_list;
    }
}

FacesInfo ModelProcessing::GetFacesInfo(const TopoDS_Edge &edge, const TopoDS_Shape &full_shape)
{
    FacesInfo faces_info;

    TopExp_Explorer face_explorer(full_shape, TopAbs_FACE);
    while (face_explorer.More()) {
        TopoDS_Face face = TopoDS::Face(face_explorer.Current());
        TopExp_Explorer edge_in_face(face, TopAbs_EDGE);
        bool found = false;

        while (edge_in_face.More()) {
            TopoDS_Edge e2 = TopoDS::Edge(edge_in_face.Current());
            if (e2.IsSame(edge)) {
                found = true;
                break;
            }
            edge_in_face.Next();
        }

        if (found) {
            faces_info.faces.push_back(face);
            faces_info.face_count++;
        }

        face_explorer.Next();
    }

    return faces_info;
}

std::pair<Point3D, Point3D> ModelProcessing::EdgeEndpoints(const TopoDS_Edge &edge)
{
    TopExp_Explorer vexp(edge, TopAbs_VERTEX);
    std::vector<Point3D> verts;

    while (vexp.More()) {
        TopoDS_Vertex v = TopoDS::Vertex(vexp.Current());
        gp_Pnt p = BRep_Tool::Pnt(v);
        verts.push_back(Point2Tuple(p));
        vexp.Next();
    }

    if (verts.empty()) {
        double f, l;
        Handle(Geom_Curve) curve_handle = BRep_Tool::Curve(edge, f, l);
        if (!curve_handle.IsNull()) {
            gp_Pnt p1, p2;
            curve_handle->D0(f, p1);
            curve_handle->D0(l, p2);
            return std::make_pair(Point2Tuple(p1), Point2Tuple(p2));
        }
        return std::make_pair(Point3D(), Point3D());
    }

    if (verts.size() == 1) {
        verts.push_back(verts[0]);
    }

    return std::make_pair(verts[0], verts[1]);
}

bool ModelProcessing::IsArcOrSpline(const TopoDS_Edge &edge)
{
    double f, l;
    Handle(Geom_Curve) curve_handle = BRep_Tool::Curve(edge, f, l);
    if (curve_handle.IsNull()) {
        return false;
    }

    // 使用lambda表达式
    auto get_curve_type_name = [&]() -> std::string {
        try {
            return std::string(curve_handle->DynamicType()->Name());
        } catch (...) {
            return "";
        }
    };

    std::string tname = get_curve_type_name();
    if (tname.find("Geom_Circle") != std::string::npos ||
        tname.find("Geom_BSplineCurve") != std::string::npos ||
        tname.find("Geom_BezierCurve") != std::string::npos) {
        return true;
    }

    return false;
}

std::vector<EdgeInfo> ModelProcessing::ExtractEdgesFromStep(const TopoDS_Shape& shape)
{
    std::vector<EdgeInfo> edges_info;
    int idx = 0;

    TopExp_Explorer explorer(shape, TopAbs_EDGE);
    while (explorer.More()) {
        TopoDS_Edge edge = TopoDS::Edge(explorer.Current());

        auto faces_info = GetFacesInfo(edge, shape);

        if (faces_info.face_count <= 1) {
            explorer.Next();
            continue;
        }

        bool isConvex = IsEdgeConvex(edge, faces_info);
        if (isConvex) {
            explorer.Next();
            continue;
        }

        auto ends = EdgeEndpoints(edge);

        bool arc_flag = IsArcOrSpline(edge);

        EdgeInfo edge_info;
        edge_info.edge = edge;
        edge_info.ends = ends;
        edge_info.is_arc_or_spline = arc_flag;
        edge_info.idx = idx;
        edge_info.faces_info = faces_info;

        edges_info.push_back(edge_info);

        idx++;
        explorer.Next();
    }

    GenerateDiscretePointsNormals(edges_info, 8.0);

    return edges_info;
}

bool ModelProcessing::IsEdgeConvex(const TopoDS_Edge& edge, const FacesInfo &faces_info)
{
    BRepAdaptor_Curve curveAdaptor(edge);

    // 获取参数范围
    double firstParam = curveAdaptor.FirstParameter();
    double lastParam = curveAdaptor.LastParameter();

    // 计算中点参数
    double midParam = (firstParam + lastParam) * 0.5;

    // 获取中点坐标
    gp_Pnt midpoint;
    curveAdaptor.D0(midParam, midpoint);
    Point3D start_point = Point2Tuple(midpoint);

    // 获取两个面的法向量
    Normal3D n1, n2;
    bool ok1, ok2;
    UVParams uv1, uv2;

    std::tie(n1, ok1, uv1) = NormalOnFaceAtPoint(faces_info.faces[0], start_point);
    std::tie(n2, ok2, uv2) = NormalOnFaceAtPoint(faces_info.faces[1], start_point);

    if (!ok1 || !ok2) {
        return false;
    }

    // 归一化法向量
    Normal3D n1_norm = NormalizeVec(n1);
    Normal3D n2_norm = NormalizeVec(n2);

    // 计算向量b（两个法向量的和）
    Normal3D vector_b = AddVecs(n1_norm, n2_norm);
    double mag_b = std::sqrt(vector_b.x*vector_b.x + vector_b.y*vector_b.y + vector_b.z*vector_b.z);
    if (mag_b < 1e-9) {
        return false;
    }
    Normal3D vector_b_norm = NormalizeVec(vector_b);


    Point3D point;
    point.x = start_point.x + vector_b_norm.x * 2.0;
    point.y = start_point.y + vector_b_norm.y * 2.0;
    point.z = start_point.z + vector_b_norm.z * 2.0;

    Handle(Geom_Surface) surf1 = BRep_Tool::Surface(faces_info.faces[0]);
    if (!surf1.IsNull()) {
        gp_Pnt pnt(point.x, point.y, point.z);
        GeomAPI_ProjectPointOnSurf proj1(pnt, surf1);
        if (proj1.NbPoints() > 0) {
            gp_Pnt projected_pnt = proj1.NearestPoint();
            Standard_Real u, v;
            proj1.LowerDistanceParameters(u, v);
            gp_Pnt2d uvPoint(u, v);

            BRepTopAdaptor_FClass2d classifier(faces_info.faces[0], Precision::Confusion());
            TopAbs_State state = classifier.Perform(uvPoint);

            if (state == TopAbs_IN || state == TopAbs_ON) {
                return false;
            } else {
                return true;
            }
        }
    }

}

