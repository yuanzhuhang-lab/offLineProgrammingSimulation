#include "EdgeVisualizer.h"

EdgeVisualizer::EdgeVisualizer(vtkRenderer* renderer, vtkRenderWindowInteractor* interactor)
    : renderer_(renderer), interactor_(interactor)
{
    // 设置回调
    mouseMoveCallback_ = vtkSmartPointer<vtkCallbackCommand>::New();
    mouseMoveCallback_->SetClientData(this);
    mouseMoveCallback_->SetCallback(EdgeVisualizer::OnMouseMove);

    leftButtonCallback_ = vtkSmartPointer<vtkCallbackCommand>::New();
    leftButtonCallback_->SetClientData(this);
    leftButtonCallback_->SetCallback(EdgeVisualizer::OnLeftButtonPress);

    rightButtonCallback_ = vtkSmartPointer<vtkCallbackCommand>::New();
    rightButtonCallback_->SetClientData(this);
    rightButtonCallback_->SetCallback(EdgeVisualizer::OnRightButtonPress);

    interactor_->AddObserver(vtkCommand::MouseMoveEvent, mouseMoveCallback_);
    interactor_->AddObserver(vtkCommand::LeftButtonPressEvent, leftButtonCallback_);
    interactor_->AddObserver(vtkCommand::RightButtonPressEvent, rightButtonCallback_);
}

EdgeVisualizer::~EdgeVisualizer() {
    interactor_->RemoveObservers(vtkCommand::MouseMoveEvent);
    interactor_->RemoveObservers(vtkCommand::LeftButtonPressEvent);
    interactor_->RemoveObservers(vtkCommand::RightButtonPressEvent);
}

void EdgeVisualizer::AddEdges(const std::vector<EdgeInfo>& edges) {
    for (const auto& e : edges) {
        if (e.sample_points.size() < 2) continue;
        vtkSmartPointer<vtkActor> actor = CreateActorFromSamplePoints(e);
        renderer_->AddActor(actor);
        prop_to_idx_[actor] = e.idx;
        idx_to_actor_[e.idx] = actor;
        ApplyDefaultStyle(actor);
    }
    renderer_->GetRenderWindow()->Render();
}

vtkSmartPointer<vtkActor> EdgeVisualizer::CreateActorFromSamplePoints(const EdgeInfo& e) {
    auto pts = vtkSmartPointer<vtkPoints>::New();
    vtkIdType n = static_cast<vtkIdType>(e.sample_points.size());
    for (vtkIdType i=0;i<n;++i) {
        const Point3D& p = e.sample_points[(size_t)i];
        pts->InsertNextPoint(p.x, p.y, p.z);
    }

    auto polyLine = vtkSmartPointer<vtkPolyLine>::New();
    polyLine->GetPointIds()->SetNumberOfIds(n);
    for (vtkIdType i=0;i<n;++i) polyLine->GetPointIds()->SetId(i, i);

    auto cells = vtkSmartPointer<vtkCellArray>::New();
    cells->InsertNextCell(polyLine);

    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(pts);
    polyData->SetLines(cells);

    // 创建管状过滤器，将线条转换为管状结构
    auto tubeFilter = vtkSmartPointer<vtkTubeFilter>::New();
    tubeFilter->SetInputData(polyData);
    tubeFilter->SetRadius(1.5); // 设置管状半径，可根据需要调整
    tubeFilter->SetNumberOfSides(10); // 设置管状的分段数，值越大越平滑
    tubeFilter->CappingOn(); // 开启端盖

    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(tubeFilter->GetOutputPort());

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    // 确保 actor 可被拾取
    actor->PickableOn();

    // 管状结构样式：使用表面渲染而非线宽
//    actor->GetProperty()->SetOpacity(1.0);
//    actor->GetProperty()->SetLighting(true); // 开启光照
//    actor->GetProperty()->SetSpecular(0.3); // 设置高光
//    actor->GetProperty()->SetSpecularPower(20); // 设置高光强度

    return actor;
}

void EdgeVisualizer::AddSelectedEdge(int edge_idx)
{
    if (selected_edge_indices_.find(edge_idx) == selected_edge_indices_.end()) {
        selected_edge_indices_.insert(edge_idx);
        // 应用选中样式
        auto it = idx_to_actor_.find(edge_idx);
        if (it != idx_to_actor_.end()) {
            ApplySelectedStyle(it->second.Get());
        }
    }
}

void EdgeVisualizer::RemoveSelectedEdge(int edge_idx)
{
    if (selected_edge_indices_.find(edge_idx) != selected_edge_indices_.end()) {
        selected_edge_indices_.erase(edge_idx);
        // 恢复默认样式
        auto it = idx_to_actor_.find(edge_idx);
        if (it != idx_to_actor_.end()) {
            ApplyDefaultStyle(it->second.Get());
        }
    }
}

void EdgeVisualizer::ClearAllSelectedEdges()
{
    // 恢复所有已选边的默认样式
    for (int edge_idx : selected_edge_indices_) {
        auto it = idx_to_actor_.find(edge_idx);
        if (it != idx_to_actor_.end()) {
            ApplyDefaultStyle(it->second.Get());
        }
    }
    selected_edge_indices_.clear();
    selected_edge_idx_ = -1;
    selected_actor_ = nullptr;
}

void EdgeVisualizer::ApplyDefaultStyle(vtkActor* actor) {
    if (!actor) return;
    actor->GetProperty()->SetColor(0.2, 0.2, 0.8); // 默认蓝色（可改）

}

void EdgeVisualizer::ApplyHoverStyle(vtkActor* actor) {
    if (!actor) return;
    actor->GetProperty()->SetColor(1.0, 0.5, 0.0); // 悬停橙色
}

void EdgeVisualizer::ApplySelectedStyle(vtkActor* actor) {
    if (!actor) return;
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 选中红色
}

vtkProp* EdgeVisualizer::PickPropAtEventPosition() {
    int x, y;
    interactor_->GetEventPosition(x, y);

    // 使用vtkPropPicker，更适合表面几何体
    auto propPicker = vtkSmartPointer<vtkPropPicker>::New();

    if (propPicker->Pick(x, y, 0.0, renderer_)) {
        vtkProp* prop = propPicker->GetViewProp();

        // 修复映射查找：使用裸指针查找
        auto it = prop_to_idx_.find(prop);
        if (it != prop_to_idx_.end()) {
            return prop;
        }
    }

    return nullptr;
}

/* static callbacks */
void EdgeVisualizer::OnMouseMove(vtkObject* caller, unsigned long, void* clientdata, void*) {
    EdgeVisualizer* self = static_cast<EdgeVisualizer*>(clientdata);
    if (!self) return;

    vtkProp* pickedProp = self->PickPropAtEventPosition();
    vtkActor* pickedActor = vtkActor::SafeDownCast(pickedProp);

    // 将 hover_actor_ 解包成裸指针
    vtkActor* hoverRaw = self->hover_actor_ ? self->hover_actor_.Get() : nullptr;

    // 如果 pickedProp 是当前 hover actor，什么也不做
    if (hoverRaw && pickedActor == hoverRaw) {
        return;
    }

    // 恢复先前 hover 的样式（但不影响已选中的 actor）
    if (hoverRaw) {
        // 检查该边是否在已选集合中
        auto it = self->prop_to_idx_.find(static_cast<vtkProp*>(hoverRaw));
        if (it != self->prop_to_idx_.end()) {
            int edge_idx = it->second;
            if (self->selected_edge_indices_.find(edge_idx) != self->selected_edge_indices_.end()) {
                // 如果是已选中的边，则保留选中样式
                self->ApplySelectedStyle(hoverRaw);
            } else {
                // 如果不是已选中的边，则恢复默认样式
                self->ApplyDefaultStyle(hoverRaw);
            }
        }
        self->hover_actor_ = nullptr;
    }

    // 如果拾取到 actor，则应用 hover 样式（无论是否已选中）
    if (pickedActor) {
        // 正确设置 hover_actor_，避免重复赋值和引用计数问题
        self->hover_actor_ = vtkActor::SafeDownCast(pickedProp);
        self->ApplyHoverStyle(self->hover_actor_.Get());
    }

    self->renderer_->GetRenderWindow()->Render();
}

void EdgeVisualizer::OnLeftButtonPress(vtkObject* caller, unsigned long, void* clientdata, void*) {
    EdgeVisualizer* self = static_cast<EdgeVisualizer*>(clientdata);
    if (!self) return;

    vtkProp* pickedProp = self->PickPropAtEventPosition();
    vtkActor* pickedActor = vtkActor::SafeDownCast(pickedProp);

    if (pickedActor) {
        // 查找 idx：map 的 key 是裸指针
        int idx = -1;
        auto it = self->prop_to_idx_.find(static_cast<vtkProp*>(pickedActor));
        if (it != self->prop_to_idx_.end()) {
            idx = it->second;

            // 检查是否已选中（支持Ctrl多选）
            bool isCtrlPressed = (self->interactor_->GetControlKey() == 1);

            if (self->selected_edge_indices_.find(idx) != self->selected_edge_indices_.end()) {
                // 如果已选中，且按下了Ctrl键，则取消选择
                if (isCtrlPressed) {
                    self->RemoveSelectedEdge(idx);
                }
                // 否则保持选中状态
            } else {
                // 如果未选中，则添加选中
                if (!isCtrlPressed) {
                    // 如果没有按下Ctrl键，先清除所有已选边
                    self->ClearAllSelectedEdges();
                }
                self->AddSelectedEdge(idx);

                // 更新单选的actor和idx（用于向后兼容）
                self->selected_actor_ = vtkActor::SafeDownCast(pickedProp);
                self->selected_edge_idx_ = idx;
            }
        }
    }

    /*else {
        // 点击空白处，如果没有按下Ctrl键，清除所有选择
        bool isCtrlPressed = (self->interactor_->GetControlKey() == 1);
        if (!isCtrlPressed) {
            self->ClearAllSelectedEdges();
        }
    }*/

    self->renderer_->GetRenderWindow()->Render();

    // 将事件传回 interactor 的默认处理
    vtkRenderWindowInteractor* iren = static_cast<vtkRenderWindowInteractor*>(caller);
    if (iren) {
        iren->InvokeEvent(vtkCommand::LeftButtonPressEvent, nullptr);
    }
}

void EdgeVisualizer::OnRightButtonPress(vtkObject *caller, unsigned long eid, void *clientdata, void *calldata)
{
    EdgeVisualizer* self = static_cast<EdgeVisualizer*>(clientdata);
    if (!self) return;

    vtkProp* pickedProp = self->PickPropAtEventPosition();
    vtkActor* pickedActor = vtkActor::SafeDownCast(pickedProp);

    if (pickedActor) {
        // 查找 idx：map 的 key 是裸指针
        int idx = -1;
        auto it = self->prop_to_idx_.find(static_cast<vtkProp*>(pickedActor));
        if (it != self->prop_to_idx_.end()) {
            idx = it->second;

            // 右键取消选择该边
            self->RemoveSelectedEdge(idx);

            // 更新单选的actor和idx
            if (self->selected_edge_idx_ == idx) {
                self->selected_actor_ = nullptr;
                self->selected_edge_idx_ = -1;
            }
        }
    } else {
        // 右键点击空白处，清除所有选择
        self->ClearAllSelectedEdges();
    }

    self->renderer_->GetRenderWindow()->Render();

    // 将事件传回 interactor 的默认处理
    vtkRenderWindowInteractor* iren = static_cast<vtkRenderWindowInteractor*>(caller);
    if (iren) {
        iren->InvokeEvent(vtkCommand::RightButtonPressEvent, nullptr);
    }
}
