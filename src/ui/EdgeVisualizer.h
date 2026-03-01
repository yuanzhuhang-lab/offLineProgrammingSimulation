#ifndef EDGEVISUALIZER_H
#define EDGEVISUALIZER_H

#include "ui/head/headOfMainWindow.h"
#include "model_processing/common.h"

class EdgeVisualizer
{
public:
    EdgeVisualizer(vtkRenderer* renderer, vtkRenderWindowInteractor* interactor);
    ~EdgeVisualizer();
    // 添加一批边（每条边会对应一个 actor）
    void AddEdges(const std::vector<EdgeInfo>& edges);

    // 获取当前被选择的边索引（-1 表示无）
    int GetSelectedEdgeIdx() const { return selected_edge_idx_; }

    // 获取所有已选边的索引
    std::unordered_set<int> GetSelectedEdgeIndices() const { return selected_edge_indices_; }

private:
    vtkRenderer* renderer_;
    vtkRenderWindowInteractor* interactor_;

    // Picker 回调对象
    vtkSmartPointer<vtkCallbackCommand> mouseMoveCallback_;
    vtkSmartPointer<vtkCallbackCommand> leftButtonCallback_;
    vtkSmartPointer<vtkCallbackCommand> rightButtonCallback_;

    // 映射 actor -> edge idx
    std::unordered_map<vtkProp*, int> prop_to_idx_;
    // 反向映射（若需要）
    std::unordered_map<int, vtkSmartPointer<vtkActor>> idx_to_actor_;

    // 当前 hover actor 和选中 actor
    vtkSmartPointer<vtkActor> hover_actor_;
    vtkSmartPointer<vtkActor> selected_actor_;
    int selected_edge_idx_ = -1;

    // 多选：存储所有已选边的索引
    std::unordered_set<int> selected_edge_indices_;

    // 样式常量
    void ApplyDefaultStyle(vtkActor* actor);
    void ApplyHoverStyle(vtkActor* actor);
    void ApplySelectedStyle(vtkActor* actor);

    // 回调静态函数
    static void OnMouseMove(vtkObject* caller, unsigned long eid, void* clientdata, void* calldata);
    static void OnLeftButtonPress(vtkObject* caller, unsigned long eid, void* clientdata, void* calldata);
    static void OnRightButtonPress(vtkObject* caller, unsigned long eid, void* clientdata, void* calldata);

    // 辅助：从 interactor 的鼠标位置 pick 出 actor
    vtkProp* PickPropAtEventPosition();

    // 创建 actor（整条曲边）
    vtkSmartPointer<vtkActor> CreateActorFromSamplePoints(const EdgeInfo& e);

    // 辅助函数：添加/移除选中边
    void AddSelectedEdge(int edge_idx);
    void RemoveSelectedEdge(int edge_idx);
    void ClearAllSelectedEdges();

};

#endif // EDGEVISUALIZER_H
