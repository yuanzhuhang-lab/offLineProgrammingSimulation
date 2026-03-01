#ifndef HEADOFMAINWINDOW_H
#define HEADOFMAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QColor>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <unordered_map>
#include <memory>
#include <unordered_set>
#include <iomanip>
#include <ctime>
#include <chrono>
#include <thread>

// VTK
#include <QVTKOpenGLWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSTLReader.h>
#include <vtkTransform.h>
#include <vtkAssembly.h>
#include <vtkNamedColors.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkLine.h>
#include <vtkAutoInit.h>
#include <vtkPolyLine.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyDataNormals.h>
#include <vtkType.h>
#include <vtkPoints.h>
#include <vtkTriangle.h>
#include <vtkCellPicker.h>
#include <vtkProp.h>
#include <vtkCallbackCommand.h>
#include <vtkTubeFilter.h>
#include <vtkPropPicker.h>        // 添加属性拾取器
#include <vtkSelection.h>         // 添加选择相关头文件
#include <vtkSelectionNode.h>

// OpenCASCADE
#include <STEPControl_Reader.hxx>
#include <TopoDS_Shape.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <Standard_Integer.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IFSelect_PrintCount.hxx>
#include <IVtkVTK_ShapeData.hxx>
#include <IVtkOCC_ShapeMesher.hxx>
#include <TopoDS_Edge.hxx>
#include <Poly_PolygonOnTriangulation.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <IVtkTools_ShapeDataSource.hxx>

#include <StlAPI_Writer.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Poly_Triangulation.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <Poly_Array1OfTriangle.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <TopExp.hxx>



#endif  // HEADOFMAINWINDOW_H
