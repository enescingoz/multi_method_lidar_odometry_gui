#ifndef VTK_ACTORS_H
#define VTK_ACTORS_H

#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkConeSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkStructuredPoints.h>
#include <vtkPointData.h>
#include <vtkFloatArray.h>
#include <vtkCellData.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

// Function declarations
vtkSmartPointer<vtkActor> CreateActorFromPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
vtkSmartPointer<vtkActor> GetPointCloudActor();
vtkSmartPointer<vtkActor> GetConeActor();
vtkSmartPointer<vtkActor> GetLineActor();

#endif // VTK_ACTORS_H