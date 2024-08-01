#ifndef VTK_ACTORS_H
#define VTK_ACTORS_H

#include <vtkActor.h>
#include <vtkSmartPointer.h>

// Function declarations
vtkSmartPointer<vtkActor> GetPointCloudActor();
vtkSmartPointer<vtkActor> GetConeActor();
vtkSmartPointer<vtkActor> GetLineActor();

#endif // VTK_ACTORS_H