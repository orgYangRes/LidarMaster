#ifndef QVTKWINDOW_H
#define QVTKWINDOW_H

// Point Cloud Library

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>//loadPolygonFileOBJ所属头文件；
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "vtkGenericOpenGLRenderWindow.h"
#include <vtkRenderWindow.h>
#include <QVTKRenderWidget.h>
#include <QVTKOpenGLNativeWidget.h>
#include "LidarMaster.h"
using namespace pcl;
using namespace pcl::io;
using namespace std;

typedef pcl::PointXYZRGBA       PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

