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
class LidarMaster;
typedef pcl::PointXYZRGBA       PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

//点云数据
typedef struct vtkpointcloud{
    float x;
    float y;
    float z;
    unsigned int red;
    unsigned int green;
    unsigned int blue;
}VTK_POINT_CLOUD_S;

class QVTKWindow : public QVTKOpenGLNativeWidget
{
    Q_OBJECT
public:
    explicit QVTKWindow(int win_size,QWidget *parent, LidarMaster* lasMaster);
    ~QVTKWindow();

    void showLidarData(QString& lidarFile);
    VTK_POINT_CLOUD_S *Vtk_Win_Point_Cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_Cloud;
    QString m_strLasFile;
    pcl::visualization::PCLVisualizer::Ptr viewer;
private:
    LidarMaster* m_PtrLidarMaster;
private:
    
    vtkSmartPointer<vtkRenderer>renderer2;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow>renderWindow2;
signals:
    void updateWind();
private slots:
    void recvRenderCoords(QString& strAxis);

    void recColorInfoSlot(QColor& color);

};

#endif // QVTKWINDOW_H
