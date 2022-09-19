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

// Visualization Toolkit (VTK)
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

    void setLidarMaster();

    void VTKWindow_Resize(int win_size);
    void Update_Window_PointCloude();
    void Set_Error_Text(QString str);
    void Set_OK_Text(QString str);
    void Set_Info_Text(QString str);

    void showLidarData(QString& lidarFile);
    VTK_POINT_CLOUD_S *Vtk_Win_Point_Cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr m_Cloud;
    QString m_strLasFile;
    pcl::visualization::PCLVisualizer::Ptr viewer;
private:
    LidarMaster* m_PtrLidarMaster;
private:
    
    PointCloudT::Ptr                       cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr              cloude_color;
    std::string                            Text;
    unsigned int red;
    unsigned int green;
    unsigned int blue;
    int Window_Size;
private slots:
    void recvRenderCoords(QString& strAxis);

};

#endif // QVTKWINDOW_H
