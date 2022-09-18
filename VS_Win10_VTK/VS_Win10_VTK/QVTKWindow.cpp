#include "QVTKWindow.h"
#include <qpainter.h>
#include <qdebug.h>

#include "vtkGenericOpenGLRenderWindow.h"
#include <qdebug.h>
#include <qfileinfo.h>
QVTKWindow::QVTKWindow(int win_size, QWidget *parent, LidarMaster* lasMaster)
    : QVTKOpenGLNativeWidget(parent)
{
    m_PtrLidarMaster = (LidarMaster*)lasMaster;
    // The default color
    red         = 128;
    green       = 128;
    blue        = 128;
    Window_Size = 0;
    Vtk_Win_Point_Cloud = new VTK_POINT_CLOUD_S[1920*1080*4*10];   
}

QVTKWindow::~QVTKWindow()
{
    delete Vtk_Win_Point_Cloud;
}

void QVTKWindow::setLidarMaster()
{
}

void QVTKWindow::VTKWindow_Resize(int win_size)
{
    if (win_size == Window_Size) return;
    Window_Size = win_size;
    cloud->points.resize (win_size);

    // Fill the cloud with some points
    for (size_t i = 0; i < cloud->points.size (); ++i)
    {
      cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
      cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);

      cloud->points[i].r = red;
      cloud->points[i].g = green;
      cloud->points[i].b = blue;
    }

    viewer->updatePointCloud(cloud, "cloud");
    update ();

}

void QVTKWindow::Update_Window_PointCloude()
{
    for (int i = 0; i < Window_Size; ++i) {
        cloud->points[i].x = Vtk_Win_Point_Cloud[i].x;
        cloud->points[i].y = Vtk_Win_Point_Cloud[i].y;
        cloud->points[i].z = Vtk_Win_Point_Cloud[i].z;

        cloud->points[i].r = red;
        cloud->points[i].g = green;
        cloud->points[i].b = blue;
    }
    viewer->updatePointCloud(cloud, "cloud");
    update();
}

void QVTKWindow::Set_Error_Text(QString str)
{
    int x = 10;
    int y = size().height()-50;
    viewer->updateText(QString("No: %1").arg(str).toStdString(), x, y, 20, 1.0,255.0,255.0,"No_text");//红
    update();
}

void QVTKWindow::Set_OK_Text(QString str)
{
    int x = size().width() -200;
    int y = size().height()-50;
    viewer->updateText(QString("Yes: %1").arg(str).toStdString(), x, y, 20, 255.0,1.0,255.0,"Yes_text");//蓝
    update();
}

void QVTKWindow::Set_Info_Text(QString str)
{
    int x = size().width()/2 -10;
    int y = size().height()-50;
    viewer->updateText(QString("INFO: %1").arg(str).toStdString(), x, y, 20, 255.0,255.0,1.0,"INFO_text");//绿
    update();
}

void QVTKWindow::showLidarData(QString& lidarFile)
{
#if VTK_MAJOR_VERSION > 8

    auto renderer2 = vtkSmartPointer<vtkRenderer>::New();
    auto renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
    this->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->interactor(), this->renderWindow());

#else
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    this->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
#endif


    if (lidarFile.isEmpty())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        return;
    }
    m_Cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    if (QFileInfo(lidarFile).suffix().contains("pcd"))
    {
        pcl::PCDReader reader;
        reader.read(lidarFile.toStdString(), *m_Cloud);
        m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1, QString::number(m_Cloud->points.size()));
        
        qDebug()<<"info:" << m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(1)->text(0);
        m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(1)->setText(1, QStringLiteral("pcd"));
        if (m_Cloud->height == 1)
        {
            m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(2)->setText(1, "0");
        }
        else
        {
            m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(2)->setText(1, "1");
        }
    }
    else if (QFileInfo(lidarFile).suffix().contains("ply"))
    {
        pcl::PolygonMesh meshData;//读取原始数据
        pcl::io::loadPolygonFile(lidarFile.toStdString(), meshData);
        pcl::fromPCLPointCloud2(meshData.cloud, *m_Cloud);//将obj数据转换为点云数据
        m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1, QString::number(m_Cloud->points.size()));
        m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(1)->setText(1, QStringLiteral("ply"));
    }
    //	m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1,"100000");
    // 显示结果图

    viewer->setBackgroundColor(0, 0, 0); //设置背景
    // viewer->addCoordinateSystem (15.0); //设置坐标系
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(m_Cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(m_Cloud, fildColor, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->resetCamera();

    update();
}







