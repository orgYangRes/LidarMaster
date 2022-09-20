#include "QVTKWindow.h"
#include <qpainter.h>
#include <qdebug.h>
#include "lasreader.hpp"
#include "laswriter.hpp"

#include <qdebug.h>
#include <qfileinfo.h>
QVTKWindow::QVTKWindow(int win_size, QWidget *parent, LidarMaster* lasMaster)
    : QVTKOpenGLNativeWidget(parent)
{
    m_PtrLidarMaster = (LidarMaster*)lasMaster;
    // The default color
    Vtk_Win_Point_Cloud = new VTK_POINT_CLOUD_S[1920*1080*4*10];   
    renderer2 = vtkSmartPointer<vtkRenderer>::New();
    renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));
    
}

QVTKWindow::~QVTKWindow()
{
    delete Vtk_Win_Point_Cloud;
}

void QVTKWindow::showLidarData(QString& lidarFile)
{

//#if VTK_MAJOR_VERSION > 8
    this->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(this->interactor(), this->renderWindow());
//#else
//    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
//    this->SetRenderWindow(viewer->getRenderWindow());
//    viewer->setupInteractor(this->GetInteractor(), this->GetRenderWindow());
//#endif

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
    else if (QFileInfo(lidarFile).suffix().contains("las"))
    {
        LASreadOpener lasreadopener;
        QByteArray ba = lidarFile.toLatin1();
        lasreadopener.set_file_name(ba.data());
        LASreader* lasreader = lasreadopener.open(false);
        size_t ct = lasreader->header.number_of_point_records;
        m_Cloud->points.resize(ct);
        m_Cloud->width = 1;
        m_Cloud->height = ct;
        m_Cloud->is_dense = false;
        size_t i = 0;
        while (lasreader->read_point() && i < ct)
        {
            m_Cloud->points[i].x = lasreader->point.get_x();
            m_Cloud->points[i].y = lasreader->point.get_y();
            m_Cloud->points[i].z = lasreader->point.get_z();
            ++i;
        }
        m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(1)->setText(1, QStringLiteral("las"));
    }
    m_PtrLidarMaster->m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1, QString::number(m_Cloud->points.size()));

    //	m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1,"100000");
    // 显示结果图

    viewer->setBackgroundColor(0, 0, 0); //设置背景
    // viewer->addCoordinateSystem (15.0); //设置坐标系
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(m_Cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(m_Cloud, fildColor, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->resetCamera();
   /* update();*/
}
void QVTKWindow::recvRenderCoords(QString& strAxis)
{
    if (m_Cloud->points.size() > 0)
    {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> render(m_Cloud, strAxis.toStdString());
        viewer->updatePointCloud(m_Cloud, render, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
        update();
        emit updateWind();
    }
}
void QVTKWindow::recColorInfoSlot(QColor& color)
{
    if (color.isValid() && viewer)
    {
        viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
        this->update();
        emit updateWind();
    }

}





