#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_LidarMaster.h"
#include "QVTKOpenGLNativeWidget.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include <qdebug.h>
#include <QDockWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include "LidarMenu.h"
#include "QVTKWindow.h"
#include <qsharedpointer.h>
#include <QMap>
#include <atomic>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/edge_aware_upsample_point_set.h>

#include <utility> // defines std::pair
#include <list>
#include <fstream>
#include <string>
#include <vector>
#define CGAL_LINKED_WITH_TBB

typedef CGAL::Simple_cartesian<double> Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> PointVectorPair;
#ifdef CGAL_LINKED_WITH_TBB
typedef CGAL::Parallel_tag Concurrency_tag;
#else
typedef CGAL::Sequential_tag Concurrency_tag;
#endif


class LidarMenu;
class QVTKWindow;
class LidarMaster : public QMainWindow
{
    Q_OBJECT

public:
    LidarMaster(QWidget *parent = nullptr);
    ~LidarMaster();

     // 菜单
     QPointer<LidarMenu> m_PtrMenu;
     // 显示Las的区域
     QPointer<QDockWidget> m_dockMain;
     // 显示工程信息的区域
     QPointer<QDockWidget> m_dockProInfo;
     // 显示Las信息的区域
     QPointer<QDockWidget> m_dockLasInfo;
     // 显示其他信息的区域
     QPointer<QDockWidget> m_dockOtherInfo;

     // 工程文件树
     QPointer<QTreeWidget> m_PtrProTree;

     // 点云信息树
     QPointer<QTreeWidget> m_PtrLasInfoTree;

     QPointer< QVTKWindow> m_PtrQVtkWindow;

     QPointer<QVTKOpenGLNativeWidget> m_LidarWidget;
     pcl::visualization::PCLVisualizer::Ptr viewer;
     vtkSmartPointer<vtkRenderer>renderer2;
     vtkSmartPointer<vtkGenericOpenGLRenderWindow>renderWindow2;

     //点云上采样CGAL

     void pcl2CGAL();

private:
    Ui::LidarMasterClass ui;
    int counts = 0;
    // 主窗体属性配置
    void MainFramAttri();
    std::atomic<int> m_nCloudIndex = 0;
    QMap<int,QString>m_mapCloud;

    // 设置点云信息窗口
    void setLasInfoDock();
    QSharedPointer<std::thread> m_MainThread;
    void showLidarData(QString& lidarFile,int col =0);

    void showPtCloudHeightInfo();

    void getPtCLoud(QString& lidarFile,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    int savePtCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud, const QString& saveFileName);
    int saveCloudToMap(QString& lasFile);
    int getIndex(QString& file);

    void addItems(const QString& saveFileName);
signals:
    void sendRenderAxis(QString& strAxis);
    void sendColorInfo(QColor& color);
    void closeFilterDialogSignal();

    void closeGridFilterDialogSignal();

    void closeSIFTDialog();

    void closeHarrisDialog();
private slots:
    // 关闭点云信息信号
    void isLasInfo();

    // 关闭工程信息信号
    void isProInfo();

    // 关闭其他信息信号
    void isOtherInfo();

    // 工程树节点点击槽
    void treeItemClickedSlot(QTreeWidgetItem* item, int col);

    void recvRenderCoords(QString& strAxis);

    void recColorInfoSlot(QColor& color);

    //体素滤波
    void recvFilterVal(int type, double filterVal, QString& lasFile);

    //撤销
    void rectoLeftSlot();

    //反撤销
    void rectoRightSlot();

    //按点采样
    void recvGridAndType(int gridVal, int type);

    //SIFT
    void recvSIFTval(float std, int level, int num, float val);

    // Harris
    void recvHarrisval(float normal,float check,float thr);
};


