#include "LidarMaster.h"
#include <iostream>
#include <string>
#include <QVTKOpenGLNativeWidget.h>
#include "vtkGenericOpenGLRenderWindow.h"
#include "lasreader.hpp"
#include "laswriter.hpp"
LidarMaster::LidarMaster(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//初始化工程文件树
	m_PtrProTree = new QTreeWidget(this);
	//设置文件树头为空
	m_PtrProTree->setHeaderHidden(true);

	m_PtrLasInfoTree = new QTreeWidget(this);


	connect(m_PtrProTree, SIGNAL(itemClicked(QTreeWidgetItem*, int)), this, SLOT(treeItemClickedSlot(QTreeWidgetItem*, int)));
	// 设置主窗体属性
	MainFramAttri();
	// 初始化菜单栏指针
	m_PtrMenu = new LidarMenu(this, this);
	this->setMenuBar(m_PtrMenu);
	m_LidarWidget = new QVTKOpenGLNativeWidget;
	/*m_PtrQVtkWindow = new QVTKWindow(this, this);*/

	m_dockMain->setWidget(m_LidarWidget);
	
	renderer2 = vtkSmartPointer<vtkRenderer>::New();
    renderWindow2 = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderWindow2->AddRenderer(renderer2);
    viewer.reset(new pcl::visualization::PCLVisualizer(renderer2, renderWindow2, "viewer", false));

}

LidarMaster::~LidarMaster()
{

}
void LidarMaster::updateWindSlot()
{
	/*m_PtrQVtkWindow->update();
	m_PtrQVtkWindow->viewer->resetCamera();*/
}
void LidarMaster::recvRenderCoords(QString& strAxis)
{
	if (m_Cloud->points.size() > 0)
    {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> render(m_Cloud, strAxis.toStdString());
        viewer->updatePointCloud(m_Cloud, render, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		m_LidarWidget->update();
		m_dockMain->update();
		update();
    }

}
void LidarMaster::recColorInfoSlot(QColor& color)
{
	if (color.isValid() && viewer)
    {
        viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
		m_LidarWidget->update();
    }

}
void LidarMaster::MainFramAttri()
{
	// 软件名称
	setWindowTitle(QStringLiteral("LidarMaster"));
	// 软件图标
	setWindowIcon(QIcon(":/LidarMaster/img/logo.png"));
	// 设置大小
	resize(QGuiApplication::primaryScreen()->availableSize() * 16 / 9);

	// 删除中央窗体
	QWidget* centralWidget = takeCentralWidget();
	if (centralWidget)
		delete centralWidget;
	//允许嵌套dock
	setDockNestingEnabled(true);

	// 设置状态栏
	QStatusBar* stBar = statusBar();
	setStatusBar(stBar);
	//添加提示信息到右侧
	QLabel* lab_cprt = new QLabel(QStringLiteral("@版权所属1114809057@qq.com"), this);
	lab_cprt->setStyleSheet("color:#ffffff;margin-right:10px;");
	stBar->addPermanentWidget(lab_cprt);
	stBar->setStyleSheet("background:#000000;");

	// 点云显示区域
	m_dockMain = new QDockWidget(this);
	m_dockMain->setWindowTitle(QStringLiteral("点云"));
	// 设置为可移动可浮动，但不可关闭
	m_dockMain->setFeatures(QDockWidget::DockWidgetMovable | QDockWidget::DockWidgetFloatable);
	// 可移动范围：左右
	m_dockMain->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);  

	// 工程信息
	m_dockProInfo = new QDockWidget(this);
	m_dockProInfo->setWindowTitle(QStringLiteral("工程"));
	m_dockProInfo->setFeatures(QDockWidget::DockWidgetClosable);

	// 点云信息
	m_dockLasInfo = new QDockWidget(this);
	m_dockLasInfo->setWindowTitle(QStringLiteral("属性"));
	m_dockLasInfo->setFeatures(QDockWidget::AllDockWidgetFeatures);

	// 其他信息
	m_dockOtherInfo = new QDockWidget(this);
	m_dockOtherInfo->setWindowTitle(QStringLiteral("其他"));
	m_dockOtherInfo->setFeatures(QDockWidget::AllDockWidgetFeatures);

	// 设置子窗体大小
	m_dockProInfo->setFixedWidth(200);
	m_dockLasInfo->setFixedWidth(200);
	m_dockOtherInfo->setFixedWidth(200);
	m_dockLasInfo->setMaximumHeight(400);

	// 布局方向
	setCentralWidget(m_dockMain);
	addDockWidget(Qt::LeftDockWidgetArea, m_dockProInfo);
	addDockWidget(Qt::RightDockWidgetArea, m_dockLasInfo);
	addDockWidget(Qt::RightDockWidgetArea, m_dockOtherInfo);

	// 布局方向
	splitDockWidget(m_dockLasInfo, m_dockOtherInfo, Qt::Vertical);

	connect(m_dockLasInfo, SIGNAL(visibilityChanged(bool)), this, SLOT(isLasInfo()));
	connect(m_dockOtherInfo, SIGNAL(visibilityChanged(bool)), this, SLOT(isOtherInfo()));
	connect(m_dockProInfo, SIGNAL(visibilityChanged(bool)), this, SLOT(isProInfo()));
	m_dockProInfo->setWidget(m_PtrProTree);
	setLasInfoDock();

}
void LidarMaster::setLasInfoDock()
{
	QStringList strList;
	strList << QStringLiteral("属性") << QStringLiteral("值");
	m_PtrLasInfoTree->setHeaderLabels(strList);

	QTreeWidgetItem* parentItem1 = new QTreeWidgetItem(QStringList() << QStringLiteral("点云信息"));
	m_PtrLasInfoTree->addTopLevelItem(parentItem1);
	QTreeWidgetItem* item11 = new QTreeWidgetItem();
	item11->setText(0, QStringLiteral("点云个数"));
	item11->setText(1, QStringLiteral(""));

	QTreeWidgetItem* item12 = new QTreeWidgetItem();
	item12->setText(0, QStringLiteral("点云格式"));
	item12->setText(1, QStringLiteral(""));
	QTreeWidgetItem* item13 = new QTreeWidgetItem();
	item13->setText(0, QStringLiteral("点云类型"));
	item13->setText(1, QStringLiteral(""));
	parentItem1->addChild(item11);
	parentItem1->addChild(item12);
	parentItem1->addChild(item13);

	m_dockLasInfo->setWidget(m_PtrLasInfoTree);
}

void LidarMaster::isProInfo()
{
	if (m_dockProInfo->isHidden())
	{
		m_PtrMenu->m_isShowProInfoAct->setIcon(QIcon(":/LidarMaster/img/hide.png"));
	}
	else
	{
		m_PtrMenu->m_isShowProInfoAct->setIcon(QIcon(":/LidarMaster/img/show.png"));
	}
}

void LidarMaster::isOtherInfo()
{
	if (m_dockOtherInfo->isHidden())
	{
		m_PtrMenu->m_isShowOtherInfoAct->setIcon(QIcon(":/LidarMaster/img/hide.png"));
	}
	else
	{
		m_PtrMenu->m_isShowOtherInfoAct->setIcon(QIcon(":/LidarMaster/img/show.png"));
	}
}

void LidarMaster::treeItemClickedSlot(QTreeWidgetItem* item, int col)
{
	if (item->parent())
	{
		if (item->checkState(0)==Qt::Checked)
		{
			QString lidarFile = item->data(0, Qt::UserRole + 1).toString();
			showLidarData(lidarFile);
		}
		else
		{
			QString tmp = "";
			showLidarData(tmp);
		}
	}
}
void LidarMaster::showLidarData(QString& lidarFile)
{

//#if VTK_MAJOR_VERSION > 8
	m_LidarWidget->setRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(m_LidarWidget->interactor(), m_LidarWidget->renderWindow());
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
		m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1, QString::number(m_Cloud->points.size()));
        
        m_PtrLasInfoTree->topLevelItem(0)->child(1)->setText(1, QStringLiteral("pcd"));
        if (m_Cloud->height == 1)
        {
            m_PtrLasInfoTree->topLevelItem(0)->child(2)->setText(1, "0");
        }
        else
        {
            m_PtrLasInfoTree->topLevelItem(0)->child(2)->setText(1, "1");
        }
    }
    else if (QFileInfo(lidarFile).suffix().contains("ply"))
    {
        pcl::PolygonMesh meshData;//读取原始数据
        pcl::io::loadPolygonFile(lidarFile.toStdString(), meshData);
        pcl::fromPCLPointCloud2(meshData.cloud, *m_Cloud);//将obj数据转换为点云数据
        m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1, QString::number(m_Cloud->points.size()));
       m_PtrLasInfoTree->topLevelItem(0)->child(1)->setText(1, QStringLiteral("ply"));
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
        m_PtrLasInfoTree->topLevelItem(0)->child(1)->setText(1, QStringLiteral("las"));
    }
    m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1, QString::number(m_Cloud->points.size()));

   
    // 显示结果图

    viewer->setBackgroundColor(0, 0, 0); //设置背景
    // viewer->addCoordinateSystem (15.0); //设置坐标系
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(m_Cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(m_Cloud, fildColor, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->resetCamera();
	m_LidarWidget->update();
}




void LidarMaster::isLasInfo()
{
	if (m_dockLasInfo->isHidden())
	{
		m_PtrMenu->m_isShowLasInfoAct->setIcon(QIcon(":/LidarMaster/img/hide.png"));
	}
	else
	{
		m_PtrMenu->m_isShowLasInfoAct->setIcon(QIcon(":/LidarMaster/img/show.png"));
	}
}
