#include "LidarMaster.h"
#include <iostream>
#include <string>
#include <QVTKOpenGLNativeWidget.h>
#include "vtkGenericOpenGLRenderWindow.h"
#include "lasreader.hpp"
#include "laswriter.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
LidarMaster::LidarMaster(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);

	//初始化工程文件树
	m_PtrProTree = new QTreeWidget(this);
	//设置文件树头为空
	m_PtrProTree->setHeaderHidden(true);

	m_PtrLasInfoTree = new QTreeWidget(this);
	m_Cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

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
	if (m_MainThread != nullptr)
	{
		if (m_MainThread->joinable())
		{
			m_MainThread->join();
		}
		m_MainThread = nullptr;
	}
}

void LidarMaster::recvRenderCoords(QString& strAxis)
{
	if (m_Cloud&&m_Cloud->points.size() > 0)
    {
        pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> render(m_Cloud, strAxis.toStdString());
		viewer->updatePointCloud(m_Cloud, render, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		viewer->spin();
		update();
    }

}
void LidarMaster::recColorInfoSlot(QColor& color)
{
	/*if (color.isValid())
	{
		viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
		viewer->spin();
		update();
	}*/
	if (!m_Cloud->empty())
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> render(m_Cloud, color.redF() * 255, color.greenF() * 255, color.blueF() * 255);
		viewer->updatePointCloud(m_Cloud, render, "cloud");
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		viewer->spin();
		update();
	}
	
	
	

}
int LidarMaster::savePtCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud, const QString& saveFileName)
{
	int saveRes = 1;
	if (saveFileName.endsWith(".pcd", Qt::CaseInsensitive))
	{
		saveRes = pcl::io::savePCDFileBinary(saveFileName.toStdString(), *tmpCloud);
	}
	if (saveFileName.endsWith(".ply", Qt::CaseInsensitive))
	{
		saveRes = pcl::io::savePLYFileBinary(saveFileName.toStdString(), *tmpCloud);
	}

	if (saveRes != 0)
	{
		PCL_ERROR("Error writing point cloud %s\n", saveFileName.toStdString().c_str());
		return -1;
	}
	else
	{
		return 0;
	}
}

void LidarMaster::saveCloudToMap(pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud)
{
	int index = 0;
	if (m_mapCloud.isEmpty())
	{
		m_mapCloud.insert(0, tmpCloud);
		index = 0;
	}
	else
	{
		auto mm = std::minmax_element(m_mapCloud.keys().begin(), m_mapCloud.keys().end());
		m_mapCloud.insert(*mm.first +1, tmpCloud);
		index = *mm.first + 1;
	}
	m_nCloudIndex = index;
}
void LidarMaster::recvFilterVal(int type, double filterVal,QString& lasFile)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud;
	tmpCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

	getPtCLoud(lasFile, tmpCloud);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
	QString saveFileName = QFileDialog::getSaveFileName(this,QStringLiteral("保存滤波结果"), QStringLiteral("./result.pcd"), QStringLiteral("点云数据(*.ply *.pcd)"));
	

	if (saveFileName.isEmpty()) return;
	//体素
	if (type == 1)
	{
		pcl::VoxelGrid<pcl::PointXYZ> voxel_Grid;
		voxel_Grid.setLeafSize(filterVal, filterVal, filterVal);
		voxel_Grid.setInputCloud(tmpCloud);
		voxel_Grid.filter(*cloud_out);
	}
	// 近似体素
	else if (type == 2)
	{
		pcl::ApproximateVoxelGrid<pcl::PointXYZ> appVoxel_Grid;
		appVoxel_Grid.setLeafSize(filterVal, filterVal, filterVal);
		appVoxel_Grid.setInputCloud(tmpCloud);
		appVoxel_Grid.filter(*cloud_out);
	}
	int saveRes = 1;
	saveRes = savePtCloud(cloud_out, saveFileName);

	saveCloudToMap(cloud_out);

	if (saveRes != 0)
	{
		PCL_ERROR("Error writing point cloud %s\n", saveFileName.toStdString().c_str());
		return;
	}
	else
	{
		QMessageBox::information(this, QStringLiteral("提示"), QStringLiteral("滤波完成"), QStringLiteral("关闭"));
		emit closeFilterDialogSignal();
	}
}
void LidarMaster::rectoLeftSlot()
{
	if (m_mapCloud.size() <= 1)
		return;
	if (m_nCloudIndex.load() == 0)
		return;
}
void LidarMaster::rectoRightSlot()
{
	if (m_mapCloud.size() <= 1)
		return;
	if (m_nCloudIndex.load() == 0)
		return;
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
	lab_cprt->setStyleSheet("color:#000000;margin-right:10px;");
	stBar->addPermanentWidget(lab_cprt);
	stBar->setStyleSheet("background:#ffffff;border: 1px solid black;");

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
	parentItem1->setIcon(0, QIcon(":/LidarMaster/img/info.png"));
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

	QTreeWidgetItem* parentItem2 = new QTreeWidgetItem(QStringList() << QStringLiteral("高度信息"));
	parentItem2->setIcon(0, QIcon(":/LidarMaster/img/height.png"));
	m_PtrLasInfoTree->addTopLevelItem(parentItem2);
	QTreeWidgetItem* item21 = new QTreeWidgetItem();
	item21->setText(0, QStringLiteral("最大高度"));
	item21->setText(1, QStringLiteral(""));

	QTreeWidgetItem* item22 = new QTreeWidgetItem();
	item22->setText(0, QStringLiteral("最小高度"));
	item22->setText(1, QStringLiteral(""));
	QTreeWidgetItem* item23 = new QTreeWidgetItem();
	item23->setText(0, QStringLiteral("平均高度"));
	item23->setText(1, QStringLiteral(""));

	parentItem2->addChild(item21);
	parentItem2->addChild(item22);
	parentItem2->addChild(item23);

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
   
	getPtCLoud(lidarFile, m_Cloud);

	m_PtrLasInfoTree->topLevelItem(0)->child(0)->setText(1, QString::number(m_Cloud->points.size()));
	m_PtrLasInfoTree->topLevelItem(0)->child(1)->setText(1, QFileInfo(lidarFile).suffix());
	if (m_Cloud->height == 1)
	{
		m_PtrLasInfoTree->topLevelItem(0)->child(2)->setText(1, QStringLiteral("无序点云"));
	}
	else
	{
		m_PtrLasInfoTree->topLevelItem(0)->child(2)->setText(1, QStringLiteral("有序点云"));
	}
	if (m_MainThread != nullptr)
		m_MainThread->join();
	m_MainThread.reset(new std::thread([=] {showPtCloudHeightInfo(); }));
   
    // 显示结果图

    viewer->setBackgroundColor(0, 0, 0); //设置背景
    // viewer->addCoordinateSystem (15.0); //设置坐标系
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> fildColor(m_Cloud, "z");
    viewer->addPointCloud<pcl::PointXYZ>(m_Cloud, fildColor, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->resetCamera();
	m_LidarWidget->update();

	saveCloudToMap(m_Cloud);
}

void LidarMaster::showPtCloudHeightInfo()
{
	double aveHeight = 0, maxHeight = 0,minHeight  = DBL_MAX;

	if (!m_Cloud->empty())
	{	
		double totalHeight = 0;
		for (int i = 0; i < m_Cloud->points.size(); i++)
		{
			maxHeight = maxHeight > m_Cloud->points[i].z ? maxHeight : m_Cloud->points[i].z;
			minHeight = minHeight > m_Cloud->points[i].z ?  m_Cloud->points[i].z : minHeight;
			totalHeight += m_Cloud->points[i].z;
		}
		aveHeight = totalHeight / (m_Cloud->points.size()*1.0);

		m_PtrLasInfoTree->topLevelItem(1)->child(0)->setText(1, QString::number(maxHeight));
		m_PtrLasInfoTree->topLevelItem(1)->child(1)->setText(1, QString::number(minHeight));
		m_PtrLasInfoTree->topLevelItem(1)->child(2)->setText(1, QString::number(aveHeight));
	}
}

void LidarMaster::getPtCLoud(QString& lidarFile,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	if (QFileInfo(lidarFile).suffix().contains("pcd"))
	{
		pcl::PCDReader reader;
		reader.read(lidarFile.toStdString(), *cloud);
	}
	else if (QFileInfo(lidarFile).suffix().contains("ply"))
	{
		pcl::PolygonMesh meshData;//读取原始数据
		pcl::io::loadPolygonFile(lidarFile.toStdString(), meshData);
		pcl::fromPCLPointCloud2(meshData.cloud, *cloud);//将obj数据转换为点云数据
	}
	else if (QFileInfo(lidarFile).suffix().contains("las"))
	{
		LASreadOpener lasreadopener;
		QByteArray ba = lidarFile.toLatin1();
		lasreadopener.set_file_name(ba.data());
		LASreader* lasreader = lasreadopener.open(false);
		size_t ct = lasreader->header.number_of_point_records;
		cloud->points.resize(ct);
		cloud->width = 1;
		cloud->height = ct;
		cloud->is_dense = false;
		size_t i = 0;
		while (lasreader->read_point() && i < ct)
		{
			cloud->points[i].x = lasreader->point.get_x();
			cloud->points[i].y = lasreader->point.get_y();
			cloud->points[i].z = lasreader->point.get_z();
			++i;
		}
	}
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
