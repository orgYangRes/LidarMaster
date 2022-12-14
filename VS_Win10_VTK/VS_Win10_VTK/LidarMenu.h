#pragma once

#include <QObject>
#include <QMenuBar>
#include <QtWidgets>
#include "LidarMaster.h"
#include <QPointer>
#include <QToolBar>
#include <QAction>
#include "LidarNewPro.h"
#include "PtRenderWidget.h"
#include <qcolordialog.h>
#include <qcolor.h>
#include "PtFilterDialog.h"
#include "PtGridFilterDialog.h"
#include "PtKeyPointSIFT.h"
#include "PtKeyPointHarris.h"
class LidarMaster;
class LidarNewPro;
class PtCloudRender;
class LidarMenu  : public QMenuBar
{
	Q_OBJECT

public:
	LidarMenu(QWidget*parent, LidarMaster *lasMaster);
	~LidarMenu();

	// 是否显示工程区域
	QPointer<QAction> m_isShowProInfoAct;

	// 是否显示点云信息区域
	QPointer<QAction> m_isShowLasInfoAct;

	// 是否显示其他信息区域
	QPointer<QAction> m_isShowOtherInfoAct;

private:
	//主框架指针
	LidarMaster *m_PtrLidarMaster;
	// 工具栏
	QPointer<QToolBar> m_PtrToolBar;
	//新建工程
	QPointer<LidarNewPro>m_PtrNewPro;
	//渲染对话框
	QPointer< PtRenderWidget> m_PtrCloudRender;

	//滤波对话框
	QPointer<PtFilterDialog>m_PtrFilterDialog;

	//按点采样对话框
	QPointer<PtGridFilterDialog>m_PtGridFilterDialog;

	//SIFT关键点显示对话框
	QPointer<PtKeyPointSIFT>m_PtSIFTKeyPoint;

	//Harris关键点显示对话框
	QPointer<PtKeyPointHarris>m_PtKeyPointHarris;
	
	// 文件模块
	void menu_File();
	// 视图模块
	void menu_ShowWin();
	// 点云设置
	void menu_PtCloudSetup();
	//点云滤波
	void menu_PtFilter();

	// 关键点显示
	void menu_showKeyPt();

	//点云配准
	void menu_registPt();

	//点云分割
	void menu_ClipPt();

	//点云重建
	void menu_reBuild();

	//帮助与关于
	void menu_about();

signals:
	void sendLidarColor(QColor& color);

private slots:
	//新建工程
	void File_new();
	//新建工程
	void Pro_Open();

	// 是否显示工程区域
	void showProInfo();
	// 是否显示点云信息区域
	void showLasInfo();
	// 是否显示其他信息区域
	void showOtherInfo();

	//显示渲染对话框
	void showRenderDialog();
	// 显示点云颜色框
	void showColorDialog();

	//滤波对话框
	void showFilterDialog();
	// 关键点显示
	void showKeypoint();

	// 关键点显示
	void  showHarrisKeypoint();
	// 下采样对话框
	void showGridFilterDialog();


};
