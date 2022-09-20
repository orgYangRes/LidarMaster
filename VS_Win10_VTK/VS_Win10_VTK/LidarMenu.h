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
	
	// 文件模块
	void menu_File();
	// 视图模块
	void menu_ShowWin();
	// 点云设置
	void menu_PtCloudSetup();

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



};
