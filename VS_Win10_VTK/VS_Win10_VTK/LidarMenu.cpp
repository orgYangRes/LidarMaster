#include "LidarMenu.h"
#include <QAction>
#include <qdebug.h>

LidarMenu::LidarMenu(QWidget* parent, LidarMaster* lasMaster)
	: QMenuBar(parent)	
{
	m_PtrLidarMaster = (LidarMaster*) lasMaster;
	m_PtrLidarMaster = lasMaster;
	// 添加工具栏
	m_PtrToolBar = new QToolBar;
	m_PtrToolBar = m_PtrLidarMaster->addToolBar(QStringLiteral("文件工具栏"));
	m_PtrToolBar->setMovable(false);

	m_PtrNewPro = new LidarNewPro;
	m_PtrNewPro->setLidarMatser(lasMaster);

	m_PtrCloudRender = new PtRenderWidget(this);

	m_PtrFilterDialog = new PtFilterDialog(this);
	connect(m_PtrFilterDialog, SIGNAL(sendFilterVal(int,double,QString&)), m_PtrLidarMaster, SLOT(recvFilterVal(int,double, QString&)));
	connect(m_PtrLidarMaster, SIGNAL(closeFilterDialogSignal()), m_PtrFilterDialog, SLOT(recCloseFilterDialogSlot()));

	m_PtGridFilterDialog = new PtGridFilterDialog(this);
	connect(m_PtGridFilterDialog, SIGNAL(sendGridAndType(int, int)), m_PtrLidarMaster, SLOT(recvGridAndType(int, int)));
	connect(m_PtrLidarMaster, SIGNAL(closeGridFilterDialogSignal), m_PtGridFilterDialog, SLOT(closeDialog));

	connect(m_PtrCloudRender, SIGNAL(sendData(QString&)), m_PtrLidarMaster, SLOT(recvRenderCoords(QString&)));
	connect(this, SIGNAL(sendLidarColor(QColor&)), m_PtrLidarMaster, SLOT(recColorInfoSlot(QColor&)));
	menu_File();
	menu_PtCloudSetup();
	menu_PtFilter();


	menu_ShowWin();
}

LidarMenu::~LidarMenu()
{

}
/// @brief 新建工程
void LidarMenu::menu_File()
{
	QAction* Act_new_file = new QAction(QIcon(":/LidarMaster/img/new.png"), QStringLiteral("新建工程"), this);
	Act_new_file->setShortcut(Qt::Key_Control & Qt::Key_N);
	connect(Act_new_file, SIGNAL(triggered()), this, SLOT(File_new()));
	QAction* Act_open_pro= new QAction(QIcon(":/LidarMaster/img/open.png"), QStringLiteral("打开工程"), this);

	connect(Act_open_pro, SIGNAL(triggered()), this, SLOT(Pro_Open()));


	QMenu* file = addMenu(QStringLiteral("文件(N)"));
	file->addAction(Act_new_file);
	m_PtrToolBar->addAction(Act_new_file);
	file->addAction(Act_open_pro);
	m_PtrToolBar->addAction(Act_open_pro);


	QAction* Act_to_left = new QAction(QIcon(":/LidarMaster/img/toLeft.png"), QStringLiteral("撤销"), this);
	QAction* Act_to_right = new QAction(QIcon(":/LidarMaster/img/toright.png"), QStringLiteral("反撤销"), this);
	connect(Act_to_left, SIGNAL(triggered()), m_PtrLidarMaster, SLOT(rectoLeftSlot()));
	connect(Act_to_right, SIGNAL(triggered()), m_PtrLidarMaster, SLOT(rectoRightSlot()));

	m_PtrToolBar->addAction(Act_to_left);
	m_PtrToolBar->addAction(Act_to_right);
}
void LidarMenu::File_new()
{
	m_PtrNewPro->show();
}
void LidarMenu::menu_ShowWin()
{
	m_isShowProInfoAct = new QAction(QIcon(":/LidarMaster/img/show.png"), QStringLiteral("工程区域"), this);
	connect(m_isShowProInfoAct, SIGNAL(triggered()), this, SLOT(showProInfo()));

	m_isShowLasInfoAct = new QAction(QIcon(":/LidarMaster/img/show.png"), QStringLiteral("点云信息"), this);
	connect(m_isShowLasInfoAct, SIGNAL(triggered()), this, SLOT(showLasInfo()));

	m_isShowOtherInfoAct = new QAction(QIcon(":/LidarMaster/img/show.png"), QStringLiteral("其他信息"), this);
	connect(m_isShowOtherInfoAct, SIGNAL(triggered()), this, SLOT(showOtherInfo()));

	QMenu* showWin = addMenu(QStringLiteral("视图显示"));
	showWin->addAction(m_isShowProInfoAct);
	showWin->addAction(m_isShowLasInfoAct);
	showWin->addAction(m_isShowOtherInfoAct);
}
void LidarMenu::menu_PtCloudSetup()
{
	QAction* Act_pt_render = new QAction(QIcon(":/LidarMaster/img/render.png"), QStringLiteral("点云渲染"), this);
	connect(Act_pt_render, SIGNAL(triggered()), this, SLOT(showRenderDialog()));

	QAction* Act_pt_color = new QAction(QIcon(":/LidarMaster/img/color.png"), QStringLiteral("点云颜色"), this);
	connect(Act_pt_color, SIGNAL(triggered()), this, SLOT(showColorDialog()));

	QAction* Act_pt_size = new QAction(QIcon(":/LidarMaster/img/size.png"), QStringLiteral("点云大小"), this);


	QMenu* file = addMenu(QStringLiteral("点云渲染"));
	file->addAction(Act_pt_render);
	file->addAction(Act_pt_color);
	file->addAction(Act_pt_size);
}
void LidarMenu::menu_PtFilter()
{
	QAction* Act_pt_filter1 = new QAction(QIcon(":/LidarMaster/img/gridfilter.png"), QStringLiteral("体素滤波"), this);
	connect(Act_pt_filter1, SIGNAL(triggered()), this, SLOT(showFilterDialog()));


	QAction* Act_pt_filter3 = new QAction(QIcon(":/LidarMaster/img/filter1.png"), QStringLiteral("采样滤波"), this);
	connect(Act_pt_filter3, SIGNAL(triggered()), this, SLOT(showGridFilterDialog()));

	QMenu* filterMenu = addMenu(QStringLiteral("点云滤波"));
	filterMenu->addAction(Act_pt_filter1);
	
	filterMenu->addAction(Act_pt_filter3);
}
void LidarMenu::Pro_Open()
{
	QFileDialog* fileDialog = new QFileDialog(this);
	fileDialog->setWindowTitle(QStringLiteral("选择工程文件"));
	fileDialog->setDirectory("./");
	fileDialog->setNameFilter(tr("las File(*.pri*)"));
	// 多选
	fileDialog->setFileMode(QFileDialog::ExistingFile);
	fileDialog->setViewMode(QFileDialog::Detail);
	QStringList fileNameList;
	if (fileDialog->exec()) {
		fileNameList = fileDialog->selectedFiles();
	}
	if (fileNameList.size() <= 0)
		return;
	QString fileName = fileNameList.at(0);
	if (!fileName.isEmpty())
	{
		QFile file(fileName);
		if (file.open(QIODevice::ReadOnly))
		{
			QByteArray data = file.readAll();
			QJsonDocument doc = QJsonDocument::fromJson(data);
			if (doc.isObject())
			{
				QJsonObject jsonObj = doc.object();

				QString lasPath = jsonObj["lasPath"].toString();
				QString proPath = jsonObj["proPath"].toString();

				m_PtrNewPro->addTrees(lasPath, proPath);
			}
			file.close();
		}
	}
}

void LidarMenu::showProInfo()
{
	if (m_PtrLidarMaster->m_dockProInfo->isHidden())
	{
		m_isShowProInfoAct->setIcon(QIcon(":/LidarMaster/img/show.png"));
		m_PtrLidarMaster->m_dockProInfo->show();
	}
	else
	{
		m_isShowProInfoAct->setIcon(QIcon(":/LidarMaster/img/hide.png"));
		m_PtrLidarMaster->m_dockProInfo->hide();
	}
}

void LidarMenu::showLasInfo()
{
	if (m_PtrLidarMaster->m_dockLasInfo->isHidden())
	{
		m_isShowLasInfoAct->setIcon(QIcon(":/LidarMaster/img/show.png"));
		m_PtrLidarMaster->m_dockLasInfo->show();
	}
	else
	{
		m_isShowLasInfoAct->setIcon(QIcon(":/LidarMaster/img/hide.png"));
		m_PtrLidarMaster->m_dockLasInfo->hide();
	}
}

void LidarMenu::showOtherInfo()
{
	if (m_PtrLidarMaster->m_dockOtherInfo->isHidden())
	{
		m_isShowOtherInfoAct->setIcon(QIcon(":/LidarMaster/img/show.png"));
		m_PtrLidarMaster->m_dockOtherInfo->show();
	}
	else
	{
		m_isShowOtherInfoAct->setIcon(QIcon(":/LidarMaster/img/hide.png"));
		m_PtrLidarMaster->m_dockOtherInfo->hide();
	}
}
void LidarMenu::showRenderDialog()
{
	m_PtrCloudRender->show();
}

void LidarMenu::showColorDialog()
{
	QColor color = QColorDialog::getColor(Qt::white,this);
	sendLidarColor(color);
}

void LidarMenu::showFilterDialog()
{
	m_PtrFilterDialog->show();
}

void LidarMenu::showGridFilterDialog()
{
	m_PtGridFilterDialog->show();
}

