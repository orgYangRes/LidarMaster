#include "LidarNewPro.h"
#include <QFileDialog>	
#include <QMessageBox>
#include <QTreeWidgetItem>
#include <qfileinfo.h>
#include <QIcon>
#include <qjsonobject.h>
#include <qjsondocument.h>
LidarNewPro::LidarNewPro(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	this->setWindowFlags(Qt::WindowCloseButtonHint);
	
}

LidarNewPro::~LidarNewPro()
{

}
void LidarNewPro::setLidarMatser(LidarMaster* Matser)
{
	m_PtrLidarMaster = (LidarMaster*)Matser;
}
void LidarNewPro::on_tb_OK_clicked()
{
	QString proName = ui.ld_ProName->text();
	if (proName.isEmpty())
	{
		QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("工程路径未设置"));
		box.setStandardButtons(QMessageBox::Ok);
		box.setWindowFlags(Qt::WindowStaysOnTopHint | Qt::WindowCloseButtonHint);
		box.setButtonText(QMessageBox::Ok, QString(u8"确定"));
		box.exec();
		return;
	}

	QString lasPath = ui.ld_lasPath->text();

	if (lasPath.isEmpty())
	{
		QMessageBox box(QMessageBox::Warning, QStringLiteral("提示"), QStringLiteral("点云路径不能为空"));
		box.setStandardButtons(QMessageBox::Ok);
		box.setWindowFlags(Qt::WindowStaysOnTopHint | Qt::WindowCloseButtonHint);
		box.setButtonText(QMessageBox::Ok, QString(u8"确定"));
		box.exec();
		return;
	}
	m_proPath = proName;
	m_lasPath = lasPath;
	
	
	
	QJsonDocument jsonDoc;
	QJsonObject jsonObj;
	jsonObj.insert("proName", QFileInfo(proName).baseName());
	jsonObj.insert("proPath", proName);
	jsonObj.insert("lasName", QFileInfo(lasPath).fileName());
	jsonObj.insert("lasPath", lasPath);
	jsonDoc.setObject(jsonObj);
	QFile file(proName);
	if (file.open(QIODevice::ReadWrite | QIODevice::Text))
	{
		file.write(jsonDoc.toJson(QJsonDocument::Indented));
		file.close();
	}


	addTrees(lasPath, proName);

	this->close();
}

void LidarNewPro::addTrees(const QString& lasPath, const QString& proPath)
{
	int topCount = 0;
	topCount = m_PtrLidarMaster->m_PtrProTree->topLevelItemCount();

	QTreeWidgetItem* item1 = new QTreeWidgetItem();
	item1->setIcon(0, QIcon(":/LidarMaster/img/pro.png"));
	/*item1->setFlags(Qt::ItemIsEnabled  | Qt::ItemIsAutoTristate);*/
	item1->setText(0, QFileInfo(proPath).baseName());
	item1->setData(0, Qt::CheckStateRole, QVariant());

	QTreeWidgetItem* item11 = new QTreeWidgetItem();
	item11->setIcon(0, QIcon(":/LidarMaster/img/las.png"));
	item11->setCheckState(0, Qt::Unchecked);
	item11->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable | Qt::ItemIsAutoTristate);
	item11->setText(0, QFileInfo(lasPath).fileName());
	item11->setData(0, Qt::UserRole + 1, lasPath);
	item1->addChild(item11);
	m_PtrLidarMaster->m_PtrProTree->insertTopLevelItem(topCount, item1);
	m_PtrLidarMaster->m_PtrProTree->setItemsExpandable(true);
	m_PtrLidarMaster->m_PtrProTree->expandAll();
}
void LidarNewPro::on_tb_proPath_clicked()
{
	QString fileName = QFileDialog::getSaveFileName(this, QStringLiteral("工程路径"),"./", QStringLiteral("工程文件 (*.pri)"));
	if (!fileName.isEmpty())
	{
		ui.ld_ProName->setText(fileName);
	}

}
void LidarNewPro::on_tb_OpenLas_clicked()
{
	QFileDialog* fileDialog = new QFileDialog(this);
	fileDialog->setWindowTitle(QStringLiteral("选择点云文件"));
	fileDialog->setDirectory("./");
	fileDialog->setNameFilter(tr("las File(*.las* *.ply* *.pcd* *.txt*)"));
	// 多选
	fileDialog->setFileMode(QFileDialog::ExistingFiles);
	fileDialog->setViewMode(QFileDialog::Detail);
	QStringList fileNames;
	if (fileDialog->exec()) {
		fileNames = fileDialog->selectedFiles();
	}
	if (fileNames.size() > 0)
		ui.ld_lasPath->setText(fileNames[0]);
}