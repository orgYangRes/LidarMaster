#include "PtFilterDialog.h"
#include <QFileDialog>
PtFilterDialog::PtFilterDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

PtFilterDialog::~PtFilterDialog()
{

}

void PtFilterDialog::on_pbt_ok_clicked()
{
	QString lasFile = ui.ld_lasFile->text();
	if (lasFile.isEmpty()) return;


	if (!ui.ld_filterVal->text().isEmpty())
	{
		QString str = ui.ld_filterVal->text();
		if(ui.rb_jstsu->isChecked())
		  emit sendFilterVal(2,str.toDouble(), lasFile);
		else if(ui.rb_tisuFilter->isChecked())
			emit sendFilterVal(1, str.toDouble(), lasFile);
	}
}

void PtFilterDialog::on_pbt_cancel_clicked()
{
	this->close();
}

void PtFilterDialog::on_pbt_browser_clicked()
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
		ui.ld_lasFile->setText(fileNames[0]);
}

void PtFilterDialog::recCloseFilterDialogSlot()
{
	this->close();
}

