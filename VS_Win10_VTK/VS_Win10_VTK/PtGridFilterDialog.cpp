#include "PtGridFilterDialog.h"
#include <QRegExpValidator>
PtGridFilterDialog::PtGridFilterDialog(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	QRegExp rx("^[1-9][0-9]*$");
	QRegExpValidator* validator = new QRegExpValidator(rx, this);
	ui.ld_gdd->setValidator(validator);
	ui.ld_jyd->setValidator(validator);
	ui.ld_tj->setValidator(validator);
}

PtGridFilterDialog::~PtGridFilterDialog()
{
}
void PtGridFilterDialog::closeDialog()
{
	this->close();
}
void PtGridFilterDialog::on_pb_Confirm_clicked()
{
	int index = ui.tabWidget->currentIndex();

	int gridVal = 0;
	switch (index)
	{
	case 0:
		gridVal = ui.ld_gdd->text().toInt();
		break;
	case 1:
		gridVal = ui.ld_jyd->text().toInt();
		break;
	case 2:
		gridVal = ui.ld_tj->text().toInt();
		break;
	default:
		break;
	}
	emit sendGridAndType(gridVal,index);
}