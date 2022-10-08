#include "PtKeyPointHarris.h"

PtKeyPointHarris::PtKeyPointHarris(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

PtKeyPointHarris::~PtKeyPointHarris()
{
	if (ui.ld_normal_radius->text().isEmpty() || ui.ld_check_radius->text().isEmpty() || ui.ld_threshlod->text().isEmpty())
	{
		return;
	}
	float  noraml = ui.ld_normal_radius->text().toFloat();
	float  checkR = ui.ld_check_radius->text().toFloat();
	float  thr = ui.ld_threshlod->text().toFloat();

	emit sendHarrisArgument(noraml,checkR,thr);
}

void PtKeyPointHarris::closeHarrisDialogslot()
{
	this->close();
}