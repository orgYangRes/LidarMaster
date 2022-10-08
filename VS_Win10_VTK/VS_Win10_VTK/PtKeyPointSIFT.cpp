#include "PtKeyPointSIFT.h"

PtKeyPointSIFT::PtKeyPointSIFT(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
}

PtKeyPointSIFT::~PtKeyPointSIFT()
{
}

void PtKeyPointSIFT::on_pb_SIFT_OK_clicked()
{
	if (ui.ld_std->text().isEmpty() || ui.ld_level->text().isEmpty() || ui.ld_num->text().isEmpty() || ui.ld_level->text().isEmpty())
		return;
	float std = ui.ld_std->text().toFloat();
	int level = ui.ld_level->text().toInt();
	int num = ui.ld_num->text().toInt();
	float val = ui.ld_level->text().toFloat();
	emit sendSIFTval(std, level, num, val);
}
void PtKeyPointSIFT::closeSIFTDialogslot()
{
	this->close();
}
