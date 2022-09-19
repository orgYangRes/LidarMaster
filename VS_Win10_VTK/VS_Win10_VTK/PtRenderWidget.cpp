#include "PtRenderWidget.h"

PtRenderWidget::PtRenderWidget(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	connect(ui.radio_X, SIGNAL(toggled(bool)), this, SLOT(recRadioCliked()));
	connect(ui.radio_Y, SIGNAL(toggled(bool)), this, SLOT(recRadioCliked()));
	connect(ui.radio_Z, SIGNAL(toggled(bool)), this, SLOT(recRadioCliked()));
}

PtRenderWidget::~PtRenderWidget()
{}
void PtRenderWidget::on_pbt_ok_clicked()
{

	emit sendData(m_axisVal);
	this->close();
}

void PtRenderWidget::recRadioCliked()
{
	if (ui.radio_X->isChecked())
	{
		m_axisVal = "x";
	}
	else if (ui.radio_Y->isChecked())
	{
		m_axisVal = "y";
	}
	else if (ui.radio_Z->isChecked())
	{
		m_axisVal = "z";
	}
}
