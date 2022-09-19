#pragma once

#include <QDialog>
#include "ui_PtRenderWidget.h"

class PtRenderWidget : public QDialog
{
	Q_OBJECT

public:
	PtRenderWidget(QWidget *parent = nullptr);
	~PtRenderWidget();

private:
	Ui::PtRenderWidgetClass ui;
	QString m_axisVal;
private slots:

	void recRadioCliked();
	void on_pbt_ok_clicked();
signals:
	void sendData(QString& data);
};
