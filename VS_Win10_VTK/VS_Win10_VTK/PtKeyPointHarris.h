#pragma once

#include <QDialog>
#include "ui_PtKeyPointHarris.h"

class PtKeyPointHarris : public QDialog
{
	Q_OBJECT

public:
	PtKeyPointHarris(QWidget *parent = nullptr);
	~PtKeyPointHarris();

private:
	Ui::PtKeyPointHarrisClass ui;
private slots:
	void closeHarrisDialogslot();
signals:
	void sendHarrisArgument(float normal,float check,float thr);
};
