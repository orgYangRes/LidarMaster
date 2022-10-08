#pragma once

#include <QDialog>
#include "ui_PtKeyPointSIFT.h"

class PtKeyPointSIFT : public QDialog
{
	Q_OBJECT

public:
	PtKeyPointSIFT(QWidget *parent = nullptr);
	~PtKeyPointSIFT();

private:
	Ui::PtKeyPointSIFTClass ui;
signals:
	void sendSIFTval(float std, int level, int nums, float val);
private slots:
	void on_pb_SIFT_OK_clicked();
	void closeSIFTDialogslot();
};
