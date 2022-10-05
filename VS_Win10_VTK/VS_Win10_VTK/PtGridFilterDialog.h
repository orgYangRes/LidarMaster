#pragma once

#include <QDialog>
#include "ui_PtGridFilterDialog.h"

class PtGridFilterDialog : public QDialog
{
	Q_OBJECT

public:
	PtGridFilterDialog(QWidget *parent = nullptr);
	~PtGridFilterDialog();

private:
	Ui::PtGridFilterDialogClass ui;

private slots:
	void on_pb_Confirm_clicked();

	void closeDialog();
signals:
	void sendGridAndType(int gridVal, int type);
};
