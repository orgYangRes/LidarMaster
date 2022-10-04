#pragma once

#include <QDialog>
#include "ui_PtFilterDialog.h"

class PtFilterDialog : public QDialog
{
	Q_OBJECT

public:
	PtFilterDialog(QWidget *parent = nullptr);
	~PtFilterDialog();

private:
	Ui::PtFilterDialogClass ui;
private slots:
	void on_pbt_ok_clicked();
	void on_pbt_cancel_clicked();
	void on_pbt_browser_clicked();
	void recCloseFilterDialogSlot();
signals:
	void sendFilterVal(int type,double val,QString& lasFile);
};
