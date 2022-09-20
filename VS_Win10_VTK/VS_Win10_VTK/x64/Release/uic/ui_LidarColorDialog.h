/********************************************************************************
** Form generated from reading UI file 'LidarColorDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LIDARCOLORDIALOG_H
#define UI_LIDARCOLORDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LidarColorDialogClass
{
public:
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QWidget *centralWidget;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *LidarColorDialogClass)
    {
        if (LidarColorDialogClass->objectName().isEmpty())
            LidarColorDialogClass->setObjectName(QString::fromUtf8("LidarColorDialogClass"));
        LidarColorDialogClass->resize(600, 400);
        menuBar = new QMenuBar(LidarColorDialogClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        LidarColorDialogClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(LidarColorDialogClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        LidarColorDialogClass->addToolBar(mainToolBar);
        centralWidget = new QWidget(LidarColorDialogClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        LidarColorDialogClass->setCentralWidget(centralWidget);
        statusBar = new QStatusBar(LidarColorDialogClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        LidarColorDialogClass->setStatusBar(statusBar);

        retranslateUi(LidarColorDialogClass);

        QMetaObject::connectSlotsByName(LidarColorDialogClass);
    } // setupUi

    void retranslateUi(QMainWindow *LidarColorDialogClass)
    {
        LidarColorDialogClass->setWindowTitle(QCoreApplication::translate("LidarColorDialogClass", "LidarColorDialog", nullptr));
    } // retranslateUi

};

namespace Ui {
    class LidarColorDialogClass: public Ui_LidarColorDialogClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIDARCOLORDIALOG_H
