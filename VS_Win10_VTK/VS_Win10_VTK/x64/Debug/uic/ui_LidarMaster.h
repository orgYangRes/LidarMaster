/********************************************************************************
** Form generated from reading UI file 'LidarMaster.ui'
**
** Created by: Qt User Interface Compiler version 5.11.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LIDARMASTER_H
#define UI_LIDARMASTER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_LidarMasterClass
{
public:
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout;
    QVTKWidget *qvtkWidget;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *LidarMasterClass)
    {
        if (LidarMasterClass->objectName().isEmpty())
            LidarMasterClass->setObjectName(QStringLiteral("LidarMasterClass"));
        LidarMasterClass->resize(926, 693);
        centralWidget = new QWidget(LidarMasterClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayout = new QVBoxLayout(centralWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));

        verticalLayout->addWidget(qvtkWidget);

        LidarMasterClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(LidarMasterClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 926, 23));
        LidarMasterClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(LidarMasterClass);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        LidarMasterClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(LidarMasterClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        LidarMasterClass->setStatusBar(statusBar);

        retranslateUi(LidarMasterClass);

        QMetaObject::connectSlotsByName(LidarMasterClass);
    } // setupUi

    void retranslateUi(QMainWindow *LidarMasterClass)
    {
        LidarMasterClass->setWindowTitle(QApplication::translate("LidarMasterClass", "LidarMaster", nullptr));
    } // retranslateUi

};

namespace Ui {
    class LidarMasterClass: public Ui_LidarMasterClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIDARMASTER_H
