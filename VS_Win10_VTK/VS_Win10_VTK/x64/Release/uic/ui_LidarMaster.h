/********************************************************************************
** Form generated from reading UI file 'LidarMaster.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LIDARMASTER_H
#define UI_LIDARMASTER_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LidarMasterClass
{
public:
    QWidget *centralWidget;

    void setupUi(QMainWindow *LidarMasterClass)
    {
        if (LidarMasterClass->objectName().isEmpty())
            LidarMasterClass->setObjectName(QString::fromUtf8("LidarMasterClass"));
        LidarMasterClass->resize(1112, 756);
        centralWidget = new QWidget(LidarMasterClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        LidarMasterClass->setCentralWidget(centralWidget);

        retranslateUi(LidarMasterClass);

        QMetaObject::connectSlotsByName(LidarMasterClass);
    } // setupUi

    void retranslateUi(QMainWindow *LidarMasterClass)
    {
        LidarMasterClass->setWindowTitle(QCoreApplication::translate("LidarMasterClass", "LidarMaster", nullptr));
    } // retranslateUi

};

namespace Ui {
    class LidarMasterClass: public Ui_LidarMasterClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIDARMASTER_H
