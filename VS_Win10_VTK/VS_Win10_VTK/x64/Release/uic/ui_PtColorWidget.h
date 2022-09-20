/********************************************************************************
** Form generated from reading UI file 'PtColorWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PTCOLORWIDGET_H
#define UI_PTCOLORWIDGET_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_PtColorWidgetClass
{
public:

    void setupUi(QWidget *PtColorWidgetClass)
    {
        if (PtColorWidgetClass->objectName().isEmpty())
            PtColorWidgetClass->setObjectName(QString::fromUtf8("PtColorWidgetClass"));
        PtColorWidgetClass->resize(600, 400);

        retranslateUi(PtColorWidgetClass);

        QMetaObject::connectSlotsByName(PtColorWidgetClass);
    } // setupUi

    void retranslateUi(QWidget *PtColorWidgetClass)
    {
        PtColorWidgetClass->setWindowTitle(QCoreApplication::translate("PtColorWidgetClass", "PtColorWidget", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PtColorWidgetClass: public Ui_PtColorWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PTCOLORWIDGET_H
