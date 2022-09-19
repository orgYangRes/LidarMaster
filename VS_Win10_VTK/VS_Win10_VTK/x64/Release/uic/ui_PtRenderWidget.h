/********************************************************************************
** Form generated from reading UI file 'PtRenderWidget.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PTRENDERWIDGET_H
#define UI_PTRENDERWIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_PtRenderWidgetClass
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QRadioButton *radio_X;
    QRadioButton *radio_Y;
    QRadioButton *radio_Z;
    QPushButton *pbt_ok;

    void setupUi(QDialog *PtRenderWidgetClass)
    {
        if (PtRenderWidgetClass->objectName().isEmpty())
            PtRenderWidgetClass->setObjectName(QString::fromUtf8("PtRenderWidgetClass"));
        PtRenderWidgetClass->resize(304, 64);
        QIcon icon;
        icon.addFile(QString::fromUtf8("img/coords.png"), QSize(), QIcon::Normal, QIcon::Off);
        PtRenderWidgetClass->setWindowIcon(icon);
        verticalLayout = new QVBoxLayout(PtRenderWidgetClass);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        radio_X = new QRadioButton(PtRenderWidgetClass);
        radio_X->setObjectName(QString::fromUtf8("radio_X"));

        horizontalLayout->addWidget(radio_X);

        radio_Y = new QRadioButton(PtRenderWidgetClass);
        radio_Y->setObjectName(QString::fromUtf8("radio_Y"));

        horizontalLayout->addWidget(radio_Y);

        radio_Z = new QRadioButton(PtRenderWidgetClass);
        radio_Z->setObjectName(QString::fromUtf8("radio_Z"));

        horizontalLayout->addWidget(radio_Z);

        pbt_ok = new QPushButton(PtRenderWidgetClass);
        pbt_ok->setObjectName(QString::fromUtf8("pbt_ok"));

        horizontalLayout->addWidget(pbt_ok);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(PtRenderWidgetClass);

        QMetaObject::connectSlotsByName(PtRenderWidgetClass);
    } // setupUi

    void retranslateUi(QDialog *PtRenderWidgetClass)
    {
        PtRenderWidgetClass->setWindowTitle(QCoreApplication::translate("PtRenderWidgetClass", "\347\202\271\344\272\221\346\270\262\346\237\223", nullptr));
        radio_X->setText(QCoreApplication::translate("PtRenderWidgetClass", "X\345\217\202\346\225\260", nullptr));
        radio_Y->setText(QCoreApplication::translate("PtRenderWidgetClass", "Y\345\217\202\346\225\260", nullptr));
        radio_Z->setText(QCoreApplication::translate("PtRenderWidgetClass", "Z\345\217\202\346\225\260", nullptr));
        pbt_ok->setText(QCoreApplication::translate("PtRenderWidgetClass", "\347\241\256\350\256\244", nullptr));
    } // retranslateUi

};

namespace Ui {
    class PtRenderWidgetClass: public Ui_PtRenderWidgetClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PTRENDERWIDGET_H
