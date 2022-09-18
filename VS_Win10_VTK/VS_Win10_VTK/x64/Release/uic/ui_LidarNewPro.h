/********************************************************************************
** Form generated from reading UI file 'LidarNewPro.ui'
**
** Created by: Qt User Interface Compiler version 5.14.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LIDARNEWPRO_H
#define UI_LIDARNEWPRO_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LidarNewProClass
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout;
    QLabel *lab_ProName;
    QLineEdit *ld_ProName;
    QToolButton *tb_proPath;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *ld_lasPath;
    QToolButton *tb_OpenLas;
    QHBoxLayout *horizontalLayout_3;
    QSpacerItem *horizontalSpacer;
    QToolButton *tb_OK;
    QToolButton *tb_cancel;
    QSpacerItem *horizontalSpacer_2;

    void setupUi(QWidget *LidarNewProClass)
    {
        if (LidarNewProClass->objectName().isEmpty())
            LidarNewProClass->setObjectName(QString::fromUtf8("LidarNewProClass"));
        LidarNewProClass->resize(425, 135);
        LidarNewProClass->setMinimumSize(QSize(425, 135));
        LidarNewProClass->setMaximumSize(QSize(425, 135));
        QIcon icon;
        icon.addFile(QString::fromUtf8("img/logo.png"), QSize(), QIcon::Normal, QIcon::Off);
        LidarNewProClass->setWindowIcon(icon);
        verticalLayout = new QVBoxLayout(LidarNewProClass);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        lab_ProName = new QLabel(LidarNewProClass);
        lab_ProName->setObjectName(QString::fromUtf8("lab_ProName"));

        horizontalLayout->addWidget(lab_ProName);

        ld_ProName = new QLineEdit(LidarNewProClass);
        ld_ProName->setObjectName(QString::fromUtf8("ld_ProName"));
        ld_ProName->setMinimumSize(QSize(300, 24));
        ld_ProName->setMaximumSize(QSize(300, 24));

        horizontalLayout->addWidget(ld_ProName);

        tb_proPath = new QToolButton(LidarNewProClass);
        tb_proPath->setObjectName(QString::fromUtf8("tb_proPath"));
        tb_proPath->setMinimumSize(QSize(32, 32));
        tb_proPath->setMaximumSize(QSize(32, 32));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8("img/open.png"), QSize(), QIcon::Normal, QIcon::Off);
        tb_proPath->setIcon(icon1);

        horizontalLayout->addWidget(tb_proPath);


        verticalLayout->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        label_2 = new QLabel(LidarNewProClass);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        horizontalLayout_2->addWidget(label_2);

        ld_lasPath = new QLineEdit(LidarNewProClass);
        ld_lasPath->setObjectName(QString::fromUtf8("ld_lasPath"));
        ld_lasPath->setMinimumSize(QSize(300, 24));
        ld_lasPath->setMaximumSize(QSize(300, 24));

        horizontalLayout_2->addWidget(ld_lasPath);

        tb_OpenLas = new QToolButton(LidarNewProClass);
        tb_OpenLas->setObjectName(QString::fromUtf8("tb_OpenLas"));
        tb_OpenLas->setMinimumSize(QSize(32, 32));
        tb_OpenLas->setMaximumSize(QSize(32, 32));
        tb_OpenLas->setIcon(icon1);

        horizontalLayout_2->addWidget(tb_OpenLas);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer);

        tb_OK = new QToolButton(LidarNewProClass);
        tb_OK->setObjectName(QString::fromUtf8("tb_OK"));
        tb_OK->setMinimumSize(QSize(64, 32));
        tb_OK->setMaximumSize(QSize(64, 32));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8("img/ok.png"), QSize(), QIcon::Normal, QIcon::Off);
        tb_OK->setIcon(icon2);

        horizontalLayout_3->addWidget(tb_OK);

        tb_cancel = new QToolButton(LidarNewProClass);
        tb_cancel->setObjectName(QString::fromUtf8("tb_cancel"));
        tb_cancel->setMinimumSize(QSize(64, 32));
        tb_cancel->setMaximumSize(QSize(64, 32));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8("img/cancel.png"), QSize(), QIcon::Normal, QIcon::Off);
        tb_cancel->setIcon(icon3);

        horizontalLayout_3->addWidget(tb_cancel);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_3->addItem(horizontalSpacer_2);


        verticalLayout->addLayout(horizontalLayout_3);


        retranslateUi(LidarNewProClass);

        QMetaObject::connectSlotsByName(LidarNewProClass);
    } // setupUi

    void retranslateUi(QWidget *LidarNewProClass)
    {
        LidarNewProClass->setWindowTitle(QCoreApplication::translate("LidarNewProClass", "\346\226\260\345\273\272\345\267\245\347\250\213", nullptr));
        lab_ProName->setText(QCoreApplication::translate("LidarNewProClass", "\345\267\245\347\250\213\350\267\257\345\276\204:", nullptr));
        tb_proPath->setText(QString());
        label_2->setText(QCoreApplication::translate("LidarNewProClass", "\347\202\271\344\272\221\350\267\257\345\276\204:", nullptr));
        tb_OpenLas->setText(QString());
        tb_OK->setText(QString());
        tb_cancel->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class LidarNewProClass: public Ui_LidarNewProClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIDARNEWPRO_H
