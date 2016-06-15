/********************************************************************************
** Form generated from reading UI file 'adafruit_widget.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ADAFRUIT_WIDGET_H
#define UI_ADAFRUIT_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_AdafruitWidget
{
public:
    QGridLayout *gridLayout;
    QPushButton *pushButton_Open;
    QPushButton *pushButton_Close;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QPushButton *pushButton_Quaternion;
    QLabel *label_Calib;
    QPushButton *pushButton_Calib;
    QLabel *label_Quaternion;
    QPushButton *pushButton_Init;

    void setupUi(QWidget *AdafruitWidget)
    {
        if (AdafruitWidget->objectName().isEmpty())
            AdafruitWidget->setObjectName(QString::fromUtf8("AdafruitWidget"));
        AdafruitWidget->resize(435, 294);
        gridLayout = new QGridLayout(AdafruitWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        pushButton_Open = new QPushButton(AdafruitWidget);
        pushButton_Open->setObjectName(QString::fromUtf8("pushButton_Open"));

        gridLayout->addWidget(pushButton_Open, 0, 0, 1, 1);

        pushButton_Close = new QPushButton(AdafruitWidget);
        pushButton_Close->setObjectName(QString::fromUtf8("pushButton_Close"));

        gridLayout->addWidget(pushButton_Close, 1, 0, 1, 1);

        groupBox = new QGroupBox(AdafruitWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setEnabled(false);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        pushButton_Quaternion = new QPushButton(groupBox);
        pushButton_Quaternion->setObjectName(QString::fromUtf8("pushButton_Quaternion"));

        gridLayout_2->addWidget(pushButton_Quaternion, 2, 0, 1, 1);

        label_Calib = new QLabel(groupBox);
        label_Calib->setObjectName(QString::fromUtf8("label_Calib"));

        gridLayout_2->addWidget(label_Calib, 3, 1, 1, 1);

        pushButton_Calib = new QPushButton(groupBox);
        pushButton_Calib->setObjectName(QString::fromUtf8("pushButton_Calib"));

        gridLayout_2->addWidget(pushButton_Calib, 3, 0, 1, 1);

        label_Quaternion = new QLabel(groupBox);
        label_Quaternion->setObjectName(QString::fromUtf8("label_Quaternion"));

        gridLayout_2->addWidget(label_Quaternion, 2, 1, 1, 1);

        pushButton_Init = new QPushButton(groupBox);
        pushButton_Init->setObjectName(QString::fromUtf8("pushButton_Init"));

        gridLayout_2->addWidget(pushButton_Init, 1, 0, 1, 1);


        gridLayout->addWidget(groupBox, 2, 0, 1, 1);


        retranslateUi(AdafruitWidget);
        QObject::connect(pushButton_Open, SIGNAL(clicked()), AdafruitWidget, SLOT(openConnection()));
        QObject::connect(pushButton_Close, SIGNAL(clicked()), AdafruitWidget, SLOT(closeConnection()));
        QObject::connect(pushButton_Quaternion, SIGNAL(clicked()), AdafruitWidget, SLOT(getQuaternion()));
        QObject::connect(pushButton_Init, SIGNAL(clicked()), AdafruitWidget, SLOT(initializeSensor()));

        QMetaObject::connectSlotsByName(AdafruitWidget);
    } // setupUi

    void retranslateUi(QWidget *AdafruitWidget)
    {
        AdafruitWidget->setWindowTitle(QApplication::translate("AdafruitWidget", "Form", 0, QApplication::UnicodeUTF8));
        pushButton_Open->setText(QApplication::translate("AdafruitWidget", "Open", 0, QApplication::UnicodeUTF8));
        pushButton_Close->setText(QApplication::translate("AdafruitWidget", "Close", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("AdafruitWidget", "Measurements", 0, QApplication::UnicodeUTF8));
        pushButton_Quaternion->setText(QApplication::translate("AdafruitWidget", "Get Quat", 0, QApplication::UnicodeUTF8));
        label_Calib->setText(QApplication::translate("AdafruitWidget", "Calib", 0, QApplication::UnicodeUTF8));
        pushButton_Calib->setText(QApplication::translate("AdafruitWidget", "Get Calib", 0, QApplication::UnicodeUTF8));
        label_Quaternion->setText(QApplication::translate("AdafruitWidget", "Quaternion", 0, QApplication::UnicodeUTF8));
        pushButton_Init->setText(QApplication::translate("AdafruitWidget", "Init", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class AdafruitWidget: public Ui_AdafruitWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ADAFRUIT_WIDGET_H
