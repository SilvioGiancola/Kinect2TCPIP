/********************************************************************************
** Form generated from reading UI file 'yei_widget.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_YEI_WIDGET_H
#define UI_YEI_WIDGET_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_YEIWidget
{
public:
    QGridLayout *gridLayout;
    QCheckBox *checkBox;
    QPushButton *pushButton_Open;
    QPushButton *pushButton_Close;
    QGroupBox *groupBox;
    QGridLayout *gridLayout_2;
    QPushButton *pushButton_Quaternion;
    QLabel *label_Quaternion;
    QLabel *label;
    QPushButton *pushButton_TareQuat;
    QLabel *label_AngleDegree;
    QLineEdit *lineEdit_path;

    void setupUi(QWidget *YEIWidget)
    {
        if (YEIWidget->objectName().isEmpty())
            YEIWidget->setObjectName(QString::fromUtf8("YEIWidget"));
        YEIWidget->resize(543, 361);
        gridLayout = new QGridLayout(YEIWidget);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        checkBox = new QCheckBox(YEIWidget);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setChecked(true);

        gridLayout->addWidget(checkBox, 3, 0, 1, 1);

        pushButton_Open = new QPushButton(YEIWidget);
        pushButton_Open->setObjectName(QString::fromUtf8("pushButton_Open"));

        gridLayout->addWidget(pushButton_Open, 0, 0, 1, 1);

        pushButton_Close = new QPushButton(YEIWidget);
        pushButton_Close->setObjectName(QString::fromUtf8("pushButton_Close"));

        gridLayout->addWidget(pushButton_Close, 1, 0, 1, 1);

        groupBox = new QGroupBox(YEIWidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setEnabled(false);
        gridLayout_2 = new QGridLayout(groupBox);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        pushButton_Quaternion = new QPushButton(groupBox);
        pushButton_Quaternion->setObjectName(QString::fromUtf8("pushButton_Quaternion"));

        gridLayout_2->addWidget(pushButton_Quaternion, 1, 0, 1, 1);

        label_Quaternion = new QLabel(groupBox);
        label_Quaternion->setObjectName(QString::fromUtf8("label_Quaternion"));

        gridLayout_2->addWidget(label_Quaternion, 1, 1, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout_2->addWidget(label, 2, 0, 1, 1);

        pushButton_TareQuat = new QPushButton(groupBox);
        pushButton_TareQuat->setObjectName(QString::fromUtf8("pushButton_TareQuat"));

        gridLayout_2->addWidget(pushButton_TareQuat, 3, 0, 1, 1);

        label_AngleDegree = new QLabel(groupBox);
        label_AngleDegree->setObjectName(QString::fromUtf8("label_AngleDegree"));

        gridLayout_2->addWidget(label_AngleDegree, 2, 1, 1, 1);


        gridLayout->addWidget(groupBox, 2, 0, 1, 1);

        lineEdit_path = new QLineEdit(YEIWidget);
        lineEdit_path->setObjectName(QString::fromUtf8("lineEdit_path"));
        lineEdit_path->setEnabled(false);

        gridLayout->addWidget(lineEdit_path, 4, 0, 1, 1);


        retranslateUi(YEIWidget);
        QObject::connect(pushButton_Open, SIGNAL(clicked()), YEIWidget, SLOT(openConnection()));
        QObject::connect(pushButton_Close, SIGNAL(clicked()), YEIWidget, SLOT(closeConnection()));
        QObject::connect(pushButton_Quaternion, SIGNAL(clicked()), YEIWidget, SLOT(getQuaternion()));

        QMetaObject::connectSlotsByName(YEIWidget);
    } // setupUi

    void retranslateUi(QWidget *YEIWidget)
    {
        YEIWidget->setWindowTitle(QApplication::translate("YEIWidget", "Form", 0, QApplication::UnicodeUTF8));
        checkBox->setText(QApplication::translate("YEIWidget", "Log", 0, QApplication::UnicodeUTF8));
        pushButton_Open->setText(QApplication::translate("YEIWidget", "Open", 0, QApplication::UnicodeUTF8));
        pushButton_Close->setText(QApplication::translate("YEIWidget", "Close", 0, QApplication::UnicodeUTF8));
        groupBox->setTitle(QApplication::translate("YEIWidget", "Measurements", 0, QApplication::UnicodeUTF8));
        pushButton_Quaternion->setText(QApplication::translate("YEIWidget", "Get Quat", 0, QApplication::UnicodeUTF8));
        label_Quaternion->setText(QApplication::translate("YEIWidget", "Quaternion", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("YEIWidget", "Angle", 0, QApplication::UnicodeUTF8));
        pushButton_TareQuat->setText(QApplication::translate("YEIWidget", "Tare Quaternion", 0, QApplication::UnicodeUTF8));
        label_AngleDegree->setText(QApplication::translate("YEIWidget", "TextLabel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class YEIWidget: public Ui_YEIWidget {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_YEI_WIDGET_H
