/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGridLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QGridLayout *gridLayout;
    QSplitter *splitter_2;
    QGroupBox *groupBox_3;
    QGridLayout *gridLayout_2;
    QPushButton *pushButton_openall;
    QPushButton *pushButton_closeall;
    QPushButton *pushButton_graball;
    QLabel *label_ClientState;
    QLabel *label_IP;
    QSplitter *splitter;
    QMenuBar *menuBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(628, 603);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        centralWidget->setMinimumSize(QSize(0, 0));
        gridLayout = new QGridLayout(centralWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        splitter_2 = new QSplitter(centralWidget);
        splitter_2->setObjectName(QString::fromUtf8("splitter_2"));
        splitter_2->setOrientation(Qt::Vertical);
        groupBox_3 = new QGroupBox(splitter_2);
        groupBox_3->setObjectName(QString::fromUtf8("groupBox_3"));
        gridLayout_2 = new QGridLayout(groupBox_3);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        pushButton_openall = new QPushButton(groupBox_3);
        pushButton_openall->setObjectName(QString::fromUtf8("pushButton_openall"));

        gridLayout_2->addWidget(pushButton_openall, 0, 0, 1, 1);

        pushButton_closeall = new QPushButton(groupBox_3);
        pushButton_closeall->setObjectName(QString::fromUtf8("pushButton_closeall"));

        gridLayout_2->addWidget(pushButton_closeall, 0, 1, 1, 1);

        pushButton_graball = new QPushButton(groupBox_3);
        pushButton_graball->setObjectName(QString::fromUtf8("pushButton_graball"));

        gridLayout_2->addWidget(pushButton_graball, 1, 0, 1, 1);

        label_ClientState = new QLabel(groupBox_3);
        label_ClientState->setObjectName(QString::fromUtf8("label_ClientState"));

        gridLayout_2->addWidget(label_ClientState, 1, 1, 1, 1);

        label_IP = new QLabel(groupBox_3);
        label_IP->setObjectName(QString::fromUtf8("label_IP"));

        gridLayout_2->addWidget(label_IP, 2, 0, 1, 1);

        splitter_2->addWidget(groupBox_3);
        splitter = new QSplitter(splitter_2);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        splitter_2->addWidget(splitter);

        gridLayout->addWidget(splitter_2, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 628, 25));
        MainWindow->setMenuBar(menuBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "OdroidSfM", 0, QApplication::UnicodeUTF8));
        groupBox_3->setTitle(QApplication::translate("MainWindow", "Project", 0, QApplication::UnicodeUTF8));
        pushButton_openall->setText(QApplication::translate("MainWindow", "Open ALL", 0, QApplication::UnicodeUTF8));
        pushButton_closeall->setText(QApplication::translate("MainWindow", "Close ALL", 0, QApplication::UnicodeUTF8));
        pushButton_graball->setText(QApplication::translate("MainWindow", "Grab ALL", 0, QApplication::UnicodeUTF8));
        label_ClientState->setText(QApplication::translate("MainWindow", "Client : ?", 0, QApplication::UnicodeUTF8));
        label_IP->setText(QApplication::translate("MainWindow", "myIPlabel", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
