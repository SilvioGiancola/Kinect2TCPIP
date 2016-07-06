#include "LogWidget.h"
#include "ui_LogWidget.h"

LogWidget::LogWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::LogWidget)
{
    ui->setupUi(this);

}

LogWidget::~LogWidget()
{
}


void LogWidget::appendText(QString log)
{
    ui->plainTextEdit->appendPlainText(QString("[%1]: %2").arg(QDateTime::currentDateTime().toString(TIMEFORMAT)).arg(log));
}
