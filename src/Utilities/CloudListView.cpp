#include "CloudListView.h"
#include "ui_CloudListView.h"

CloudListView::CloudListView(QWidget *parent) :
    QTreeView(parent),
    ui(new Ui::CloudListView)
{
    ui->setupUi(this);
}

CloudListView::~CloudListView()
{
    delete ui;
}






