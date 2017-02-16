#include "IDlist.h"
#include "ui_IDlist.h"

IDlist::IDlist(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::IDlist)
{
    ui->setupUi(this);

   // addItems();

    this->addItem("ID 0");
    this->addItem("ID 1");
    this->addItem("ID 2");
    this->addItem("ID 3");
}

IDlist::~IDlist()
{
    delete ui;
}


QStringList IDlist::getList()
{
    QStringList str_list;
    for (int i = 0; i < this->count(); i++)
        str_list.append(this->item(i)->text());

    return str_list;
}


