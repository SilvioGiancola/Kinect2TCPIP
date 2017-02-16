#ifndef IDlist_H
#define IDlist_H

#include <QWidget>
#include <QStringList>
#include <QListWidget>
#include "define.h"

namespace Ui {
class IDlist;
}

class IDlist : public QListWidget
{
    Q_OBJECT

public:
    explicit IDlist(QWidget *parent = 0);
    virtual ~IDlist();


    QStringList getList();

public slots:
    void emitTransform();

signals:
    void matrixchanged(Transform mat);


private:
    Ui::IDlist *ui;

    int nRow = 4;
};

#endif // IDlist_H
