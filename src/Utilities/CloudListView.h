#ifndef CloudListView_H
#define CloudListView_H

#include <QWidget>
#include <Transform.h>
#include <CloudListModel.h>
#include "define.h"
#include <QTreeView>

namespace Ui {
class CloudListView;
}

class CloudListView : public QTreeView
{
    Q_OBJECT

public:
    explicit CloudListView(QWidget *parent = 0);
    virtual ~CloudListView();



  //  void setModel(CloudListModel *mymodel);



public slots:
  //  void addCloud(PointCloudT::Ptr PC);

private:
    Ui::CloudListView *ui;

};

#endif // CloudListView_H
