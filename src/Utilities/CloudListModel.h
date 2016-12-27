#ifndef CloudList_H
#define CloudList_H

#include <QStandardItemModel>
#include <QWidget>
#include <Transform.h>
#include "define.h"


#define PROP_NUMBER 0
#define PROP_ORGANIZED 1
#define PROP_DENSE 2
#define PROP_OPACITY 3
#define PROP_SIZE 4
#define PROP_NORMAL 5
#define PROP_NBPROP 6

#define PROP_KEYPOINTS 19
#define PROP_TIME_STAMP 20
#define PROP_SEQ 21
#define PROP_WIDTH 22
#define PROP_HEIGHT 23



class CloudListModel : public QStandardItemModel
{
    Q_OBJECT
public:
    CloudListModel(QObject *parent = 0);


    int rowCount(const QModelIndex &parent = QModelIndex()) const;
    int columnCount() const  { return 2; }

    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const;
    QVariant headerData(int section, Qt::Orientation orientation, int role) const;

    bool setData(const QModelIndex & index, const QVariant & value, int role = Qt::EditRole);
    Qt::ItemFlags flags(const QModelIndex & index) const ;

public slots:
    void addCloud(PointCloudT::Ptr PC);

private:
    QList<PointCloudT::Ptr> _PCList;
};
#endif // CloudList_H
