#ifndef KinectWidget_H
#define KinectWidget_H

#include <QWidget>
#include "define.h"

namespace Ui {
class KinectWidget;
}

class KinectWidget : public QWidget
{
    Q_OBJECT

public:
    explicit KinectWidget(QWidget *parent = 0);
    ~KinectWidget();

private:
    Ui::KinectWidget *ui;
};

#endif // KinectWidget_H
