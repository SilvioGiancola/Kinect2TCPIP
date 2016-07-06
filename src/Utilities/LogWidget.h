#ifndef LogWidget_H_
#define LogWidget_H_

#include <QWidget>
#include <QDateTime>
#include "define.h"


namespace Ui {
class LogWidget;
}

class LogWidget : public QWidget
{
    Q_OBJECT

public:
    LogWidget(QWidget * parent = 0);
    virtual ~LogWidget();


public slots:
    void appendText(QString);

private:
    Ui::LogWidget *ui;

};

#endif /* LogWidget_H_ */
