#ifndef TransformationWidget_H
#define TransformationWidget_H

#include <QWidget>
#include <Transform.h>
#include "define.h"

namespace Ui {
class TransformationWidget;
}

class TransformationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TransformationWidget(QWidget *parent = 0);
    virtual ~TransformationWidget();

    Transform getTransform();
    void setTransform(Transform);

public slots:
    void emitTransform();

signals:
    void matrixchanged(Transform mat);

private slots:
  /*  void on_TranslationX_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_TranslationY_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_TranslationZ_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_RotationX_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_RotationY_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_RotationZ_valueChanged(double arg1){ emit matrixchanged(getTransform());}*/

    void on_pushButton_reset_clicked();


private:
    Ui::TransformationWidget *ui;
};

#endif // TransformationWidget_H
