#ifndef TransformationWidget_H
#define TransformationWidget_H

#include <QWidget>
#include "define.h"

namespace Ui {
class TransformationWidget;
}

class TransformationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit TransformationWidget(QWidget *parent = 0);
    ~TransformationWidget();

    TransformT getTransform();
    void setTransform(TransformT);

signals:
    void matrixchanged(TransformT mat);

private slots:
    void on_TranslationX_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_TranslationY_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_TranslationZ_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_RotationX_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_RotationY_valueChanged(double arg1){ emit matrixchanged(getTransform());}
    void on_RotationZ_valueChanged(double arg1){ emit matrixchanged(getTransform());}

private:
    Ui::TransformationWidget *ui;
};

#endif // TransformationWidget_H
