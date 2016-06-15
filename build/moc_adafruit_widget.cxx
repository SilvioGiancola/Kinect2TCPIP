/****************************************************************************
** Meta object code from reading C++ file 'adafruit_widget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../adafruit_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'adafruit_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_AdafruitWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      16,   15,   15,   15, 0x0a,
      33,   15,   15,   15, 0x0a,
      51,   15,   15,   15, 0x0a,
      89,   15,   70,   15, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_AdafruitWidget[] = {
    "AdafruitWidget\0\0openConnection()\0"
    "closeConnection()\0initializeSensor()\0"
    "Eigen::Quaternionf\0getQuaternion()\0"
};

void AdafruitWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        AdafruitWidget *_t = static_cast<AdafruitWidget *>(_o);
        switch (_id) {
        case 0: _t->openConnection(); break;
        case 1: _t->closeConnection(); break;
        case 2: _t->initializeSensor(); break;
        case 3: { Eigen::Quaternionf _r = _t->getQuaternion();
            if (_a[0]) *reinterpret_cast< Eigen::Quaternionf*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObjectExtraData AdafruitWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject AdafruitWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_AdafruitWidget,
      qt_meta_data_AdafruitWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &AdafruitWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *AdafruitWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *AdafruitWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_AdafruitWidget))
        return static_cast<void*>(const_cast< AdafruitWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int AdafruitWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
