/****************************************************************************
** Meta object code from reading C++ file 'yei_widget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../yei_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'yei_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_YEIWidget[] = {

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
      15,   10,   11,   10, 0x0a,
      32,   10,   10,   10, 0x0a,
      69,   10,   50,   10, 0x0a,
      85,   10,   10,   10, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_YEIWidget[] = {
    "YEIWidget\0\0int\0openConnection()\0"
    "closeConnection()\0Eigen::Quaternionf\0"
    "getQuaternion()\0on_pushButton_TareQuat_clicked()\0"
};

void YEIWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        YEIWidget *_t = static_cast<YEIWidget *>(_o);
        switch (_id) {
        case 0: { int _r = _t->openConnection();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 1: _t->closeConnection(); break;
        case 2: { Eigen::Quaternionf _r = _t->getQuaternion();
            if (_a[0]) *reinterpret_cast< Eigen::Quaternionf*>(_a[0]) = _r; }  break;
        case 3: _t->on_pushButton_TareQuat_clicked(); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData YEIWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject YEIWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_YEIWidget,
      qt_meta_data_YEIWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &YEIWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *YEIWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *YEIWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_YEIWidget))
        return static_cast<void*>(const_cast< YEIWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int YEIWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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
