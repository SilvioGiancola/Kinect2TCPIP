/****************************************************************************
** Meta object code from reading C++ file 'pololu_controller_widget.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.6)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../pololu_controller_widget.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pololu_controller_widget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.6. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_PololuControllerWidget[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      28,   23,   24,   23, 0x0a,
      45,   23,   23,   23, 0x0a,
      63,   23,   23,   23, 0x0a,
      81,   23,   23,   23, 0x0a,
      99,   23,   23,   23, 0x0a,
     117,   23,   23,   23, 0x0a,
     135,   23,   23,   23, 0x0a,
     153,   23,   23,   23, 0x0a,
     177,  171,   23,   23, 0x08,
     213,  171,   23,   23, 0x08,
     249,  171,   23,   23, 0x08,
     285,  171,   23,   23, 0x08,
     321,  171,   23,   23, 0x08,
     357,  171,   23,   23, 0x08,

       0        // eod
};

static const char qt_meta_stringdata_PololuControllerWidget[] = {
    "PololuControllerWidget\0\0int\0"
    "openConnection()\0closeConnection()\0"
    "impulseChannel0()\0impulseChannel1()\0"
    "impulseChannel2()\0impulseChannel3()\0"
    "impulseChannel4()\0impulseChannel5()\0"
    "value\0on_spinBox_Servo0_valueChanged(int)\0"
    "on_spinBox_Servo1_valueChanged(int)\0"
    "on_spinBox_Servo2_valueChanged(int)\0"
    "on_spinBox_Servo3_valueChanged(int)\0"
    "on_spinBox_Servo4_valueChanged(int)\0"
    "on_spinBox_Servo5_valueChanged(int)\0"
};

void PololuControllerWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        PololuControllerWidget *_t = static_cast<PololuControllerWidget *>(_o);
        switch (_id) {
        case 0: { int _r = _t->openConnection();
            if (_a[0]) *reinterpret_cast< int*>(_a[0]) = _r; }  break;
        case 1: _t->closeConnection(); break;
        case 2: _t->impulseChannel0(); break;
        case 3: _t->impulseChannel1(); break;
        case 4: _t->impulseChannel2(); break;
        case 5: _t->impulseChannel3(); break;
        case 6: _t->impulseChannel4(); break;
        case 7: _t->impulseChannel5(); break;
        case 8: _t->on_spinBox_Servo0_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->on_spinBox_Servo1_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->on_spinBox_Servo2_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->on_spinBox_Servo3_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->on_spinBox_Servo4_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->on_spinBox_Servo5_valueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData PololuControllerWidget::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject PololuControllerWidget::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_PololuControllerWidget,
      qt_meta_data_PololuControllerWidget, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &PololuControllerWidget::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *PololuControllerWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *PololuControllerWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_PololuControllerWidget))
        return static_cast<void*>(const_cast< PololuControllerWidget*>(this));
    return QWidget::qt_metacast(_clname);
}

int PololuControllerWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
