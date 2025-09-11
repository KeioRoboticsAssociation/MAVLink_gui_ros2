/****************************************************************************
** Meta object code from reading C++ file 'mavlink_gui_widget.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.15.13)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../include/mavlink_gui_tester/mavlink_gui_widget.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mavlink_gui_widget.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.15.13. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_mavlink_gui_tester__MAVLinkGUIWidget_t {
    QByteArrayData data[14];
    char stringdata0[255];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_mavlink_gui_tester__MAVLinkGUIWidget_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_mavlink_gui_tester__MAVLinkGUIWidget_t qt_meta_stringdata_mavlink_gui_tester__MAVLinkGUIWidget = {
    {
QT_MOC_LITERAL(0, 0, 36), // "mavlink_gui_tester::MAVLinkGU..."
QT_MOC_LITERAL(1, 37, 15), // "onConnectSerial"
QT_MOC_LITERAL(2, 53, 0), // ""
QT_MOC_LITERAL(3, 54, 18), // "onDisconnectSerial"
QT_MOC_LITERAL(4, 73, 15), // "onSendHeartbeat"
QT_MOC_LITERAL(5, 89, 19), // "onSendManualControl"
QT_MOC_LITERAL(6, 109, 17), // "onSendCommandLong"
QT_MOC_LITERAL(7, 127, 16), // "onSendRCChannels"
QT_MOC_LITERAL(8, 144, 17), // "onSendServoOutput"
QT_MOC_LITERAL(9, 162, 14), // "onSendAttitude"
QT_MOC_LITERAL(10, 177, 15), // "onSendCustomHex"
QT_MOC_LITERAL(11, 193, 15), // "onClearMessages"
QT_MOC_LITERAL(12, 209, 22), // "updateReceivedMessages"
QT_MOC_LITERAL(13, 232, 22) // "updateConnectionStatus"

    },
    "mavlink_gui_tester::MAVLinkGUIWidget\0"
    "onConnectSerial\0\0onDisconnectSerial\0"
    "onSendHeartbeat\0onSendManualControl\0"
    "onSendCommandLong\0onSendRCChannels\0"
    "onSendServoOutput\0onSendAttitude\0"
    "onSendCustomHex\0onClearMessages\0"
    "updateReceivedMessages\0updateConnectionStatus"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_mavlink_gui_tester__MAVLinkGUIWidget[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x08 /* Private */,
       3,    0,   75,    2, 0x08 /* Private */,
       4,    0,   76,    2, 0x08 /* Private */,
       5,    0,   77,    2, 0x08 /* Private */,
       6,    0,   78,    2, 0x08 /* Private */,
       7,    0,   79,    2, 0x08 /* Private */,
       8,    0,   80,    2, 0x08 /* Private */,
       9,    0,   81,    2, 0x08 /* Private */,
      10,    0,   82,    2, 0x08 /* Private */,
      11,    0,   83,    2, 0x08 /* Private */,
      12,    0,   84,    2, 0x08 /* Private */,
      13,    0,   85,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void mavlink_gui_tester::MAVLinkGUIWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<MAVLinkGUIWidget *>(_o);
        (void)_t;
        switch (_id) {
        case 0: _t->onConnectSerial(); break;
        case 1: _t->onDisconnectSerial(); break;
        case 2: _t->onSendHeartbeat(); break;
        case 3: _t->onSendManualControl(); break;
        case 4: _t->onSendCommandLong(); break;
        case 5: _t->onSendRCChannels(); break;
        case 6: _t->onSendServoOutput(); break;
        case 7: _t->onSendAttitude(); break;
        case 8: _t->onSendCustomHex(); break;
        case 9: _t->onClearMessages(); break;
        case 10: _t->updateReceivedMessages(); break;
        case 11: _t->updateConnectionStatus(); break;
        default: ;
        }
    }
    (void)_a;
}

QT_INIT_METAOBJECT const QMetaObject mavlink_gui_tester::MAVLinkGUIWidget::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_mavlink_gui_tester__MAVLinkGUIWidget.data,
    qt_meta_data_mavlink_gui_tester__MAVLinkGUIWidget,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *mavlink_gui_tester::MAVLinkGUIWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *mavlink_gui_tester::MAVLinkGUIWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_mavlink_gui_tester__MAVLinkGUIWidget.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int mavlink_gui_tester::MAVLinkGUIWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
