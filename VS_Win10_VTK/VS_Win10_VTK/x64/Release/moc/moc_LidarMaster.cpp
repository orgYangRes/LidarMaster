/****************************************************************************
** Meta object code from reading C++ file 'LidarMaster.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.14.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include <memory>
#include "../../../LidarMaster.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'LidarMaster.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.14.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_LidarMaster_t {
    QByteArrayData data[27];
    char stringdata0[327];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_LidarMaster_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_LidarMaster_t qt_meta_stringdata_LidarMaster = {
    {
QT_MOC_LITERAL(0, 0, 11), // "LidarMaster"
QT_MOC_LITERAL(1, 12, 14), // "sendRenderAxis"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 8), // "QString&"
QT_MOC_LITERAL(4, 37, 7), // "strAxis"
QT_MOC_LITERAL(5, 45, 13), // "sendColorInfo"
QT_MOC_LITERAL(6, 59, 7), // "QColor&"
QT_MOC_LITERAL(7, 67, 5), // "color"
QT_MOC_LITERAL(8, 73, 23), // "closeFilterDialogSignal"
QT_MOC_LITERAL(9, 97, 27), // "closeGridFilterDialogSignal"
QT_MOC_LITERAL(10, 125, 9), // "isLasInfo"
QT_MOC_LITERAL(11, 135, 9), // "isProInfo"
QT_MOC_LITERAL(12, 145, 11), // "isOtherInfo"
QT_MOC_LITERAL(13, 157, 19), // "treeItemClickedSlot"
QT_MOC_LITERAL(14, 177, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(15, 194, 4), // "item"
QT_MOC_LITERAL(16, 199, 3), // "col"
QT_MOC_LITERAL(17, 203, 16), // "recvRenderCoords"
QT_MOC_LITERAL(18, 220, 16), // "recColorInfoSlot"
QT_MOC_LITERAL(19, 237, 13), // "recvFilterVal"
QT_MOC_LITERAL(20, 251, 4), // "type"
QT_MOC_LITERAL(21, 256, 9), // "filterVal"
QT_MOC_LITERAL(22, 266, 7), // "lasFile"
QT_MOC_LITERAL(23, 274, 13), // "rectoLeftSlot"
QT_MOC_LITERAL(24, 288, 14), // "rectoRightSlot"
QT_MOC_LITERAL(25, 303, 15), // "recvGridAndType"
QT_MOC_LITERAL(26, 319, 7) // "gridVal"

    },
    "LidarMaster\0sendRenderAxis\0\0QString&\0"
    "strAxis\0sendColorInfo\0QColor&\0color\0"
    "closeFilterDialogSignal\0"
    "closeGridFilterDialogSignal\0isLasInfo\0"
    "isProInfo\0isOtherInfo\0treeItemClickedSlot\0"
    "QTreeWidgetItem*\0item\0col\0recvRenderCoords\0"
    "recColorInfoSlot\0recvFilterVal\0type\0"
    "filterVal\0lasFile\0rectoLeftSlot\0"
    "rectoRightSlot\0recvGridAndType\0gridVal"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_LidarMaster[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
      14,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   84,    2, 0x06 /* Public */,
       5,    1,   87,    2, 0x06 /* Public */,
       8,    0,   90,    2, 0x06 /* Public */,
       9,    0,   91,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      10,    0,   92,    2, 0x08 /* Private */,
      11,    0,   93,    2, 0x08 /* Private */,
      12,    0,   94,    2, 0x08 /* Private */,
      13,    2,   95,    2, 0x08 /* Private */,
      17,    1,  100,    2, 0x08 /* Private */,
      18,    1,  103,    2, 0x08 /* Private */,
      19,    3,  106,    2, 0x08 /* Private */,
      23,    0,  113,    2, 0x08 /* Private */,
      24,    0,  114,    2, 0x08 /* Private */,
      25,    2,  115,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void,
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 14, QMetaType::Int,   15,   16,
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void, 0x80000000 | 6,    7,
    QMetaType::Void, QMetaType::Int, QMetaType::Double, 0x80000000 | 3,   20,   21,   22,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int, QMetaType::Int,   26,   20,

       0        // eod
};

void LidarMaster::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<LidarMaster *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendRenderAxis((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->sendColorInfo((*reinterpret_cast< QColor(*)>(_a[1]))); break;
        case 2: _t->closeFilterDialogSignal(); break;
        case 3: _t->closeGridFilterDialogSignal(); break;
        case 4: _t->isLasInfo(); break;
        case 5: _t->isProInfo(); break;
        case 6: _t->isOtherInfo(); break;
        case 7: _t->treeItemClickedSlot((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 8: _t->recvRenderCoords((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 9: _t->recColorInfoSlot((*reinterpret_cast< QColor(*)>(_a[1]))); break;
        case 10: _t->recvFilterVal((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< QString(*)>(_a[3]))); break;
        case 11: _t->rectoLeftSlot(); break;
        case 12: _t->rectoRightSlot(); break;
        case 13: _t->recvGridAndType((*reinterpret_cast< int(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (LidarMaster::*)(QString & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&LidarMaster::sendRenderAxis)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (LidarMaster::*)(QColor & );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&LidarMaster::sendColorInfo)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (LidarMaster::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&LidarMaster::closeFilterDialogSignal)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (LidarMaster::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&LidarMaster::closeGridFilterDialogSignal)) {
                *result = 3;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject LidarMaster::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_meta_stringdata_LidarMaster.data,
    qt_meta_data_LidarMaster,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *LidarMaster::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *LidarMaster::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_LidarMaster.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int LidarMaster::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 14)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 14;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 14)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 14;
    }
    return _id;
}

// SIGNAL 0
void LidarMaster::sendRenderAxis(QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void LidarMaster::sendColorInfo(QColor & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void LidarMaster::closeFilterDialogSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 2, nullptr);
}

// SIGNAL 3
void LidarMaster::closeGridFilterDialogSignal()
{
    QMetaObject::activate(this, &staticMetaObject, 3, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
