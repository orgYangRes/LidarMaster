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
    QByteArrayData data[14];
    char stringdata0[153];
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
QT_MOC_LITERAL(5, 45, 9), // "isLasInfo"
QT_MOC_LITERAL(6, 55, 9), // "isProInfo"
QT_MOC_LITERAL(7, 65, 11), // "isOtherInfo"
QT_MOC_LITERAL(8, 77, 19), // "treeItemClickedSlot"
QT_MOC_LITERAL(9, 97, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(10, 114, 4), // "item"
QT_MOC_LITERAL(11, 119, 3), // "col"
QT_MOC_LITERAL(12, 123, 20), // "recPtCloudRenderSlot"
QT_MOC_LITERAL(13, 144, 8) // "strCoord"

    },
    "LidarMaster\0sendRenderAxis\0\0QString&\0"
    "strAxis\0isLasInfo\0isProInfo\0isOtherInfo\0"
    "treeItemClickedSlot\0QTreeWidgetItem*\0"
    "item\0col\0recPtCloudRenderSlot\0strCoord"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_LidarMaster[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   44,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       5,    0,   47,    2, 0x08 /* Private */,
       6,    0,   48,    2, 0x08 /* Private */,
       7,    0,   49,    2, 0x08 /* Private */,
       8,    2,   50,    2, 0x08 /* Private */,
      12,    1,   55,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    4,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9, QMetaType::Int,   10,   11,
    QMetaType::Void, 0x80000000 | 3,   13,

       0        // eod
};

void LidarMaster::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<LidarMaster *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendRenderAxis((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->isLasInfo(); break;
        case 2: _t->isProInfo(); break;
        case 3: _t->isOtherInfo(); break;
        case 4: _t->treeItemClickedSlot((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 5: _t->recPtCloudRenderSlot((*reinterpret_cast< QString(*)>(_a[1]))); break;
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
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}

// SIGNAL 0
void LidarMaster::sendRenderAxis(QString & _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(std::addressof(_t1))) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
