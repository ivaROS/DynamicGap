/****************************************************************************
** Meta object code from reading C++ file 'waitingqueue.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.12.8)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../../../src/arena/utils/pedsim_ros/pedsim_engine/pedsim_simulator/include/pedsim_simulator/element/waitingqueue.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'waitingqueue.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.12.8. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_WaitingQueue_t {
    QByteArrayData data[18];
    char stringdata0[203];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_WaitingQueue_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_WaitingQueue_t qt_meta_stringdata_WaitingQueue = {
    {
QT_MOC_LITERAL(0, 0, 12), // "WaitingQueue"
QT_MOC_LITERAL(1, 13, 16), // "directionChanged"
QT_MOC_LITERAL(2, 30, 0), // ""
QT_MOC_LITERAL(3, 31, 11), // "radianAngle"
QT_MOC_LITERAL(4, 43, 12), // "agentMayPass"
QT_MOC_LITERAL(5, 56, 10), // "pedsim::id"
QT_MOC_LITERAL(6, 67, 2), // "id"
QT_MOC_LITERAL(7, 70, 13), // "agentDequeued"
QT_MOC_LITERAL(8, 84, 18), // "queueLeaderChanged"
QT_MOC_LITERAL(9, 103, 15), // "queueEndChanged"
QT_MOC_LITERAL(10, 119, 23), // "queueEndPositionChanged"
QT_MOC_LITERAL(11, 143, 1), // "x"
QT_MOC_LITERAL(12, 145, 1), // "y"
QT_MOC_LITERAL(13, 147, 13), // "onTimeChanged"
QT_MOC_LITERAL(14, 161, 6), // "timeIn"
QT_MOC_LITERAL(15, 168, 26), // "onLastAgentPositionChanged"
QT_MOC_LITERAL(16, 195, 3), // "xIn"
QT_MOC_LITERAL(17, 199, 3) // "yIn"

    },
    "WaitingQueue\0directionChanged\0\0"
    "radianAngle\0agentMayPass\0pedsim::id\0"
    "id\0agentDequeued\0queueLeaderChanged\0"
    "queueEndChanged\0queueEndPositionChanged\0"
    "x\0y\0onTimeChanged\0timeIn\0"
    "onLastAgentPositionChanged\0xIn\0yIn"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_WaitingQueue[] = {

 // content:
       8,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       6,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   54,    2, 0x06 /* Public */,
       4,    1,   57,    2, 0x06 /* Public */,
       7,    1,   60,    2, 0x06 /* Public */,
       8,    1,   63,    2, 0x06 /* Public */,
       9,    0,   66,    2, 0x06 /* Public */,
      10,    2,   67,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
      13,    1,   72,    2, 0x09 /* Protected */,
      15,    2,   75,    2, 0x09 /* Protected */,

 // signals: parameters
    QMetaType::Void, QMetaType::Double,    3,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   11,   12,

 // slots: parameters
    QMetaType::Void, QMetaType::Double,   14,
    QMetaType::Void, QMetaType::Double, QMetaType::Double,   16,   17,

       0        // eod
};

void WaitingQueue::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        auto *_t = static_cast<WaitingQueue *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->directionChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 1: _t->agentMayPass((*reinterpret_cast< pedsim::id(*)>(_a[1]))); break;
        case 2: _t->agentDequeued((*reinterpret_cast< pedsim::id(*)>(_a[1]))); break;
        case 3: _t->queueLeaderChanged((*reinterpret_cast< pedsim::id(*)>(_a[1]))); break;
        case 4: _t->queueEndChanged(); break;
        case 5: _t->queueEndPositionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        case 6: _t->onTimeChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 7: _t->onLastAgentPositionChanged((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (WaitingQueue::*)(double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::directionChanged)) {
                *result = 0;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)(pedsim::id );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::agentMayPass)) {
                *result = 1;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)(pedsim::id );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::agentDequeued)) {
                *result = 2;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)(pedsim::id );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::queueLeaderChanged)) {
                *result = 3;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::queueEndChanged)) {
                *result = 4;
                return;
            }
        }
        {
            using _t = void (WaitingQueue::*)(double , double );
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&WaitingQueue::queueEndPositionChanged)) {
                *result = 5;
                return;
            }
        }
    }
}

QT_INIT_METAOBJECT const QMetaObject WaitingQueue::staticMetaObject = { {
    &Waypoint::staticMetaObject,
    qt_meta_stringdata_WaitingQueue.data,
    qt_meta_data_WaitingQueue,
    qt_static_metacall,
    nullptr,
    nullptr
} };


const QMetaObject *WaitingQueue::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *WaitingQueue::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_WaitingQueue.stringdata0))
        return static_cast<void*>(this);
    return Waypoint::qt_metacast(_clname);
}

int WaitingQueue::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = Waypoint::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}

// SIGNAL 0
void WaitingQueue::directionChanged(double _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void WaitingQueue::agentMayPass(pedsim::id _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}

// SIGNAL 2
void WaitingQueue::agentDequeued(pedsim::id _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 2, _a);
}

// SIGNAL 3
void WaitingQueue::queueLeaderChanged(pedsim::id _t1)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 3, _a);
}

// SIGNAL 4
void WaitingQueue::queueEndChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 4, nullptr);
}

// SIGNAL 5
void WaitingQueue::queueEndPositionChanged(double _t1, double _t2)
{
    void *_a[] = { nullptr, const_cast<void*>(reinterpret_cast<const void*>(&_t1)), const_cast<void*>(reinterpret_cast<const void*>(&_t2)) };
    QMetaObject::activate(this, &staticMetaObject, 5, _a);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
