/****************************************************************************
** Meta object code from reading C++ file 'question_added.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.11.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../question_added.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'question_added.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.11.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_question_added_t {
    QByteArrayData data[7];
    char stringdata0[97];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_question_added_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_question_added_t qt_meta_stringdata_question_added = {
    {
QT_MOC_LITERAL(0, 0, 14), // "question_added"
QT_MOC_LITERAL(1, 15, 10), // "sendUpdate"
QT_MOC_LITERAL(2, 26, 0), // ""
QT_MOC_LITERAL(3, 27, 16), // "pushConfirmSlots"
QT_MOC_LITERAL(4, 44, 13), // "pushQuitSlots"
QT_MOC_LITERAL(5, 58, 14), // "openFilesSlots"
QT_MOC_LITERAL(6, 73, 23) // "printPartialScreenSlots"

    },
    "question_added\0sendUpdate\0\0pushConfirmSlots\0"
    "pushQuitSlots\0openFilesSlots\0"
    "printPartialScreenSlots"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_question_added[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       3,    0,   40,    2, 0x08 /* Private */,
       4,    0,   41,    2, 0x08 /* Private */,
       5,    0,   42,    2, 0x08 /* Private */,
       6,    0,   43,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void question_added::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        question_added *_t = static_cast<question_added *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->sendUpdate(); break;
        case 1: _t->pushConfirmSlots(); break;
        case 2: _t->pushQuitSlots(); break;
        case 3: _t->openFilesSlots(); break;
        case 4: _t->printPartialScreenSlots(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        {
            using _t = void (question_added::*)();
            if (*reinterpret_cast<_t *>(_a[1]) == static_cast<_t>(&question_added::sendUpdate)) {
                *result = 0;
                return;
            }
        }
    }
    Q_UNUSED(_a);
}

QT_INIT_METAOBJECT const QMetaObject question_added::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_question_added.data,
      qt_meta_data_question_added,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *question_added::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *question_added::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_question_added.stringdata0))
        return static_cast<void*>(this);
    return QWidget::qt_metacast(_clname);
}

int question_added::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}

// SIGNAL 0
void question_added::sendUpdate()
{
    QMetaObject::activate(this, &staticMetaObject, 0, nullptr);
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE
