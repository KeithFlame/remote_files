#pragma once
#ifndef ADDING_SOLUTION_H_
#define ADDING_SOLUTION_H_

#include <QtWidgets/QWidget>
#include <QtWidgets/QPushButton>

class SubWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SubWidget(QWidget* parent = 0);
    ~SubWidget();

    void sendSlot();

signals:
    /**
     * �źű���Ҫsignals�ؼ���������
     * �ź�û�з���ֵ, �������в���
     * �źž��Ǻ���������, ���趨��
     * ʹ��: emit mySignal();
     * �źſ��Ա�����
     */
    void mySignal();
    void mySignalParm(int, QString);

public slots:

private:
    QPushButton* bt;
};


#endif // ADDING_SOLUTION_H_