#pragma once
#ifndef GENERATION_REPORT_H_
#define GENERATION_REPORT_H_

#include <QWidget>

class GenRep : public QWidget
{
    Q_OBJECT
        class subUI;
private:
    
    void setTipWindow(QString);
    QScopedPointer<subUI>    m_subUI;
    QString file_path;
    QString system_proc;
    QString system_serial;
    QString system_batch;
    QString tool_proc;
    QString tool_serial;
    QString tool_batch;
private slots:
    void CreatePdfAccuRepeatSlot();
    void CreatePdfSensitivitySlot();
    void CreatePdfPositionRepeatSlot();
    void CreatePdfMaxWorkspaceSlot();
    void CreatePdfDelaySlot();
    void setConfirmSlot();

public:
    
    explicit GenRep(QWidget* parent = 0);
    void setFilePath(QString);
    void setSystemProc(QString);
    void setSystemSerial(QString);
    void setSystemBatch(QString);
    void setToolProc(QString);
    void setToolSerial(QString);
    void setToolBatch(QString);
    ~GenRep();
};

#endif //GENERATION_REPORT_H_
