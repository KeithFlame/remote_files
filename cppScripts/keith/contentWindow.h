#pragma once
#include <QMainWindow>
#include "ui_contentWindow.h"
#include "instrument.h"
#include "continuum_show.h"
#include "mouse_movement.h"
#include <QKeyEvent>
#include <QTimer>
#include "windows.h"


class contentWindow : public QMainWindow
{
	Q_OBJECT

public:
	contentWindow(QWidget *parent = nullptr);
	~contentWindow();
private:
	static void showInfomation(QString);
	void setBasePoseShow();

private slots:
	void refreshFlashTimerSlots();
	void startRefreshFlash();
	void showInformationSlot(QString);
	void clearLogSlot();

	void testSlots();
private:
	QTimer* wait_refresh_flash_timer;
	QLabel* base_pose;
	Ui::contentWindowClass ui;
	Instrument tool;
	ContinuumShow* cs;
	MouseMovement* mm;
	//KeyInput* ki;
	//TiXmlDocument lconfigXML;
	HANDLE h_mutex;
	static contentWindow* keith_content_ui;
signals:
	void showInformationSig(QString);


protected:
	virtual void keyPressEvent(QKeyEvent* ev);
	virtual void keyReleaseEvent(QKeyEvent* ev);
};
