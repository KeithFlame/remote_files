#include "contentWindow.h"
#include <QDateTime>
#include <QFont>
//#include <DbgHelp.h>
//#pragma comment (lib,"dbghelp.lib")
////创建dump文件
//inline void CreateMiniDump(PEXCEPTION_POINTERS pep, LPCTSTR strFileName)
//{
//	HANDLE hFile = CreateFile(strFileName, GENERIC_READ | GENERIC_WRITE,
//		FILE_SHARE_WRITE, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
//
//	if ((hFile != NULL) && (hFile != INVALID_HANDLE_VALUE))
//	{
//		MINIDUMP_EXCEPTION_INFORMATION mdei;
//		mdei.ThreadId = GetCurrentThreadId();
//		mdei.ExceptionPointers = pep;
//		mdei.ClientPointers = NULL;
//
//		MINIDUMP_CALLBACK_INFORMATION mci;
//		mci.CallbackRoutine = (MINIDUMP_CALLBACK_ROUTINE)MiniDumpCallback;
//		mci.CallbackParam = 0;
//
//		::MiniDumpWriteDump(::GetCurrentProcess(), ::GetCurrentProcessId(), hFile, MiniDumpNormal, (pep != 0) ? &mdei : 0, NULL, &mci);
//
//		CloseHandle(hFile);
//	}
//}
//// 当出现崩溃时，会创建core.dmp 崩溃文件
//LONG __stdcall MyUnhandledExceptionFilter(PEXCEPTION_POINTERS pExceptionInfo)
//{
//	CreateMiniDump(pExceptionInfo, LPCTSTR("core.dmp"));
//
//	return EXCEPTION_EXECUTE_HANDLER;
//}

#pragma execution_character_set("utf-8")

contentWindow* contentWindow::keith_content_ui = NULL;
contentWindow::contentWindow(QWidget *parent)
	: QMainWindow(parent)
{
	h_mutex = CreateMutex(NULL, FALSE, NULL);

	ui.setupUi(this);
	keith_content_ui = this;

	cs = new ContinuumShow(ui.tab_6);
	ui.down_pushbutton_2;
	cs->setObjectName(QStringLiteral("con_show"));
	cs->setGeometry(QRect(20, 20, 960, 540));
	mm = new MouseMovement(ui.tab_6);
	mm->setObjectName(QStringLiteral("mm"));
	mm->setGeometry(QRect(20, 20, 960, 540));

	QFont ft1;
	ft1.setPointSize(14);


	base_pose = new QLabel(ui.tab_6);
	base_pose->setObjectName(QStringLiteral("cs"));
	base_pose->setGeometry(QRect(980, 50, 550, 200));
	base_pose->setFont(ft1);
	//ki = new KeyInput(ui.tab_6);
	//ki->setObjectName(QStringLiteral("ki"));
	//ki->setGeometry(QRect(990, 20, 260, 240));
	//
	wait_refresh_flash_timer = new QTimer(this);
	
	connect(wait_refresh_flash_timer, SIGNAL(timeout()), this, SLOT(refreshFlashTimerSlots()));
	connect(this, SIGNAL(showInformationSig(QString)), this, SLOT(showInformationSlot(QString)));
	connect(ui.right_pushbutton_2, SIGNAL(clicked()), this, SLOT(testSlots()));
	connect(ui.clear_log, SIGNAL(released()), this, SLOT(clearLogSlot()));
	startRefreshFlash();

}

contentWindow::~contentWindow()
{

}

void contentWindow::startRefreshFlash()
{
	wait_refresh_flash_timer->start(10);
}

void contentWindow::refreshFlashTimerSlots()
{
	
	cs->is_range = mm->is_range;
	cs->left_clicked = mm->left_clicked;
	if (cs->left_clicked)
	{
		cs->mos_x = mm->mos_x;
		cs->mos_y = mm->mos_y;
	}
	else
	{
		cs->mos_x = 0.f;
		cs->mos_y = 0.f;
	}
	cs->right_clicked = mm->right_clicked;
	cs->is_reset = cs->right_clicked;
	setBasePoseShow();
}


void contentWindow::showInformationSlot(QString string)
{
	WaitForSingleObject(h_mutex, INFINITE);
	ui.textBrowser1->append(string);
	ReleaseMutex(h_mutex);
}

void contentWindow::showInfomation(QString str)
{
	QString string_temp;
	QDateTime current_time = QDateTime::currentDateTime();
	string_temp = current_time.toString("MM.dd hh:mm:ss.zzz: ") + str;
	emit keith_content_ui->showInformationSig(string_temp);
}

void contentWindow::testSlots()
{
	QString str = "adadasdAsdasd阿松大";
	showInfomation(str);
}

void contentWindow::clearLogSlot()
{
	ui.textBrowser1->clear();
}

void contentWindow::keyPressEvent(QKeyEvent* ev)
{
	int symbol_flag = 0;
	if (ev->key() == Qt::Key_W) 
		symbol_flag = 1;
	if (ev->key() == Qt::Key_S) 
		symbol_flag = 2;
	if (ev->key() == Qt::Key_A)
		symbol_flag = 3;
	if (ev->key() == Qt::Key_D)
		symbol_flag = 4;
	if (ev->key() == Qt::Key_Q)
		symbol_flag = 5;
	if (ev->key() == Qt::Key_E)
		symbol_flag = 6;

	switch (symbol_flag)
	{
	case 1: { cs->w_clicked = true; cs->a_clicked = false; cs->s_clicked = false; cs->d_clicked = false;
		cs->q_clicked = false; cs->e_clicked = false; return; }
	case 2: { cs->w_clicked = false; cs->a_clicked = false; cs->s_clicked = true; cs->d_clicked = false;
		cs->q_clicked = false; cs->e_clicked = false; return; }
	case 3: { cs->w_clicked = false; cs->a_clicked = true; cs->s_clicked = false; cs->d_clicked = false;
		cs->q_clicked = false; cs->e_clicked = false; return; }
	case 4: { cs->w_clicked = false; cs->a_clicked = false; cs->s_clicked = false; cs->d_clicked = true;
		cs->q_clicked = false; cs->e_clicked = false; return; }
	case 5: { cs->w_clicked = false; cs->a_clicked = false; cs->s_clicked = false; cs->d_clicked = false;
		cs->q_clicked = true; cs->e_clicked = false; return; }
	case 6: { cs->w_clicked = false; cs->a_clicked = false; cs->s_clicked = false; cs->d_clicked = false;
		cs->q_clicked = false; cs->e_clicked = true; return; }
	default: { cs->w_clicked = false; cs->a_clicked = false; cs->s_clicked = false; cs->d_clicked = false; }
	}
	QWidget::keyPressEvent(ev);

}

void contentWindow::keyReleaseEvent(QKeyEvent* ev)
{

}

void contentWindow::setBasePoseShow()
{
	QString str = "P指位置，R指姿态(用欧拉角XYZ表示)\nP_base = [" + QString::number(cs->trans_x) + ", " + QString::number(cs->trans_y) + ", "
		+ QString::number(cs->trans_z) + "]. R_base = [" + QString::number(cs->rot_angle_x) + ", " + QString::number(cs->rot_angle_y) + ", "
		+ QString::number(0.00f) + "]. \n";
	base_pose->setWordWrap(true);
	base_pose->setAlignment(Qt::AlignHCenter);
	//base_pose->setAlignment(Qt::AlignTop);
	base_pose->setText(str);
}