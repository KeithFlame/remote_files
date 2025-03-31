#include "DoM_pro.h"

#include <QLabel>
#include <QGroupBox>
#include <cmath>
#include <QDir>
#include <QDateTime>
#include <QFont>
#include <QMessageBox>
#include <iomanip>
#include <sstream>
#include <fstream>
//#include "sr_mbx.h"
#include "difference_function.h"
#include "auxiliary_function.h"

#define SHOW_LOG 1

#pragma execution_character_set("utf-8") //set encoding character
DoM_pm* Dom = NULL;


DoM_pm::DoM_pm(QWidget* parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	this->setFixedSize(1334, 750);
	createUI();

	// 变量初始化
	log_file_handle = nullptr;
	log_stream = nullptr;
	Dom = this;
	QThread* dm_Thread = new QThread(this);
	diff_method = new DifferenceMethod();
	diff_method->moveToThread(dm_Thread);
	dm_Thread->start();

	displayTimer = new QTimer(this);
	commandTimer = new QTimer(this);
	inverse2Timer = new QTimer(this);

	for (size_t i = 0; i < ARM_NUM; i++) {
		trocar[i] = new Trocar();
		tb[i] = Eigen::Matrix4d::Identity();
	}


	standard_tool = new SurgicalContinuumManipulator();
	Eigen::Vector2d MP; MP << 0.0, 0.0;
	Eigen::Vector7d SP; SP << 0.95, 0.4, 381.5, 99.2, 10.0, 19.4, 25.0;
	Eigen::Vector10d FP; FP << 2.5, 2.7, 0, -Keith::PI_4, 5.0, 0.6, 0.1, 0.0, 0.0, 0.0;
	CC_tool = new SurgicalContinuumManipulator(MP, SP, FP);

	Eigen::Vector27d guess0;
	guess0 << 0.0, 0.0, 0.0,										// 初始力
		0.0, 0.0, 0.0,												// 初始力矩
		0.000, 0.0, 0.0, 0.0,										// 丝的应力
		0.0, 0.0, 0.0, 0.0,											// 进给与旋转
		5e-3, 1e-2, 1e-2, 1e-2, 0.0, 0.0, 0.0,						// ksi
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0;								// ksi
	guess.resize(3);
	guess[0] = guess0;
	guess[1] = guess0;
	guess[2] = guess0;
	guess_tem = guess;
	Rsd_last = Eigen::Vector26d::Zero();
	cur_pose = Eigen::Matrix4d::Identity();
	cur_target = Eigen::Matrix4d::Identity();
	target = Eigen::Matrix4d::Identity();
	file_path = "";

	connect(this, SIGNAL(showInformationSig(QString)), this, SLOT(showInformationSlot(QString)), Qt::QueuedConnection);
	connect(control_button[0], SIGNAL(clicked()), this, SLOT(clickBtnOpenCameraslot()));
	connect(control_button[1], SIGNAL(clicked()), this, SLOT(confirmParamsSlot()));
	connect(control_button[2], SIGNAL(clicked()), this, SLOT(connectDevicesSlot()));
	connect(control_button[3], SIGNAL(clicked()), this, SLOT(singleStepSlot()));
	connect(control_button[4], SIGNAL(clicked()), this, SLOT(calcInverseProblems3Slot()));
	connect(control_button[5], SIGNAL(clicked()), this, SLOT(calcStiffnessObserver2Slot()));
	connect(control_button[6], SIGNAL(clicked()), this, SLOT(getCurrentPoseSlot()));
	//
	connect(control_button[7], SIGNAL(clicked()), this, SLOT(getTissueParamsSlot()));
	//
	connect(control_button[8], SIGNAL(clicked()), this, SLOT(PushButtonClearLogSlot()));

	connect(arm_psi_button[2][0], SIGNAL(clicked()), this, SLOT(stopInverseProblemsSlot()));
	connect(arm_psi_button[2][1], SIGNAL(clicked()), this, SLOT(multipleHitInverseProblems2Slot()));
	connect(inverse2Timer, SIGNAL(timeout()), this, SLOT(continueInverseProblemsSlot()));


	connect(arm_psi_button[0][0], SIGNAL(clicked()), this, SLOT(connectArm1Slot()));
	connect(arm_psi_button[0][1], SIGNAL(clicked()), this, SLOT(sendCommandToArm1Slot()));
	connect(arm_psi_button[1][0], SIGNAL(clicked()), this, SLOT(connectArm2Slot()));
	connect(arm_psi_button[1][1], SIGNAL(clicked()), this, SLOT(sendCommandToArm2Slot()));
	connect(arm_psi_button[3][0], SIGNAL(clicked()), this, SLOT(connectArm4Slot()));
	connect(arm_psi_button[3][1], SIGNAL(clicked()), this, SLOT(sendCommandToArm4Slot()));

	connect(displayTimer, SIGNAL(timeout()), this, SLOT(displayVideoSlot()));
	connect(commandTimer, SIGNAL(timeout()), this, SLOT(checkCommandSlot()));

}

DoM_pm::~DoM_pm()
{
	if (log_stream) {
		delete log_stream;
		log_stream = nullptr;
	}
	if (log_file_handle) {
		log_file_handle->close();
		delete log_file_handle;
		log_file_handle = nullptr;
	}

	std::string exe_name = "kill_all.bat";
	system(exe_name.c_str());
}

void DoM_pm::clickBtnOpenCameraslot()
{
	QDir currentDir = QDir::current();
	log_file_path = currentDir.absolutePath() + "/conf/result/test_" + num_lineEdit->text();
	std::string file_name = num_lineEdit->text().toStdString();
	int rc = auxiliary_func::isExists(log_file_path.toStdString());
	if ("" == num_lineEdit->text() || 0 != rc) {
		QMessageBox::warning(nullptr, "警告", "测试名称错误或者重复！", QMessageBox::Ok);
		showInformation("失败::创建测试档案", InfoSymbol::INFO_ERROR);
		return;
	}

	if (nullptr == log_file_handle && nullptr == log_stream) {
		file_name = log_file_path.toStdString() + "/control_command.html";
		log_file_handle = new QFile(QString::fromStdString(file_name));
		if (log_file_handle->open(QIODevice::WriteOnly | QIODevice::Text)) {
			log_stream = new QTextStream(log_file_handle);
		}
	}
	cameraOpenClicked();
}

void DoM_pm::createUI()
{
	// 设置字体
	QFont font_CH("宋体", 12, QFont::Normal);
	QFont font_EN("times new roman", 12, QFont::Normal);

	// 视频流
	QGroupBox* frame_box = new QGroupBox("视频流", this);
	frame_box->setGeometry(10, 20, 820, 470);
	frame_box->setFont(font_CH);

	QString labelText = "<html>"
		"<body>"
		"<p style='text-align: center; style='font-family: Times New Roman; font-size: 14pt; font-style: italic;'>English Text</p>"
		"<p style='text-align: center; style='font-family: SimSun; font-size: 18pt; font-weight: bold;'>中文文本</p>"
		"<p style='text-align: center; style='font-size: 12pt;'>H<sub>2</sub>O</p>"
		"<p>小写的ϕ: &varphi;</p>"
		"<p style='font-family: Times New Roman, sans-serif;'>小写的φ</p>"
		"<p style='font-family: Times New Roman, serif;'>小写的ϕ</p>"
		"</body>"
		"</html>";
	view_label = new QLabel("-.-", frame_box);
	view_label->setGeometry(10, 10, 800, 450);
	view_label->setText(labelText);
	view_label->setFont(font_EN);

	// 运动控制
	QGroupBox* motion_box = new QGroupBox("运动控制", this);
	motion_box->setGeometry(10, 500, 820, 240);
	motion_box->setFont(font_CH);
	int top_left_x = 10;
	int top_left_y = 30;
	int line_width = 60;
	int line_height = 30;

	for (size_t i = 0; i < 4; i++)
		for (size_t j = 0; j < 2; j++) {
			arm_psi_button[i][j] = new QPushButton("-.-", motion_box);
		}
	for (size_t i = 0; i < 4; i++)
		for (size_t j = 0; j < 8; j++) {
			psi_lineEdit[i][j] = new QLineEdit("-.-", motion_box);
		}


	QLabel* name_label[15];

	auto lambda = [motion_box, &name_label, &top_left_x, &top_left_y, &line_width, &line_height](int n, QString labelText) {
		name_label[n] = new QLabel("-.-", motion_box);
		name_label[n]->setGeometry(top_left_x, top_left_y, line_width, line_height);
		name_label[n]->setText(labelText);
	};
	labelText = "<html>""<body>""<p style='text-align: center; font-family: SimSun; font-size: 12pt;'>臂号</p>" "</body>""</html>";
	lambda(0, labelText);
	line_width = 64;
	top_left_x += 70;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: SimSun; font-size: 12pt;'>状态</p>" "</body>""</html>";
	lambda(1, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;font-style: italic;'>l</p>" "< / body>""< / html>";
	lambda(2, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;font-style: italic;'>φ</p>" "< / body>""< / html>";
	lambda(3, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>θ</i><sub>1</sub></p>" "< / body>""< / html>";
	lambda(4, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>δ</i><sub>1</sub></p>" "< / body>""< / html>";
	lambda(5, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>θ</i><sub>2</sub></p>" "< / body>""< / html>";
	lambda(6, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>δ</i><sub>2</sub></p>" "< / body>""< / html>";
	lambda(7, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>q<sub>g</sub></i></p>" "< / body>""< / html>";
	lambda(8, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>times</i></p>" "< / body>""< / html>";
	lambda(9, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; text-align: center; font-family: SimSun; font-size: 12pt;'>指令</p>" "</body>""</html>";
	lambda(10, labelText);

	QString dimensions_varialbes[] = { "mm", "degree", "degree", "degree", "degree", "degree", "degree", "sec" };
	auto lambda_button = [font_CH, &top_left_x, &top_left_y, &line_width, &line_height, this](QPushButton* button, QString labelText) {
		//int f2 = n % 10;
		//int f1 = std::floor(n / 10);
		//this->arm_psi_button[f1][f2] = new QPushButton("-.-", motion_box);
		button->setGeometry(top_left_x, top_left_y, line_width, line_height);
		button->setText(labelText);
		button->setStyleSheet("QPushButton { text-align: center; }");
		button->setFont(font_CH);
	};
	auto lambda_lineEdit = [font_EN, &top_left_x, &top_left_y, &line_width, &line_height, this](QLineEdit* lineEdit, QString labelText, int i) {
		lineEdit->setGeometry(top_left_x, top_left_y, line_width, line_height);
		lineEdit->setText(labelText);
		lineEdit->setAlignment(Qt::AlignCenter);
		lineEdit->setFont(font_EN);
	};
	top_left_x = 10;
	int cons_tlx = top_left_x;
	top_left_y = 70;
	line_width = 60;
	line_height = 30;
	for (size_t j = 0; j < 4; j++) {
		labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman; font-size: 12pt;'>Arm_%1</p>" "</body>""</html>";
		lambda(11 + j, labelText.arg(j + 1));
		line_width = 64;
		top_left_x += 70;
		labelText = "未连接";
		lambda_button(arm_psi_button[j][0], labelText);
		for (size_t i = 0; i < 8; i++) {
			top_left_x += 74;
			labelText = "0.000";
			lambda_lineEdit(psi_lineEdit[j][i], labelText, i);
			psi_lineEdit[j][i]->setPlaceholderText(dimensions_varialbes[i]);
		}
		top_left_x += 74;
		labelText = "未下发";
		lambda_button(arm_psi_button[j][1], labelText);

		top_left_x = cons_tlx;
		top_left_y += 40;
	}
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman; font-size: 12pt;'>Endo</p>" "</body>""</html>";
	name_label[13]->setText(labelText);

	//arm_psi_button[0][0]->setFont(font_CH);

	// 默认参数
	QGroupBox* params_box = new QGroupBox("参数设置", this);
	params_box->setGeometry(850, 20, 474, 320);
	params_box->setFont(font_CH);
	top_left_x = 10;
	top_left_y = 30;
	line_width = 60;
	line_height = 30;
	QLabel* name_label_params[15];

	for (size_t i = 0; i < 6; i++)
		for (size_t j = 0; j < 6; j++) {
			params_lineEdit[i][j] = new QLineEdit("-.-", params_box);
		}

	auto lambda_params = [params_box, &name_label_params, &top_left_x, &top_left_y, &line_width, &line_height](int n, QString labelText) {
		name_label_params[n] = new QLabel("-.-", params_box);
		name_label_params[n]->setGeometry(top_left_x, top_left_y, line_width, line_height);
		name_label_params[n]->setText(labelText);
	};
	labelText = "<html><body>""<p style='text-align: center; font-family: Times New Roman; font-size: 12pt;'><i>tc</i><sub>1</sub></p>" "</body></html>";
	lambda_params(0, labelText);
	top_left_x += 74;
	labelText = "<html><body>""<p style='text-align: center; font-family: Times New Roman; font-size: 12pt;'><i>sup</i><sub>1</sub></p>" "</body></html>";
	lambda_params(1, labelText);
	top_left_x += 74;
	labelText = "<html><body>""<p style='text-align: center; font-family: Times New Roman; font-size: 12pt;'><i>tc</i><sub>2</sub></p>" "</body></html>";
	lambda_params(0, labelText);
	top_left_x += 74;
	labelText = "<html><body>""<p style='text-align: center; font-family: Times New Roman; font-size: 12pt;'><i>sup</i><sub>2</sub></p>" "</body></html>";
	lambda_params(1, labelText);
	top_left_x += 74;
	labelText = "<html><body>""<p style='text-align: center; font-family: Times New Roman; font-size: 12pt;'><i>tc</i><sub>4</sub></p>" "</body></html>";
	lambda_params(0, labelText);
	top_left_x += 74;
	labelText = "<html><body>""<p style='text-align: center; font-family: Times New Roman; font-size: 12pt;'><i>sup</i><sub>4</sub></p>" "</body></html>";
	lambda_params(1, labelText);
	top_left_x = cons_tlx;

	for (size_t j = 0; j < 6; j++) {
		top_left_y += 40;
		for (size_t i = 0; i < 6; i++) {
			labelText = "0.000";
			lambda_lineEdit(params_lineEdit[j][i], labelText, j);
			if (j < 3)
				params_lineEdit[j][i]->setPlaceholderText(dimensions_varialbes[0]);
			else
				params_lineEdit[j][i]->setPlaceholderText(dimensions_varialbes[1]);
			top_left_x += 74;
			if (i == 1 || i == 3)
				top_left_x += 10;
		}
		top_left_x = cons_tlx;

	}

	// 控制框
	QGroupBox* control_box = new QGroupBox("界面控制", this);
	control_box->setGeometry(850, 360, 474, 120);
	control_box->setFont(font_CH);
	for (size_t i = 0; i < 9; i++) {
		control_button[i] = new QPushButton("-.-", control_box);
	}
	top_left_x = 11;
	top_left_y = 30;
	line_width = 80;
	line_height = 28;

	num_lineEdit = new QLineEdit("", control_box);
	num_lineEdit->setGeometry(top_left_x, top_left_y, line_width, line_height);
	num_lineEdit->setAlignment(Qt::AlignCenter);
	num_lineEdit->setFont(font_EN);
	num_lineEdit->setPlaceholderText("测试名称");

	labelText = "打开相机";
	top_left_x += 90;
	lambda_button(control_button[0], labelText);
	top_left_x += 90;
	labelText = "确认参数";
	lambda_button(control_button[1], labelText);
	top_left_x += 90;
	labelText = "开始运动";
	lambda_button(control_button[2], labelText);
	top_left_x += 90;
	labelText = "单次循环";
	lambda_button(control_button[3], labelText);

	top_left_x = 11;
	top_left_y = 70;
	labelText = "逆运动学";
	lambda_button(control_button[4], labelText);
	top_left_x += 90;
	labelText = "刚度观测";
	lambda_button(control_button[5], labelText);
	top_left_x += 90;
	labelText = "当前坐标";
	lambda_button(control_button[6], labelText);
	top_left_x += 90;
	labelText = "组织形态";
	lambda_button(control_button[7], labelText);
	top_left_x += 90;
	labelText = "清除日志";
	lambda_button(control_button[8], labelText);
	// 默认参数
	QGroupBox* log_box = new QGroupBox("日志信息", this);
	log_box->setGeometry(850, 500, 474, 240);
	log_box->setFont(font_CH);
	textBrowser = new QTextBrowser(log_box);
	textBrowser->setGeometry(10, 30, 454, 200);
}

void DoM_pm::showInformationSlot(QString string)
{
	WaitForSingleObject(h_mutex, INFINITE);
	textBrowser->append(string);
	if (log_stream) {
		//QString str = QString::fromStdString(auxiliary_func::removeAngleBrackets(string.toStdString()));
		*log_stream << string << "<br>";
	}
	ReleaseMutex(h_mutex);
}

void DoM_pm::showInformation(QString string, InfoSymbol is) {
	QString string_temp;
	QDateTime current_date_time = QDateTime::currentDateTime();
	switch (is) {
	case InfoSymbol::INFO_NORMAL:
		string = "<font size='4'>" + string + "</font>";
		break;
	case InfoSymbol::INFO_SUCCESS:
		string = "<font color='green' size='4'>" + string + "</font>";
		break;
	case InfoSymbol::INFO_WARNING:
		string = "<font color='blue' size='4'>" + string + "</font>";
		break;
	case InfoSymbol::INFO_ERROR:
		string = "<font color='red' size='4'>" + string + "</font>";
		break;
	}
	string_temp = "<font size='4'>" + current_date_time.toString("MM.dd hh:mm:ss.zzz: ")
		+ "</font>" + string;
	emit Dom->showInformationSig(string_temp);
}

void DoM_pm::displayVideoSlot()
{
	//cv::Mat showPict;
	//DWORD t_start, t_end;
	//int sizeFrame = 1920 * 1080 * 3;
	//U32 len = serverImage.recv((S8*)frame.data, sizeFrame * sizeof(unsigned char), 10);
	//cv::cvtColor(frame, showPict, CV_BGR2RGB);//qt仅支持RGB顺序
	//cv::resize(showPict, showPict, cv::Size(), 0.4, 0.4);
	//QImage srcQImage = QImage((uchar*)(showPict.data), showPict.cols, showPict.rows, QImage::Format_RGB888);

	//QPixmap pix = QPixmap(QPixmap::fromImage(srcQImage));
	//pix = pix.scaled(view_label->size(), Qt::KeepAspectRatio);
	//view_label->setScaledContents(true);
	//view_label->setPixmap(pix);
	//view_label->resize(srcQImage.size());
	//view_label->show();
}

void DoM_pm::connectSecondProgramme()
{

	//if (SR_FALSE == clientVisionCommand.open((S8*)("ToVisionCommand")))
	//{
	//	showInformation("Error connecting to server!\n");
	//	showInformation("Press any key to continue.\n");
	//	getchar();
	//	return;
	//}
	//showInformation("Connecting to the clientMotionPara is complete.\n");

	//int setVision = (int)CONNECT_SECOND_PROGRAMME;
	//clientVisionCommand.send((S8*)&setVision, sizeof(int));


	//frame = cv::Mat(cv::Size(1920, 1080), CV_8UC3, cv::Scalar(255, 255, 0));
	//int sizeTem = 1920 * 1080 * 3; //1296 * 972 * 3
	//if (SR_FALSE == serverImage.create("ImageFrame", 5, sizeTem * sizeof(unsigned char)))
	//{
	//	showInformation("Service creation failed!\n");
	//	showInformation("Press any key to continue.\n");
	//	getchar();
	//	return;
	//}
	//else
	//{
	//	showInformation("serverImage creation complete!\n");
	//}

	//int sizeToolPose = 8 * 2;
	//if (SR_FALSE == serverMotionPara.create("MotionPara", 512, sizeToolPose * sizeof(float)))
	//{
	//	showInformation("Service creation failed!\n");
	//	showInformation("Press any key to continue.\n");
	//	getchar();
	//	return;
	//}
	//else
	//{
	//	showInformation("serverMotionPara creation complete!\n");
	//}

	//createSecondPrograme();
}

std::vector<double> DoM_pm::getPose()
{

	std::vector<double> res;

#if 0

	double received10Pose[8] = { 0 };
	double received10Pose2[8] = { 0 };
	int getSize = 3;
	std::vector<Eigen::Vector4d> quat1;
	std::vector<Eigen::Vector4d> pose1;
	std::vector<Eigen::Vector4d> quat2;
	std::vector<Eigen::Vector4d> pose2;
	static int count_c2(0);
	int i = 0;
	int j1 = 0;
	int c1(0), c2(0);
	float tool_pose[16];
	while (getSize - i)
		//for (int i = 0; i < getSize; i++)
	{
		//getPose();
		U32 len2 = serverMotionPara.recv((S8*)&tool_pose, sizeof(tool_pose), 3000);
		for (size_t ii = 0; ii < 16; ii++)
			toolPose[ii] = (double)tool_pose[ii];

#if 0
		if (!len2 || toolPose[0] == 0 || toolPose[8] == 0 || toolPose[7] < 0.6 || toolPose[15] < 0.6)//toolPose[0] == 0 ||  toolPose[8] == 0 ||  toolPose[7] < 0.8 || toolPose[15] < 0.8
		{
			//j1++;
			if (j1 < 3)
				continue;
			else
			{
				j1 = 0;
				i++;
				std::fill(toolPose, toolPose + sizeof(toolPose) / sizeof(float), 0);
			}
		}

		else
			i++;
		//if (j1 < 2)
		//	showInformation("1");
		j1 = 0;
		Eigen::Vector4f temQuat1 = { toolPose[3], toolPose[4], toolPose[5], toolPose[6] };
		Eigen::Vector4f temPose1 = { toolPose[0], toolPose[1], toolPose[2], toolPose[7] };
		quat1.emplace_back(temQuat1);
		pose1.emplace_back(temPose1);

		Eigen::Vector4f temQuat2 = { toolPose[11], toolPose[12], toolPose[13], toolPose[14] };
		Eigen::Vector4f temPose2 = { toolPose[8], toolPose[9], toolPose[10], toolPose[15] };
		quat2.emplace_back(temQuat2);
		pose2.emplace_back(temPose2);
#else 
		if (!len2 || toolPose[0] == 0 || toolPose[7] < 0.1 || c1>2 || toolPose[7] > 15)
		{

		}
		else
		{
			Eigen::Vector4d temQuat1 = { toolPose[3], toolPose[4], toolPose[5], toolPose[6] };
			Eigen::Vector4d temPose1 = { toolPose[0], toolPose[1], toolPose[2], toolPose[7] };
			quat1.emplace_back(temQuat1);
			pose1.emplace_back(temPose1);
			c1++;
			Eigen::Quaterniond quat = Eigen::Quaterniond(temQuat1);
			quat = quat.normalized();

		}
		count_c2++;
		//if (!len2 || toolPose[8] == 0 || toolPose[15] < 0.1 || c2>2 || toolPose[15] > 5)
		//{

		//}
		//else
		//{

		//	Eigen::Vector4d temQuat2 = { toolPose[11], toolPose[12], toolPose[13], toolPose[14] };
		//	Eigen::Vector4d temPose2 = { toolPose[8], toolPose[9], toolPose[10], toolPose[15] };
		//	quat2.emplace_back(temQuat2);
		//	pose2.emplace_back(temPose2);
		//	c2=3;
		//}

		//if (
		//	(!(!len2 || toolPose[0] == 0 || toolPose[7] < 0.1 || toolPose[7] > 3)
		//	&& c2<3 && (count_c2>50))
		//	
		//	)
		//{
		//	quat2.resize(3);
		//	pose2.resize(3);
		//	Eigen::Vector4d temQuat2 = { toolPose[3], toolPose[4], toolPose[5], toolPose[6] };
		//	Eigen::Vector4d temPose2 = { toolPose[0], toolPose[1], toolPose[2], toolPose[7] };

		//	Eigen::Quaterniond quat = Eigen::Quaterniond(Eigen::Vector4d{ toolPose[4], toolPose[5], toolPose[6], toolPose[3] });
		//	quat = quat.normalized();
		//	Eigen::Vector3d temQuat2_added = 25 * quat.toRotationMatrix().block(0, 2, 3, 1);
		//	temPose2 = temPose2 + Eigen::Vector4d{ temQuat2_added(0), temQuat2_added(1), temQuat2_added(2), 0.0 };
		//	quat2[0] = temQuat2;
		//	pose2[0] = temPose2;
		//	quat2[1] = temQuat2;
		//	pose2[1] = temPose2 * 20.0;
		//	quat2[2] = temQuat2;
		//	pose2[2] = temPose2 / 20.0;
		//	c2 = 3;
		//}

		std::fill(toolPose, toolPose + sizeof(toolPose) / sizeof(double), 0);
		i = c1;
#endif

	}
	count_c2 = 0;
	Eigen::Vector4d resQuat1 = Keith::getQuat(quat1, getSize);
	Eigen::Vector4d resPose1 = Keith::getPose(pose1, getSize);
	received10Pose[0] = resPose1(0); received10Pose[1] = resPose1(1); received10Pose[2] = resPose1(2);
	received10Pose[3] = resQuat1(0); received10Pose[4] = resQuat1(1); received10Pose[5] = resQuat1(2);
	received10Pose[6] = resQuat1(3); received10Pose[7] = resPose1(3);


	Eigen::Vector4d resQuat2 = Keith::getQuat(quat1, getSize);
	Eigen::Vector4d resPose2 = Keith::getPose(pose1, getSize);
	received10Pose2[0] = resPose2(0); received10Pose2[1] = resPose2(1); received10Pose2[2] = resPose2(2);
	received10Pose2[3] = resQuat2(0); received10Pose2[4] = resQuat2(1); received10Pose2[5] = resQuat2(2);
	received10Pose2[6] = resQuat2(3); received10Pose2[7] = resPose2(3);
	quat2.clear();
	pose2.clear();
	quat1.clear();
	pose1.clear();


	for (int iw = 0; iw < 8; iw++)
		res.emplace_back(received10Pose[iw]);
	for (int iw = 0; iw < 8; iw++)
		res.emplace_back(received10Pose2[iw]);
	//if(received10Pose[0]==0||received10Pose2[0]==0)
	//	cvWaitKey();
#endif
	return res;
}

void DoM_pm::setStartOptimal()
{
	//S8 setVision = (S8)START_GETTING_POSE;
	//bool rc = clientVisionCommand.send((S8*)&setVision, sizeof(S8));
}

void DoM_pm::setStopOptimal()
{
	//S8 setVision = (S8)STOP_GETTING_POSE;
	//bool rc = clientVisionCommand.send((S8*)&setVision, sizeof(S8));
	//for (auto elem : toolPose)
	//	elem = 0;
}

void DoM_pm::cameraOpenClicked()
{
	displayTimer->start(15);//15ms后继续运行
	connectSecondProgramme();
	control_button[0]->setEnabled(false);
}
void DoM_pm::createSecondPrograme() {
	//int sizeToolPose = 8;
	//if (SR_FALSE == serverInfo_Arm1.create("from_arm1_to_control", 512, sizeToolPose * sizeof(float)))
	//{
	//	showInformation("错误::from_arm1_to_control 创建失败", InfoSymbol::INFO_ERROR);
	//	return;
	//}
	//else
	//{
	//	showInformation("成功::创建 from_arm1_to_control", InfoSymbol::INFO_SUCCESS);
	//}
	//if (SR_FALSE == serverInfo_Arm2.create("from_arm2_to_control", 512, sizeToolPose * sizeof(float)))
	//{
	//	showInformation("错误::from_arm2_to_control 创建失败", InfoSymbol::INFO_ERROR);
	//	return;
	//}
	//else
	//{
	//	showInformation("成功::创建 from_arm2_to_control", InfoSymbol::INFO_SUCCESS);
	//}
	////if (SR_FALSE == serverInfo_Endo.create("from_Endo_to_control", 512, sizeToolPose * sizeof(float)))
	////{
	////	showInformation("错误::from_Endo_to_control 创建失败", InfoSymbol::INFO_ERROR);
	////	getchar();
	////	return;
	////}
	////else
	////{
	////	showInformation("成功::创建 from_Endo_to_control", InfoSymbol::INFO_SUCCESS);
	////}
	//if (SR_FALSE == serverInfo_Arm4.create("from_arm4_to_control", 512, sizeToolPose * sizeof(float)))
	//{
	//	showInformation("错误::from_arm4_to_control 创建失败", InfoSymbol::INFO_ERROR);
	//	return;
	//}
	//else
	//{
	//	showInformation("成功::创建 from_arm4_to_control", InfoSymbol::INFO_SUCCESS);
	//}
}
void DoM_pm::openSecondPrograme()
{
#if 0
	if (SR_FALSE == clientMotionPara_Arm1.open("from_control_to_Arm1"))
	{
		showInformation("错误::from_control_to_Arm1 连接失败", InfoSymbol::INFO_ERROR);
		return;
	}
	else
	{
		showInformation("成功::连接 from_control_to_Arm1", InfoSymbol::INFO_SUCCESS);
	}
	if (SR_FALSE == clientMotionPara_Arm2.open("from_control_to_Arm2"))
	{
		showInformation("错误::from_control_to_Arm2 连接失败", InfoSymbol::INFO_ERROR);
		return;
	}
	else
	{
		showInformation("成功::连接 from_control_to_Arm2", InfoSymbol::INFO_SUCCESS);
	}
	//if (SR_FALSE == clientMotionPara_Endo.open("from_control_to_Endo"))
	//{
	//	showInformation("错误::from_control_to_Endo 连接失败", InfoSymbol::INFO_ERROR);
	//	getchar();
	//	return;
	//}
	//else
	//{
	//	showInformation("成功::连接 from_control_to_Endo1", InfoSymbol::INFO_SUCCESS);
	//}
	if (SR_FALSE == clientMotionPara_Arm4.open("from_control_to_Arm4"))
	{
		showInformation("错误::from_control_to_Arm4 连接失败", InfoSymbol::INFO_ERROR);
		return;
	}
	else
	{
		showInformation("成功::连接 from_control_to_Arm4", InfoSymbol::INFO_SUCCESS);
	}
#endif
}

void DoM_pm::checkCommandSlot() {
#if 0 
	static float arm1_info[8] = { -1.f };
	static float arm2_info[8] = { -2.f };
	static float endo_info[8] = { -3.f };
	static float arm4_info[8] = { -4.f };
	float sum1 = std::accumulate(arm1_info, arm1_info + 8, 0.0f);
	float sum2 = std::accumulate(arm2_info, arm2_info + 8, 0.0f);
	float sum3 = std::accumulate(endo_info, endo_info + 8, 0.0f);
	float sum4 = std::accumulate(arm4_info, arm4_info + 8, 0.0f);
	serverInfo_Arm1.recv((S8*)&arm1_info, sizeof(arm1_info), 20);
	serverInfo_Arm2.recv((S8*)&arm2_info, sizeof(arm2_info), 20);
	//serverInfo_Endo.recv((S8*)&endo_info, sizeof(endo_info), 20);
	serverInfo_Arm4.recv((S8*)&arm4_info, sizeof(arm4_info), 20);
	float sum11 = std::accumulate(arm1_info, arm1_info + 8, 0.0f);
	float sum22 = std::accumulate(arm2_info, arm2_info + 8, 0.0f);
	float sum33 = std::accumulate(endo_info, endo_info + 8, 0.0f);
	float sum44 = std::accumulate(arm4_info, arm4_info + 8, 0.0f);
	//QString str = QString::number(number, 'f', 3);
	if (abs(sum11 - sum1) > 1e-4f) {
		for (size_t i = 0; i < 8; i++)
			psi_lineEdit[0][i]->setText(QString::number(arm1_info[i], 'f', 3));
	}
	if (abs(sum22 - sum2) > 1e-4f) {
		for (size_t i = 0; i < 8; i++)
			psi_lineEdit[1][i]->setText(QString::number(arm2_info[i], 'f', 3));
	}
	if (abs(sum33 - sum3) > 1e-4f) {
		for (size_t i = 0; i < 8; i++)
			psi_lineEdit[2][i]->setText(QString::number(endo_info[i], 'f', 3));
	}
	if (abs(sum44 - sum4) > 1e-4f) {
		for (size_t i = 0; i < 8; i++)
			psi_lineEdit[3][i]->setText(QString::number(arm4_info[i], 'f', 3));
	}
#endif
}

void DoM_pm::PushButtonClearLogSlot()
{
	textBrowser->clear();

	QString str = num_lineEdit->text();
	QChar lastChar = str.at(str.length() - 1);
	int lastDigit = lastChar.digitValue();
	if (lastDigit % 5 == 0) {
		std::string filePath = "conf/public/psi.log"; // 替换为实际的日志文件路径

		std::vector<Eigen::Vector6d> qa;

		// 打开日志文件
		std::ifstream file(filePath);
		if (!file.is_open()) {
			std::cout << "无法打开日志文件 " << filePath << std::endl;
			return;
		}

		// 读取前三行非空文本
		std::vector<std::string> lines;
		std::string line;
		int lineCount = 0;
		while (getline(file, line) && lineCount < 3) {
			if (!line.empty()) {
				lines.push_back(line);
				lineCount++;
			}
		}

		// 解析每行文本中的数字
		for (const std::string& line : lines) {
			std::stringstream ss(line);
			Eigen::Vector6d numbers;
			double number;
			int numberCount = 0;
			while (ss >> number) {
				numbers[numberCount] = number;
				numberCount++;
				if (numberCount >= 6)
					break;
			}
			qa.push_back(numbers);
			if (numberCount == 6) {
				std::cout << "解析成功，数字为: ";
				for (int ii = 0; ii < numbers.size(); ii++) {
					std::cout << numbers[ii] << " ";
				}
				std::cout << std::endl;
			}
			else {
				std::cout << "解析失败，每行文本不包含六个数字" << std::endl;
			}
		}

		// 关闭文件
		file.close();
		setQaToUI(qa);
	}

}

void DoM_pm::sendCommandToArm1Slot()
{
	std::vector<Eigen::Vector6d> qa = getQafromUI();
	Eigen::Vector6d psi1 = getPsiFromTrocar(qa[0], 0);
	float psi[6];
	for (size_t i = 0; i < 6; i++)
		psi[i] = (float)psi1[i];
	//clientMotionPara_Arm1.send((S8*)&psi, sizeof(psi));
}

void DoM_pm::sendCommandToArm2Slot()
{
	std::vector<Eigen::Vector6d> qa = getQafromUI();
	Eigen::Vector6d psi1 = getPsiFromTrocar(qa[1], 1);
	float psi[6];
	for (size_t i = 0; i < 6; i++)
		psi[i] = (float)psi1[i];
	//clientMotionPara_Arm2.send((S8*)&psi, sizeof(psi));
}

void DoM_pm::sendCommandToArm4Slot()
{
	std::vector<Eigen::Vector6d> qa = getQafromUI();
	Eigen::Vector6d psi1 = getPsiFromTrocar(qa[2], 2);
	float psi[6];
	for (size_t i = 0; i < 6; i++)
		psi[i] = (float)psi1[i];
	//bool rc = clientMotionPara_Arm4.send((S8*)&psi, sizeof(psi));
}

void DoM_pm::calcInverseProblemsSlot() {
	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	std::vector<Eigen::Vector6d> qa2 = qa;
	//diff_method->setParams(getParams());

	static bool is_refresh_target = true;
	static double dev_step = 0.2;
	//
	if (is_refresh_target) {
		//cur_target = getCurrentTarget();
		is_refresh_target = false;
	}

	//

	if (cur_target.sum() < 1e-6) {
		showInformation("到达目标位姿！");
		getCurrentTarget();
		return;
	}
	else {
		std::cout << "cur_target: " << std::endl << cur_target << std::endl;
		std::cout << "dev_mat: " << std::endl << dev_mat << std::endl << std::endl;
	}
	for (size_t i = 0; i < ARM_NUM; i++) {
		//guess[i].segment(10, 4) = Eigen::Vector4d{ 0.0, 0.0, 0.0, 0.0 };
		//guess[i][13] = qa[i][0] * 1e-3;
		//guess[i][12] = qa[i][1];
	}
	double energy = 0.0;
	diff_method->shootInverseOptimization(guess, Rsd_last, qa, energy, cur_target);

	for (size_t j = 1; j < 6; j++) {
		for (size_t i = 0; i < 2; i++) {
			if (qa[i][j] > 8.0)
				qa[i][j] = 8.0;
			else if (qa[i][j] < -8.0)
				qa[i][j] = -8.0;
		}
	}
	is_refresh_target = true;
	//double speed[] = { 1, 0.02, 0.1, 0.1, 0.1, 0.1 };
	double speed[] = { 0.5, 0.01, 0.05, 0.05, 0.05, 0.05 };
	for (size_t i = 0; i < 2; i++) {
		//std::cout << "Arm " << i << " QA2: " << qa2[i].transpose() << std::endl;
		std::cout << "Arm " << i << " QA_before: " << qa[i].transpose() << std::endl;
		for (size_t j = 0; j < 6; j++) {
			double dev = qa[i][j] - qa2[i][j];

			if (abs(dev) > speed[j]) {
				is_refresh_target = false;
				qa[i][j] = qa2[i][j] + speed[j] * dev / abs(dev);
			}
		}
		std::cout << "Arm " << i << " QA_after: " << qa[i].transpose() << std::endl;
	}
	//std::cout << std::endl;
	QString str;

	Eigen::Vector3d dev_p = cur_target.topRightCorner(3, 1) - cur_pose.topRightCorner(3, 1);
	Eigen::Matrix3d dev_r = cur_pose.topLeftCorner(3, 3).transpose() * cur_target.topLeftCorner(3, 3);
	Eigen::AngleAxisd axang(dev_r);
	if (!is_refresh_target) {
		//dev_step = dev_step>0.99 ? 0.99 : (dev_step + 0.1);
		dev_mat.topRightCorner(3, 1) = dev_p * dev_step;

		axang.angle() = axang.angle() * dev_step;
		dev_mat.topLeftCorner(3, 3) = axang.toRotationMatrix();
		//std::cout << "dev_mat: " << std::endl << dev_mat << std::endl;
		//cur_target = cur_target* dev_mat;
		str = "受限残差为：" + QString::number(Rsd_last.head(18).norm(), 'f', 6);
	}
	else {
		//Eigen::Matrix4d dev_mat = Eigen::Matrix4d::Identity();//

		dev_step = 1.0;
		axang.angle() = axang.angle() * dev_step;
		dev_mat.topLeftCorner(3, 3) = axang.toRotationMatrix();
		dev_mat.topRightCorner(3, 1) = dev_p * dev_step;
		str = "当前残差为：" + QString::number(Rsd_last.head(18).norm(), 'f', 6);
		cur_pose = cur_target;
		cur_target = getCurrentTarget();
		guess_tem = guess;
		//dev_mat = Eigen::Matrix4d::Identity();
	}
	if (Rsd_last.head(18).norm() > 0.2)
		guess = guess_tem;
	showInformation(str);
	//std::cout << " Rsd: " << std::endl << Rsd_last.head(18).transpose() << std::endl << std::endl << std::endl;
	/*qa[0][0] = (guess[0][11] + guess[0][13])*1E3;
	qa[0][1] = Keith::rad2deg(guess[0][10] + guess[0][12]);
	qa[0].segment(2, 4) = 1e3*(Rsd_last.segment(18,4));*/
	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Actuation2Psi_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());

	//qa[1][0] = (guess[1][11] + guess[1][13])*1E3;
	//qa[1][1] = Keith::rad2deg(guess[1][10] + guess[1][12]);
	//qa[1].segment(2, 4) = 1e3*(Rsd_last.segment(22, 4));
	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Actuation2Psi_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());



	//Eigen::Vector6d q_tem;
	//q_tem = qa[1];
	//qa[1] = qa[0];
	//qa[2] = q_tem;
	//qa[0] = Eigen::Vector6d::Zero();
	setQaToUI(reorderQa_set(qa));
}

const std::vector<Eigen::Vector6d> DoM_pm::getQafromUI() {

	Eigen::Vector6d psi;


	std::vector<Eigen::Vector6d> qa;
	qa.resize(3);
	int order[] = { 0, 1, 3 };
	int arm_order[] = { 0, 1, 2 };
	if ((!arm_psi_button[0][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {

	}
	else if ((!arm_psi_button[3][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {
		arm_order[0] = 1;
		arm_order[1] = 2;
		arm_order[2] = 0;
	}
	else if((!arm_psi_button[3][0]->isVisible()) &&
		(!arm_psi_button[0][0]->isVisible())){
		arm_order[0] = 0;
		arm_order[1] = 2;
		arm_order[2] = 1;

	}
	else {
		showInformation("错误：：没有选择被驱动的臂", InfoSymbol::INFO_ERROR);
	}

	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 6; j++) {
			qa[i][j] = (double)psi_lineEdit[order[i]][j]->text().toDouble();
		}
	}
	for (size_t i = 0; i < 3; i++) {
		std::cout <<"psi: " << qa[arm_order[i]].transpose() << std::endl;
		diff_method->manipulators[i]->setLso(qa[arm_order[i]][0]);
		qa[arm_order[i]] = Keith::Psi2Actuation_keith(qa[arm_order[i]], diff_method->manipulators[i]->getL12Zeta(),
			diff_method->manipulators[i]->getGc());
		//psi = Keith::Actuation2Psi_keith(qa[i], diff_method->manipulators[i]->getL12Zeta(), diff_method->manipulators[i]->getGc());
		////std::cout <<"qa: " << qa[arm_order[i]].transpose() << std::endl;
		//qa[i] = Keith::Psi2Actuation_keith(psi, diff_method->manipulators[i]->getL12Zeta(), diff_method->manipulators[i]->getGc());
		//psi = Keith::Actuation2Psi_keith(qa[i], diff_method->manipulators[i]->getL12Zeta(), diff_method->manipulators[i]->getGc());
		//std::cout << psi.transpose() << std::endl;
		//qa[i] = Keith::Psi2Actuation_keith(psi, diff_method->manipulators[i]->getL12Zeta(), diff_method->manipulators[i]->getGc());
		//psi = Keith::Actuation2Psi_keith(qa[i], diff_method->manipulators[i]->getL12Zeta(), diff_method->manipulators[i]->getGc());
		//std::cout << psi.transpose() << std::endl;
	}
	//std::cout << std::endl << "q0: " << std::endl;
	//for (size_t i = 0; i < ARM_NUM; i++)
	//	std::cout << qa[i].transpose() << std::endl;
	/*Eigen::Vector6d q_tem = Eigen::Vector6d::Zero();
	if ((!arm_psi_button[0][0]->isVisible()) &&
	(!arm_psi_button[1][0]->isVisible())){

	}
	else if ((!arm_psi_button[3][0]->isVisible()) &&
	(!arm_psi_button[1][0]->isVisible())){
	q_tem = qa[2];
	qa[0] = qa[1];
	qa[1] = q_tem;
	}
	qa[2] = Eigen::Vector6d::Zero();*/
	return qa;
}

const std::vector<Eigen::Vector12d> DoM_pm::getParams() {
	std::vector<Eigen::Vector12d> params;
	params.resize(3);
	//params2.resize(3);
	//int arm_serial[] = { 0, 1, 3 };
	for (size_t i = 0; i < ARM_NUM; i++) {
		for (size_t j = 0; j < 6; j++) {
			params[i][j] = params_lineEdit[j][0 + i * 2]->text().toDouble();
			params[i][j + 6] = params_lineEdit[j][1 + i * 2]->text().toDouble();
		}
	}
	//params2 = params;
	Eigen::Vector12d q_tem;
	//q_tem = params[2];
	//params[0] = params[1];
	//params[1] = q_tem;
	if ((!arm_psi_button[0][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {
	}
	else if ((!arm_psi_button[3][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {
		q_tem = params[2];
		params[0] = params[1];
		params[1] = q_tem;
	}
	params[2] = Eigen::Vector12d::Zero();
	diff_method->setParams(params);

	return params;
}

const Eigen::Matrix4d DoM_pm::getTarget() {
	// 读配置文件
	Eigen::Matrix4d tar = Eigen::Matrix4d::Identity();
	std::vector<double> pose(6);
	std::pair<InfoSymbol, std::string> str;
	bool rc = Keith::readTargetInfo(pose, str);
	showInformation(QString::fromStdString(str.second), str.first);

	Eigen::Vector6d target_pose;
	for (size_t i = 0; i < pose.size(); i++) {
		target_pose[i] = pose[i];
	}
	tar = Keith::fromX2T(target_pose);
	this->target = tar;
	std::cout << "target: " << std::endl << target << std::endl << std::endl;
	return tar;

}

void DoM_pm::confirmParamsSlot() {

	setTrocar();
	getTrocarBase();
	getTarget();
	std::vector<Eigen::Vector12d> params = getParams();
	for (size_t i = 0; i < ARM_NUM; i++) {
		tb[i] = Keith::fromX2T(params[i].head(6));// .inverse();
		std::cout << "trocar out: " << tb[i] << std::endl;
	}
}

void DoM_pm::getDeviceParamsSlot() {
	commandTimer->setSingleShot(true);
	commandTimer->start(15);
}

void DoM_pm::setQaToUI(const std::vector<Eigen::Vector6d>& qa) {
	//Eigen::Vector6d q_tem = Eigen::Vector6d::Zero();
	//if ((!arm_psi_button[0][0]->isVisible()) &&
	//	(!arm_psi_button[1][0]->isVisible())){

	//}
	//else if ((!arm_psi_button[3][0]->isVisible()) &&
	//	(!arm_psi_button[1][0]->isVisible())){
	//	q_tem = qa[0];
	//	qa[2] = qa[1];
	//	qa[1] = q_tem;
	//}
	int order[] = { 0, 1, 3 };
	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 6; j++) {
			psi_lineEdit[order[i]][j]->setText(QString::number(qa[i][j], 'f', 3));
		}
	}
}

void DoM_pm::getCurrentPoseSlot() {
	//cur_pose(2, 3) = 100.0;
	//return;
	setStartOptimal();
	std::vector<double> pose;
	pose = getPose();
	setStopOptimal();
	Eigen::Vector7d vec;
	QString str = "当前坐标为：";
	for (size_t i = 0; i < 7; i++) {
		vec[i] = pose[i];
		str += QString::number(pose[i], 'f', 3) + " ";
	}
	showInformation(str + QString::number(pose[7], 'f', 3));
	cur_pose = Keith::fromQuat2T(vec);

	std::cout << "cur_pose: " << std::endl << cur_pose << std::endl << std::endl;
	//std::cout << "cur_target: " << std::endl << cur_target << std::endl << std::endl;
}

const Eigen::Matrix4d DoM_pm::getCurrentTarget() {
	Eigen::Matrix4d tem = cur_target;
	//tem.topLeftCorner(3, 3) = target.topLeftCorner(3, 3)*dev_mat.topLeftCorner(3, 3);
	//tem.topRightCorner(3, 1) = target.topRightCorner(3, 1) + dev_mat.topRightCorner(3, 1);
	//std::cout << "->Move To: " << std::endl << tem << std::endl;
	Eigen::Vector6d d_vec = Keith::calcDeviationFrom2T(tem, target);
	double step_v = 1.0;
	double step_w = 0.02;
	double angle = d_vec.tail(3).norm();
	double step = d_vec.head(3).norm();

	if (angle < step_w && step < step_v) {
		//showInformation("到达目标位姿！");
		return target;
	}

	Eigen::Vector3d axis = d_vec.tail(3) / angle;
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	//Eigen::AngleAxisd axang(target.topLeftCorner(3,3));
	if (angle < step_w) {
		mat.topLeftCorner(3, 3) = target.topLeftCorner(3, 3);
	}
	else {
		Eigen::AngleAxisd rotation(step_w, axis);
		mat.topLeftCorner(3, 3) = tem.topLeftCorner(3, 3) * rotation.toRotationMatrix();
	}


	Eigen::Vector3d dist_e = d_vec.head(3) / step;
	if (step < step_v) {
		mat.topRightCorner(3, 1) = target.topRightCorner(3, 1);
	}
	else {
		mat.topRightCorner(3, 1) = tem.topRightCorner(3, 1) + dist_e * step_v;
	}
	cur_target = mat;
	return mat;
}

void DoM_pm::setTrocar() {

	std::vector<double> tro;
	std::pair<InfoSymbol, std::string> str;
	bool rc = Keith::readTrocarInfo(tro, str);
	showInformation(QString::fromStdString(str.second), str.first);
	if (rc) {
		trocar[0]->feeding_phi[0] = tro[0];	trocar[0]->feeding_phi[1] = tro[1];
		trocar[0]->curved_path[0] = tro[2];	trocar[0]->curved_path[1] = tro[3];

		trocar[1]->feeding_phi[0] = tro[4];	trocar[1]->feeding_phi[1] = tro[5];
		trocar[1]->curved_path[0] = tro[6];	trocar[1]->curved_path[1] = tro[7];

		trocar[2]->feeding_phi[0] = tro[8];	trocar[2]->feeding_phi[1] = tro[9];
		trocar[2]->curved_path[0] = tro[10]; trocar[2]->curved_path[1] = tro[11];
	}

}

const Eigen::Vector6d DoM_pm::getPsiFromTrocar(const Eigen::Vector6d& qa, const int arm_num) {
	//setTrocar();

	Eigen::Vector6d psi_main = qa;
	Eigen::Vector6d psi_trocar = Eigen::Vector6d::Zero();
	psi_trocar.head(2) = trocar[arm_num]->feeding_phi;
	psi_trocar.segment(2, 2) = trocar[arm_num]->curved_path;

	psi_trocar[3] -= psi_main[1] * 180.0 / Keith::PI;

	CC_tool->setLso(psi_trocar[0]);
	Eigen::Vector6d q_tro = Keith::Psi2Actuation_keith(psi_trocar, CC_tool->getL12Zeta(), CC_tool->getGc());

	Eigen::Vector6d q_main = psi_main;
	Eigen::Vector6d q_sub_system = q_tro + q_main;

	standard_tool->setLso(q_sub_system[0]);
	Eigen::Vector6d psi_sub_system = Keith::Actuation2Psi_keith(q_sub_system,
		standard_tool->getL12Zeta(), standard_tool->getGc());
	return psi_sub_system;
}

void DoM_pm::calcStiffnessObserverSlot() {
	//cur_pose = cur_target;
	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	std::vector<Eigen::Vector27d> guess_cur = guess;
	//diff_method->setParams(getParams());
	static int iter = 1;

#if SHOW_LOG
	std::cout << "->before the " << iter << " times stiffness: " << std::endl;
	for (size_t ii = 0; ii < 2; ii++)
		std::cout << guess[ii].transpose() << std::endl;
#endif
	//cur_pose = cur_target;
	diff_method->shootStiffnessOptimization2(guess_cur, Rsd_last, qa, cur_pose);
	double Rsd_norm = Rsd_last.norm();
	//if (Rsd_norm<2e-4)
	//	guess_tem = guess;
	if (Rsd_norm < 1e-3)
		guess = guess_cur;
#if SHOW_LOG
	std::cout << "stiffness observer Rsd: " << std::endl << Rsd_last.transpose() << std::endl << std::endl;
	std::cout << "->after the " << iter << " times stiffness: " << std::endl;
	for (size_t i = 0; i < 2; i++) {
		std::cout << "arm " << i << " stiffness params are: " << guess[i].transpose() << std::endl;
	}
#endif
	QString str = "刚度观测器当前残差为：" + QString::number(Rsd_norm, 'f', 6);
	showInformation(str);
	iter += 1;

}

void DoM_pm::temTestSlot() {
	Eigen::Vector6d psi_trocar = Eigen::Vector6d::Zero();
	getPsiFromTrocar(psi_trocar, 0);
}

void DoM_pm::connectDevicesSlot() {
	/*openSecondPrograme();
	arm_psi_button[0][0]->setVisible(false);
	arm_psi_button[1][0]->setVisible(false);
	arm_psi_button[3][0]->setVisible(false);*/
	static int pic_num = 0;
	std::string num = std::to_string(pic_num++);
	std::ostringstream oss;
	oss << std::setfill('0') << std::setw(3) << num;
	std::string file_name = log_file_path.toStdString() + "/M" + oss.str() + ".jpg";
	cv::imwrite(file_name, frame);
	if (is_new) {
		pic_num = 0;
		is_new = false;
	}
	getCurrentPoseSlot();
	if (arm_psi_button[0][0]->isVisible() == false) {
		arm_psi_button[0][1]->clicked(true);
	}
	if (arm_psi_button[1][0]->isVisible() == false) {
		arm_psi_button[1][1]->clicked(true);
	}
	if (arm_psi_button[3][0]->isVisible() == false) {
		arm_psi_button[3][1]->clicked(true);
	}
}

void DoM_pm::getTissueParamsSlot() {
	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	////getCurrentPoseSlot();
	Eigen::Vector7d vec7;
	//vec7 << 0, 30.5933, 131.791, 1.0, 0.0, 0.0, 0.0;
	//vec7 << -8.028, 21.448, 82.960, 0.606, 0.421, - 0.623, - 0.259;
	vec7 << -30.355, - 15.659, 116.231, 0.682, - 0.195, - 0.672, 0.213;

	psi_lineEdit[0][0]->setText("90.0");
	psi_lineEdit[0][1]->setText("100.0");
	psi_lineEdit[0][2]->setText("27.0");
	psi_lineEdit[0][3]->setText("-135.0");
	psi_lineEdit[0][4]->setText("27.0");
	psi_lineEdit[0][5]->setText("45.0");

	psi_lineEdit[1][0]->setText("108.0");
	psi_lineEdit[1][1]->setText("100.0");
	psi_lineEdit[1][2]->setText("60.0");
	psi_lineEdit[1][3]->setText("-120.0");
	psi_lineEdit[1][4]->setText("50.0");
	psi_lineEdit[1][5]->setText("60.0");
	arm_psi_button[1][0]->setVisible(false);
	arm_psi_button[0][0]->setVisible(false);
	//cur_pose = Eigen::Matrix4d::Identity();
	cur_pose = Keith::fromQuat2T(vec7);
	std::cout << "before forward kinematics cur_pose: "<<std::endl << cur_pose << std::endl << std::endl;

	//cur_pose(2, 3) = 64.2;
	dev_mat = Eigen::Matrix4d::Identity();
	cur_target = cur_pose;
	cur_target = getCurrentTarget();
	is_new = true;
	Eigen::Vector6d psi_i = Eigen::Vector6d::Zero();
	for (size_t i = 0; i < ARM_NUM; i++) {
		diff_method->manipulators[i]->setLso(qa[i][0]);
		psi_i = Keith::Actuation2Psi_keith(qa[i], diff_method->manipulators[i]->getL12Zeta(), diff_method->manipulators[i]->getGc());
		getTissuePose(cur_pose, psi_i, i);
	}

	Eigen::Vector27d guess0;
	guess0 << 0.0, 0.0, 0.0,										// 初始力
		0.0, 0.0, 0.0,												// 初始力矩
		0.000, 0.0, 0.0, 0.0,										// 丝的应力
		0.0, 0.0, 0.0, 0.01,										// 进给与旋转
		5e-3, 1e-2, 1e-2, 1e-2, 0.0, 0.0, 0.0,						// ksi
		0.0, 0.0, 0.0, 0.0, 0.0, 0.0;								// ksi
	guess.resize(3);
	guess[0] = guess0;
	guess[1] = guess0;
	guess[2] = guess0;
	Eigen::Vector26d Rsd = Eigen::Vector26d::Zero(); Rsd[0] = 1.0;
	Eigen::Matrix4d initial_pose = Eigen::Matrix4d::Identity();
	guess[0][13] = qa[0][0] * 1e-3;	guess[0][12] = qa[0][1];
	guess[1][13] = qa[1][0] * 1e-3;	guess[1][12] = qa[1][1];
	guess[2][13] = qa[2][0] * 1e-3;	guess[2][12] = qa[2][1];
	getParams();
	double energy(0.0);
	guess[0][19] = -0.0;
	diff_method->shootForwardOptimization(guess, Rsd, qa, energy, initial_pose);
#if SHOW_LOG
	std::cout << "forward kinematics Rsd: " << std::endl << Rsd.transpose() << std::endl << std::endl;

	std::cout << "forward kinematics Rsd: " << Rsd.norm() << std::endl;
	std::cout << "forward kinematics initial_pose: " << std::endl << initial_pose << std::endl << std::endl;
#endif
	//vec7 << -0.0000,   23.4968,  132.3500,    0.9999, - 0.0110,    0.0000,    0.0000; //-2N
	//vec7 << -0.0000,    9.4136,  133.2330,    0.9998, - 0.0211,    0.0000,    0.0000; //-5N
	//vec7 << -0.0000, - 2.9769,  132.8050,    0.9996, - 0.0299 ,   0.0000,    0.0000; //-8N
	//vec7 << -0.0000, - 10.2457,  131.8970,    0.9994, - 0.0352,    0.0000,    0.0000; //-10N

	//vec7 << 23.3503,    0.0339,  130.8170,    1.0000, - 0.0046, - 0.0045, - 0.0039; //-2N
	//vec7 << 22.7169, - 13.3489,  129.5960,    0.9998, - 0.0105, - 0.0141, - 0.0080; //-5N
	//vec7 << 21.6788, - 24.4296,  127.4910,    0.9995, - 0.0159, - 0.0249, - 0.0104; //-8N
	//vec7 << 20.8547, - 30.6569,  125.7420,    0.9992, - 0.0199, - 0.0312, - 0.0116; //-10N

	vec7 << -20.355, -15.659, 116.231, 0.682, -0.195, -0.672, 0.213;
	//vec7 << -30.614, - 15.620, 116.430, 0.688, - 0.192, - 0.666, 0.217;
	cur_target = Keith::fromQuat2T(vec7);
	std::cout << "after initial new cur_pose: " << std::endl << cur_pose << std::endl << std::endl;


	guess_tem = guess;
}

void DoM_pm::getTissuePose(const Eigen::Matrix4d& feature_pose,
	const Eigen::Vector6d& psi, const int arm_num) {
	Eigen::Matrix4d pose_kine = Eigen::Matrix4d::Identity();
	pose_kine = Keith::getForwardKinematics(psi, diff_method->manipulators[arm_num]->getL1r2g(),
		diff_method->manipulators[arm_num]->getZeta());
	std::cout << "->arm " << arm_num << " pose_kine: " << std::endl << pose_kine << std::endl;
	pose_kine = tb[arm_num] * pose_kine;
	//std::cout << "->arm " << arm_num << " pose: " << std::endl << pose_kine << std::endl;
	Eigen::Vector3d x, y, z, tem1, tem2;
	z = feature_pose.topRightCorner(3, 1) - pose_kine.topRightCorner(3, 1);
	diff_method->manipulators[arm_num]->setLdo(z.norm());
	//std::cout << "->tissue " << arm_num << " length: " << std::endl << diff_method->manipulators[arm_num]->getLdo() << std::endl;
	z = z / z.norm();
	tem1 = feature_pose.col(2).head(3);
	tem2 = pose_kine.col(2).head(3);
	if (abs((tem1 - tem2).norm()) < 1e-4)
		y = feature_pose.col(1).head(3);
	else
		y = tem1.cross(tem2);
	x = y.cross(z);
	x = x / x.norm();
	y = z.cross(x);
	Eigen::Matrix3d tdb, tde, dR1, dR2;
	tdb.col(0) = x;
	tdb.col(1) = y;
	tdb.col(2) = z;
	tde = tdb;

	dR1 = pose_kine.topLeftCorner(3, 3).transpose() * tdb;
	dR2 = tde.transpose() * feature_pose.topLeftCorner(3, 3);
	Eigen::AngleAxisd rotation1(dR1), rotation2(dR2);
	tem1 = rotation1.axis() * rotation1.angle() * 180.0 / Keith::PI;
	tem2 = rotation2.axis() * rotation2.angle() * 180.0 / Keith::PI;
	Eigen::Vector6d res;
	res.head(3) = tem1;
	res.tail(3) = tem2;

	int arm_serial[3] = { 0, 1, 2 };

	if ((!arm_psi_button[0][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {
	}
	else if ((!arm_psi_button[3][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {
		arm_serial[0] = 1;
		arm_serial[1] = 2;
		arm_serial[2] = 0;
	}

	for (size_t j = 0; j < 6; j++) {
		params_lineEdit[j][arm_serial[arm_num] * 2 + 1]->setText(QString::number(res[j], 'f', 3));
	}
	std::cout << "Arm " << arm_num << " tissue length is " << diff_method->manipulators[arm_num]->getLdo()
		<< "and pose: " << std::endl << pose_kine << std::endl << "DR1: " << std::endl <<
		dR1 << std::endl << "DR2: " << dR2 << std::endl << std::endl;
}

void DoM_pm::connectArm1Slot() {
#if 0
	if (SR_FALSE == clientMotionPara_Arm1.open("from_control_to_Arm1"))
	{
		showInformation("错误::from_control_to_Arm1 连接失败", InfoSymbol::INFO_ERROR);
		return;
	}
	else
	{
		showInformation("成功::连接 from_control_to_Arm1", InfoSymbol::INFO_SUCCESS);
		
	}
#endif
	arm_psi_button[0][0]->setVisible(false);
}

void DoM_pm::connectArm2Slot() {
#if 0
	if (SR_FALSE == clientMotionPara_Arm2.open("from_control_to_Arm2"))
	{
		showInformation("错误::from_control_to_Arm2 连接失败", InfoSymbol::INFO_ERROR);
		return;
	}
	else
	{
		showInformation("成功::连接 from_control_to_Arm2", InfoSymbol::INFO_SUCCESS);
		
	}
#endif
	arm_psi_button[1][0]->setVisible(false);
}

void DoM_pm::connectArm4Slot() {
#if 0
	if (SR_FALSE == clientMotionPara_Arm4.open("from_control_to_Arm4"))
	{
		showInformation("错误::from_control_to_Arm4 连接失败", InfoSymbol::INFO_ERROR);
		return;
	}
	else
	{
		showInformation("成功::连接 from_control_to_Arm4", InfoSymbol::INFO_SUCCESS);
		
	}
#endif
	arm_psi_button[3][0]->setVisible(false);
}

void DoM_pm::getTrocarBase() {
	Eigen::Matrix4d tar = Eigen::Matrix4d::Identity();
	std::vector<double> trocar_base(18);
	std::pair<InfoSymbol, std::string> str;
	bool rc = Keith::readTrocarBaseInfo(trocar_base, str);
	showInformation(QString::fromStdString(str.second), str.first);
	if (rc) {
		int m = 0;
		for (size_t i = 0; i < 3; i++) {
			for (size_t j = 0; j < 6; j++) {
				params_lineEdit[j][i * 2]->setText(QString::number(trocar_base[6 * i + j], 'f', 3));
			}
		}
	}

}
void DoM_pm::singleStepSlot() {
	getCurrentPoseSlot();
	calcInverseProblems3Slot();

	connectDevicesSlot();

}

void DoM_pm::calcInverseProblems2Slot() {
	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	std::vector<Eigen::Vector6d> qa2 = qa;
	//diff_method->setParams(getParams());
	Eigen::Matrix4d tem = Eigen::Matrix4d::Identity();
	double dev_step = 1.0;
	static int iter = 1;
	static int refresh_iter = 0;
	QString str1;
	Eigen::Vector6d drr = Keith::calcDeviationFrom2T(cur_pose, cur_target);
	double drr_p_norm = drr.head(3).norm();
	double drr_r_norm = drr.tail(3).norm();
	if (drr_p_norm > 2.0 || drr_r_norm > 0.035) {
		str1 = "第 " + QString::number(iter) + " 受限误差：" + QString::number(drr_p_norm, 'f', 3) + " " + QString::number(drr_r_norm, 'f', 3);
	}
	else {
		if (abs((target - cur_target).sum()) < 1e-4) {
			showInformation("到达目标位姿！", InfoSymbol::INFO_SUCCESS);
			return;
		}
		str1 = "第 " + QString::number(iter) + " 当前误差：" + QString::number(drr_p_norm, 'f', 3) + " " + QString::number(drr_r_norm, 'f', 3);
		cur_target = getCurrentTarget();
		refresh_iter = 0;
	}
	showInformation(str1);



	refresh_iter++;
	if (refresh_iter == 9) {
		cur_target = getCurrentTarget();
		//calcStiffnessObserverSlot();
		refresh_iter = 0;
		std::cout << "cur_target: " << std::endl << cur_target << std::endl << std::endl;
	}
	diff_method->inverseKinematicsPlanB(qa, cur_pose, cur_target, iter);

	for (size_t j = 1; j < 6; j++) {
		for (size_t i = 0; i < 2; i++) {
			if (qa[i][j] > 8.0)
				qa[i][j] = 8.0;
			else if (qa[i][j] < -8.0)
				qa[i][j] = -8.0;
		}
	}
	Eigen::Vector6d speed;
	speed << 0.4, 0.01, 0.05, 0.05, 0.05, 0.05;
	speed = speed * 0.8;
	for (size_t i = 0; i < 2; i++) {
		//std::cout << "Arm " << i << " QA_before: " << qa[i].transpose() << std::endl;
		for (size_t j = 0; j < 6; j++) {
			double dev = qa[i][j] - qa2[i][j];

			if (abs(dev) > speed[j]) {
				qa[i][j] = qa2[i][j] + speed[j] * dev / abs(dev);
			}
		}
		//std::cout << "Arm " << i << " QA_after: " << qa[i].transpose() << std::endl;
	}

	iter++;

	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Actuation2Psi_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());

	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Actuation2Psi_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());

	/*Eigen::Vector6d q_tem;
	q_tem = qa[1];
	qa[1] = qa[0];
	qa[2] = q_tem;
	qa[0] = Eigen::Vector6d::Zero();*/
	setQaToUI(reorderQa_set(qa));
}

void DoM_pm::singleHitInverseProblems2Slot() {
	getCurrentPoseSlot();
	//getCurrentTarget();
	calcInverseProblems2Slot();
	connectDevicesSlot();
}
void DoM_pm::multipleHitInverseProblems2Slot() {
	inverse2Timer->start(5000);
}
void DoM_pm::stopInverseProblemsSlot() {
	inverse2Timer->stop();
}

void DoM_pm::continueInverseProblemsSlot() {
	singleStepSlot();
	//singleHitInverseProblems2Slot();
}

void DoM_pm::calcInverseProblems3Slot() {
	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	std::vector<Eigen::Vector6d> qa2 = qa;
	std::cout << std::endl << "q1: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
	//static std::vector<Eigen::Vector6d> qa_remember = qa2;
	//diff_method->setParams(getParams());
	static Eigen::Matrix4d target_inv3 = cur_target;
	double dev_step = 1.0;
	static int iter = 1;
	static int refresh_iter = 0;
	QString str1;
	Eigen::Vector6d drr = Keith::calcDeviationFrom2T(cur_pose, cur_target);
	double drr_p_norm = drr.head(3).norm();
	double drr_r_norm = drr.tail(3).norm();
	if (drr_p_norm > 2.0 || drr_r_norm > 0.035) {
		str1 = "第 " + QString::number(iter) + " 当前误差：" + QString::number(drr_p_norm, 'f', 3) + " " + QString::number(drr_r_norm, 'f', 3);
	}
	else {
		if (abs((target - cur_target).sum()) < 1e-4) {
			showInformation("到达目标位姿！", InfoSymbol::INFO_SUCCESS);
			return;
		}
		//dev_mat = Eigen::Matrix4d::Identity();
		str1 = "第 " + QString::number(iter) + " 临时收敛：" + QString::number(drr_p_norm, 'f', 3) + " " + QString::number(drr_r_norm, 'f', 3);
		cur_target = getCurrentTarget();
		//qa_remember = qa;
		refresh_iter = 0;

	}
	showInformation(str1);

	refresh_iter++;
	if (refresh_iter % 2 == 0) {
		if (refresh_iter == 4)
		{
			cur_target = getCurrentTarget();
			refresh_iter = 0;

		}
		//calcStiffnessObserver2Slot();
	}
	Eigen::Matrix4d tem = Eigen::Matrix4d::Identity();

	tem.topLeftCorner(3, 3) = cur_target.topLeftCorner(3, 3) * dev_mat.topLeftCorner(3, 3);
	tem.topRightCorner(3, 1) = cur_target.topRightCorner(3, 1) + dev_mat.topRightCorner(3, 1);

	double t_step = 0.3; // 插值参数，取值范围 [0, 1]
	Eigen::Matrix3d inv_r, tem_r;
	inv_r = target_inv3.topLeftCorner(3, 3); tem_r = tem.topLeftCorner(3, 3);
	Eigen::Quaterniond quaternion1(inv_r);
	Eigen::Quaterniond quaternion2(tem_r);
	Eigen::Quaterniond interpolated_quaternion = quaternion1.slerp(t_step, quaternion2);
	Eigen::Matrix3d interpolated_rotation = interpolated_quaternion.toRotationMatrix();

	target_inv3.topLeftCorner(3, 3) = interpolated_rotation;
	target_inv3.topRightCorner(3, 1) = (1 - t_step) * target_inv3.topRightCorner(3, 1) + t_step * tem.topRightCorner(3, 1);
	target_inv =/* cur_target*/target_inv3;
	double energy_rsd = 0.0;
	diff_method->shootInverseOptimization(guess, Rsd_last, qa, energy_rsd, target_inv3/*cur_target*/);
	for (size_t j = 1; j < 6; j++) {
		for (size_t i = 0; i < 2; i++) {
			if (qa[i][j] > 6.0)
				qa[i][j] = 6.0;
			else if (qa[i][j] < -6.0)
				qa[i][j] = -6.0;
		}
	}
	//qa_remember = qa;

	double Rsd_norm = Rsd_last.head(18).norm();
	QString str;
	str = "当前残差为：" + QString::number(Rsd_norm, 'f', 6);
	showInformation(str);

	//if (Rsd_norm < 2e-4)
	//	guess_tem = guess;
	//if (Rsd_norm > 3e-3)
	//{
	//	guess = guess_tem;
	//}

#if SHOW_LOG
	std::cout << "cur_target: " << std::endl << cur_target << std::endl;
	std::cout << "dev_mat: " << std::endl << dev_mat << std::endl << std::endl;
	std::cout << "->Move To: " << std::endl << target_inv3 << std::endl;
	//std::cout << "->before the " << iter << " times inverse: " << std::endl;
	//for (size_t ii = 0; ii < 2; ii++)
	//	std::cout << guess[ii].transpose() << std::endl;
	std::cout << "inverse kinematics Rsd: " << std::endl << Rsd_last.transpose() << std::endl << std::endl;
	std::cout << "->after the " << iter << " times inverse: " << std::endl;
	for (size_t ii = 0; ii < 2; ii++)
		std::cout << guess[ii].transpose() << std::endl;
#endif

	//qa = qa_remember;
	bool is_refresh_target = true;
	Eigen::Vector6d speed;
	speed << 0.6, 0.02, 0.05, 0.05, 0.06, 0.06;
	speed = speed * 6;
	Eigen::Vector6d dev;
	for (size_t i = 0; i < 2; i++) {
#if SHOW_LOG
		std::cout << "Arm [" << i << "]  QA_before: " << qa[i].transpose() << std::endl;
		//std::cout << "Arm [" << i << "] QA2_before: " << qa2[i].transpose() << std::endl;
#endif
		dev = qa[i] - qa2[i];
		for (size_t j = 0; j < 6; j++) {
			if (abs(dev[j]) > speed[j]) {
				is_refresh_target = false;
				//std::cout << std::endl <<"-->>" << qa[i][j] << std::endl;
				qa[i][j] = qa2[i][j] + speed[j] * dev[j] / abs(dev[j]);
				//std::cout << qa[i][j]<<"=    " << qa2[i][j] << "+    "<< speed[j] <<"/    "<< dev[j] / abs(dev[j]) << std::endl << std::endl;
			}
			else {
				//qa[i][j] = qa2[i][j];
			}
		}		
#if SHOW_LOG
		std::cout << "Arm [" << i << "]   QA_after: " << qa[i].transpose() << std::endl;
		//std::cout << "dev [" << i << "]           : " << dev.transpose() << std::endl;
		//std::cout << "speed: " << speed.transpose() << std::endl;
#endif
	}
	
	if (is_refresh_target) {
		Eigen::Matrix4d tem_tar = Eigen::Matrix4d::Identity();
		std::vector<Eigen::Vector27d> guess0 = guess;
		double energy(0.0);
		diff_method->shootForwardOptimization(guess0, Rsd_last, qa, energy, tem_tar);
		tem_tar.topRightCorner(3, 1) = tem_tar.topRightCorner(3, 1) * 1e3;
		std::cout << std::endl << std::endl<< "->forward: " << tem_tar << std::endl << std::endl;
		Eigen::Vector3d dev_p = cur_target.topRightCorner(3, 1) - cur_pose.topRightCorner(3, 1);
		Eigen::Matrix3d dev_r = cur_pose.topLeftCorner(3, 3).transpose() * cur_target.topLeftCorner(3, 3);
		Eigen::AngleAxisd axang(dev_r);
		dev_step = 1.0;
		axang.angle() = axang.angle() * dev_step;
		dev_mat.topLeftCorner(3, 3) = axang.toRotationMatrix();
		dev_mat.topRightCorner(3, 1) = dev_p * dev_step;
	}

	iter++;
	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Actuation2Psi_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());
	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Actuation2Psi_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());
#if 0
	std::cout << std::endl << "Psi_1: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Psi2Actuation_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());
	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Psi2Actuation_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());
	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Actuation2Psi_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());
	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Actuation2Psi_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());
	std::cout << std::endl << "Psi_2: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Psi2Actuation_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());
	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Psi2Actuation_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());
	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Actuation2Psi_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());
	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Actuation2Psi_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());
	std::cout << std::endl << "Psi_3: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
#endif
	qa = reorderQa_set(qa);
#if 0
	std::cout << std::endl << "Psi_after: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
#endif

	setQaToUI(qa);
}

const std::vector<Eigen::Vector6d> DoM_pm::reorderQa_get(std::vector<Eigen::Vector6d>& qa) {
	Eigen::Vector6d q_tem = Eigen::Vector6d::Zero();
	if ((!arm_psi_button[0][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {
	}
	else if ((!arm_psi_button[3][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {

		q_tem = qa[2];
		qa[0] = qa[1];
		qa[1] = q_tem;
	}
	qa[2] = Eigen::Vector6d::Zero();
	return qa;
}

const std::vector<Eigen::Vector6d> DoM_pm::reorderQa_set(std::vector<Eigen::Vector6d>& qa) {
	Eigen::Vector6d q_tem = Eigen::Vector6d::Zero();
	if ((!arm_psi_button[0][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {
		qa[2] = Eigen::Vector6d::Zero();
	}
	else if ((!arm_psi_button[3][0]->isVisible()) &&
		(!arm_psi_button[1][0]->isVisible())) {

		q_tem = qa[1];
		qa[1] = qa[0];
		qa[2] = q_tem;
		qa[0] = Eigen::Vector6d::Zero();
	}
	return qa;
}

void DoM_pm::calcInverseProblems4Slot() {
	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	std::vector<Eigen::Vector6d> qa2 = qa;
	static Eigen::Matrix4d target_inv = cur_target;
	static int iter = 1;
	static int refresh_iter = 0;
	QString str1;
	Eigen::Vector6d drr = Keith::calcDeviationFrom2T(cur_pose, cur_target);
	double drr_p_norm = drr.head(3).norm();
	double drr_r_norm = drr.tail(3).norm();
	if (drr_p_norm > 1.0 || drr_r_norm > 0.02) {
		str1 = "第 " + QString::number(iter) + " 当前误差：" + QString::number(drr_p_norm, 'f', 3) + " " + QString::number(drr_r_norm, 'f', 3);
	}
	else {
		if (abs((target - cur_target).sum()) < 1e-4) {
			showInformation("到达目标位姿！", InfoSymbol::INFO_SUCCESS);
			return;
		}
		//dev_mat = Eigen::Matrix4d::Identity();
		str1 = "第 " + QString::number(iter) + " 临时收敛：" + QString::number(drr_p_norm, 'f', 3) + " " + QString::number(drr_r_norm, 'f', 3);
		cur_target = getCurrentTarget();
		//qa_remember = qa;
		refresh_iter = 2;

	}
	showInformation(str1);

	refresh_iter++;
	if (refresh_iter % 3 == 0) {
		if (refresh_iter == 6)
		{
			cur_target = getCurrentTarget();
			refresh_iter = 0;

		}
		calcStiffnessObserver2Slot();
	}
	//diff_method->shootInverseOptimization(guess, Rsd_last, qa, target_inv);
	int max_iter = 40;
	int it(0);
	std::vector<std::vector<Eigen::Vector6d>> qa_lib(max_iter);
	std::vector<double> err_lib;
	err_lib.resize(max_iter, 10.0);

	Eigen::MatrixXd J(6, 12), J_(12, 6), J__(12, 6), I6(6, 6), I12(12, 12);
	J = Eigen::MatrixXd::Zero(6, 12);
	J_ = J.transpose();
	J__ = J_;
	I12 = Eigen::MatrixXd::Identity(12, 12);
	I6 = Eigen::MatrixXd::Identity(6, 6);
	double del = 1e-5;
	Eigen::Matrix4d cur_T(Eigen::Matrix4d::Identity()), tem_forward(Eigen::Matrix4d::Identity()),cur_tar(cur_target);
	Eigen::Vector6d dT;
	Eigen::Vector12d energy, dq, q_limit;
	q_limit << 0.5, 0.02, 0.1, 0.1, 0.1, 0.1, 0.5, 0.02, 0.1, 0.1, 0.1, 0.1;
	q_limit = q_limit / 4;

	double err(10.0), energy_sum(0), energy_before(0.0), energy_after(0.0), lambda(1e-6), yita(1e-3);
	diff_method->shootForwardOptimization(guess, Rsd_last, qa, energy_before, cur_T);
	cur_T.topRightCorner(3, 1) = cur_T.topRightCorner(3, 1) * 1e3;
	drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);
	std::vector<Eigen::Vector27d> guess_cur = guess;
	//if()
	std::cout << "cur_tar: " << cur_tar << std::endl << std::endl << std::endl;
	while (it < max_iter) {
		std::cout << "cur_T: " << cur_T << std::endl << std::endl << std::endl;
		for (size_t i = 0; i < 12; i++) {
			if (i < 6)
			{
				qa[0][i] += del;
				//std::cout << "qa[0]: " << qa[0].transpose() << std::endl;

			}
			else {
				qa[1][i] += del;
				//std::cout << "qa[1]: " << qa[1].transpose() << std::endl;

			}
			energy_after = 0.0;



			diff_method->shootForwardOptimization(guess_cur, Rsd_last, qa, energy_after, tem_forward);
			tem_forward.topRightCorner(3, 1) = tem_forward.topRightCorner(3, 1) * 1e3;
			//std::cout << "tem_forward: " << tem_forward << std::endl;
			dT = Keith::calcDeviationFrom2T(cur_T, tem_forward);
			J.col(i) = dT / del;
			energy(i) = (energy_after - energy_before) / del;
			if (i < 6)
			{
				qa[0][i] -= del;
			}
			else {
				qa[1][i] -= del;
			}
		}
		J_ = J.transpose() * Keith::invScale(J * J.transpose() + lambda * I6);
		//J__ = J.transpose() * Keith::pinv(J * J.transpose());
		dq = J_ * drr;// -yita * (I12 - J__ * J) * energy);

		for (size_t ri = 0; ri < 12; ri++) {
			if (abs(dq(ri)) > q_limit(ri))
				dq(ri) = dq(ri) / abs(dq(ri)) * q_limit(ri);
		}

		qa[0] = qa[0] + dq.head(6);
		qa[1] = qa[1] + dq.tail(6);
		//std::cout <<"dq: " << dq.transpose() << std::endl;
		energy_before = 0.0;
		diff_method->shootForwardOptimization(guess_cur, Rsd_last, qa, energy_before, cur_T);
		cur_T.topRightCorner(3, 1) = cur_T.topRightCorner(3, 1) * 1e3;
		drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);
		double prr(drr.head(3).norm()), arr(drr.tail(3).norm());
		err = prr + arr * 180 / Keith::PI;
		std::cout << "posi_err: " << prr << "\tang_err: " << arr * 180 / Keith::PI << std::endl;
		if (err < 0.1) {
			break;
		}
		it++;
		//err_lib[iter] = err;
		//qa_lib[iter] = qa;
	}

	//auto minElementIter = std::min_element(err_lib.begin(), err_lib.end());
	//int minIndex = std::distance(err_lib.begin(), minElementIter);
	//qa = qa_lib[minIndex];


	for (size_t j = 1; j < 6; j++) {
		for (size_t i = 0; i < 2; i++) {
			if (qa[i][j] > 6.0)
				qa[i][j] = 6.0;
			else if (qa[i][j] < -6.0)
				qa[i][j] = -6.0;
		}
	}
#if SHOW_LOG
	std::cout << "cur_target: " << std::endl << cur_target << std::endl;
	std::cout << "drr: " << std::endl << dev_mat << std::endl << std::endl;
#endif
	Eigen::Vector6d speed;
	speed << 0.6, 0.02, 0.05, 0.05, 0.06, 0.06;
	speed = speed * 0.6;
	for (size_t i = 0; i < 2; i++) {
#if SHOW_LOG
		std::cout << "Arm " << i << " QA_before: " << qa[i].transpose() << std::endl;
#endif
		for (size_t j = 0; j < 6; j++) {
			double dev = qa[i][j] - qa2[i][j];
			if (abs(dev) > speed[j]) {
				qa[i][j] = qa2[i][j] + speed[j] * dev / abs(dev);
			}
		}
#if SHOW_LOG
		std::cout << "Arm " << i << " QA_after: " << qa[i].transpose() << std::endl;
#endif
	}
	iter++;

	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Actuation2Psi_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());
	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Actuation2Psi_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());
#if 0
	std::cout << std::endl << "Psi_before: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
#endif
	qa = reorderQa_set(qa);
#if 0
	std::cout << std::endl << "Psi_after: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
#endif

	setQaToUI(qa);
}

void DoM_pm::calcInverseProblems5Slot() {
	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	std::vector<Eigen::Vector6d> qa2 = qa;
	static Eigen::Matrix4d target_inv = cur_target;
	static int iter = 1;
	static int refresh_iter = 0;
	QString str1;
	Eigen::Vector6d drr = Keith::calcDeviationFrom2T(cur_pose, cur_target);
	double drr_p_norm = drr.head(3).norm();
	double drr_r_norm = drr.tail(3).norm();
	if (drr_p_norm > 1.0 || drr_r_norm > 0.02) {
		str1 = "第 " + QString::number(iter) + " 当前误差：" + QString::number(drr_p_norm, 'f', 3) + " " + QString::number(drr_r_norm, 'f', 3);
	}
	else {
		if (abs((target - cur_target).sum()) < 1e-4) {
			showInformation("到达目标位姿！", InfoSymbol::INFO_SUCCESS);
			return;
		}
		//dev_mat = Eigen::Matrix4d::Identity();
		str1 = "第 " + QString::number(iter) + " 临时收敛：" + QString::number(drr_p_norm, 'f', 3) + " " + QString::number(drr_r_norm, 'f', 3);
		cur_target = getCurrentTarget();
		//qa_remember = qa;
		refresh_iter = 0;

}
	showInformation(str1);

	refresh_iter++;
	if (refresh_iter % 4 == 0) {
		if (refresh_iter == 8)
		{
			cur_target = getCurrentTarget();
			refresh_iter = 0;

		}
		calcStiffnessObserver2Slot();
	}
	//diff_method->shootInverseOptimization(guess, Rsd_last, qa, target_inv);
	int max_iter = 20;
	int it(0);
	std::vector<std::vector<Eigen::Vector6d>> qa_lib(max_iter);
	std::vector<double> err_lib;
	err_lib.resize(max_iter, 10.0);

	Eigen::MatrixXd J(6, 12), J_(12, 6), J__(12, 6), I6(6, 6), Ie6(6, 6), I12(12, 12);
	J = Eigen::MatrixXd::Zero(6, 12);
	J_ = J.transpose();
	J__ = J_;
	I12 = Eigen::MatrixXd::Identity(12, 12);
	I6 = Eigen::MatrixXd::Identity(6, 6);
	double del = 1e-4;
	Ie6 = Eigen::MatrixXd::Identity(6, 6)* del;
	
	cur_target = getCurrentTarget();
	//cur_target = getCurrentTarget();
	//cur_target = getCurrentTarget();
	Eigen::Matrix4d cur_T(Eigen::Matrix4d::Identity()), tem_forward(Eigen::Matrix4d::Identity()),cur_tar(cur_target);
	Eigen::Vector6d dT,tem_dT;
	Eigen::Vector12d energy, dq,tem_dq,cur_qa;

	cur_tar.topRightCorner(3, 1) = cur_tar.topRightCorner(3, 1) * 1e-3;
	double energy_before = 0.0;
	diff_method->shootForwardOptimization(guess, Rsd_last, qa, energy_before, cur_T);
	dT = Keith::fromT2X(cur_T); dT.tail(3) = dT.tail(3) / 18000 * Keith::PI;
	drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);

	double err(10.0), energy_sum(0), energy_after(0.0), lambda(1e-6), yita(1e-3);
	tem_dq.head(6) = qa[0];
	tem_dq.tail(6) = qa[1];
	cur_qa = tem_dq;
	qa2[0] = cur_qa.head(6);
	qa2[1] = cur_qa.tail(6);

	std::cout << it << std::endl << cur_tar << std::endl;
	std::cout << cur_T << std::endl << std::endl;

	std::vector<Eigen::Vector27d> guess_cur = guess;
	while (it < max_iter) {
		std::cout <<it<<std::endl<< cur_qa.transpose() << std::endl;
		std::cout << dT.transpose() << std::endl;
		for (size_t i = 0; i < 6; i++) {
			//qa2 = qa;
			tem_dT = dT + Ie6.col(i);
			
			tem_dT.tail(3) = tem_dT.tail(3) * 18000 / Keith::PI;
			tem_dT.head(3) = tem_dT.head(3) * 1e3;
			std::cout << tem_dT.transpose() << std::endl;
			tem_forward = Keith::fromX2T(tem_dT);
			std::cout << tem_forward << std::endl << std::endl;
			energy_after = 0.0;
			//tem_forward.topRightCorner(3, 1) *= 1e3;
			diff_method->shootInverseOptimization(guess_cur, Rsd_last, qa2, energy_after, tem_forward);
			tem_dq.head(6) = qa2[0];
			tem_dq.tail(6) = qa2[1];
			J_.col(i) = (tem_dq - cur_qa) / del;
			energy(i) = (energy_after - energy_before) / del;
			std::cout << tem_dq.transpose() << std::endl;
		}
		//J__ = J.transpose() * Keith::pinv(J * J.transpose());
		dq = J_/1000 * drr;// -yita * (I12 - J__ * J) * energy);
		qa2[0] = qa2[0] + dq.head(6);
		qa2[1] = qa2[1] + dq.tail(6);
		std::cout <<"dq: " << dq.transpose() << std::endl;
		energy_before = 0.0;
		diff_method->shootInverseOptimization(guess_cur, Rsd_last, qa2, energy_after, cur_T);
		cur_qa.head(6) = qa2[0];
		cur_qa.tail(6) = qa2[1];
		dT = Keith::fromT2X(cur_T); dT.tail(3) = dT.tail(3) / 180 * Keith::PI;
		drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);
		double prr(drr.head(3).norm()), arr(drr.tail(3).norm());
		err = prr*1e3 + arr * 180 / Keith::PI;
		if (err < 0.5) {
			qa = qa2;
			guess = guess_cur;
			break;
		}
		it++;
		//err_lib[iter] = err;
		//qa_lib[iter] = qa;
	}

	//auto minElementIter = std::min_element(err_lib.begin(), err_lib.end());
	//int minIndex = std::distance(err_lib.begin(), minElementIter);
	//qa = qa_lib[minIndex];


	for (size_t j = 1; j < 6; j++) {
		for (size_t i = 0; i < 2; i++) {
			if (qa[i][j] > 6.0)
				qa[i][j] = 6.0;
			else if (qa[i][j] < -6.0)
				qa[i][j] = -6.0;
		}
	}
#if SHOW_LOG
	std::cout << "cur_target: " << std::endl << cur_target << std::endl;
	std::cout << "drr: " << std::endl << dev_mat << std::endl << std::endl;
	std::cout << "->before the " << iter << " times inverse: " << std::endl;
	for (size_t ii = 0; ii < 2; ii++)
		std::cout << guess[ii].transpose() << std::endl;
	std::cout << "->after the " << iter << " times inverse: " << std::endl;
	for (size_t ii = 0; ii < 2; ii++)
		std::cout << guess[ii].transpose() << std::endl;
#endif
	Eigen::Vector6d speed;
	speed << 0.6, 0.02, 0.05, 0.05, 0.06, 0.06;
	speed = speed * 0.6;
	for (size_t i = 0; i < 2; i++) {
#if SHOW_LOG
		std::cout << "Arm " << i << " QA_before: " << qa[i].transpose() << std::endl;
#endif
		for (size_t j = 0; j < 6; j++) {
			double dev = qa[i][j] - qa2[i][j];
			if (abs(dev) > speed[j]) {
				qa[i][j] = qa2[i][j] + speed[j] * dev / abs(dev);
			}
		}
#if SHOW_LOG
		std::cout << "Arm " << i << " QA_after: " << qa[i].transpose() << std::endl;
#endif
	}
	iter++;

	diff_method->manipulators[0]->setLso(qa[0][0]);
	qa[0] = Keith::Actuation2Psi_keith(qa[0], diff_method->manipulators[0]->getL12Zeta(), diff_method->manipulators[0]->getGc());
	diff_method->manipulators[1]->setLso(qa[1][0]);
	qa[1] = Keith::Actuation2Psi_keith(qa[1], diff_method->manipulators[1]->getL12Zeta(), diff_method->manipulators[1]->getGc());
#if 0
	std::cout << std::endl << "Psi_before: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
#endif
	qa = reorderQa_set(qa);
#if 0
	std::cout << std::endl << "Psi_after: " << std::endl;
	for (size_t i = 0; i < ARM_NUM; i++)
		std::cout << qa[i].transpose() << std::endl;
#endif

	setQaToUI(qa);
}


void DoM_pm::calcStiffnessObserver2Slot() {
	//cur_pose = cur_target;
	const int max_iter = 50;
	int iter = 0;
	static int it = 0;
	std::vector<std::vector<Eigen::Vector27d>> guess_lib(max_iter);
	static double cter[max_iter] = {};

	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	std::vector<Eigen::Vector27d> guess_cur = guess;
	std::vector<Eigen::Vector27d> guess_cur2 = guess;
		
	Eigen::MatrixXd J(6, 14), J_(14, 6), J1(6, 6), J1_(6, 6), I6(6, 6), I14(14, 14);
	J = Eigen::MatrixXd::Zero(6, 14);
	J_ = J.transpose();
	I14 = Eigen::MatrixXd::Identity(14, 14);
	I6 = Eigen::MatrixXd::Identity(6, 6);
	Eigen::VectorXd d_guess0 = Eigen::VectorXd::Constant(14, 5e-6);
	d_guess0.head(11) = d_guess0.head(11)*1e2;
	double del = 1e-2;
	Eigen::Matrix4d cur_T(Eigen::Matrix4d::Identity()), tem_forward(Eigen::Matrix4d::Identity()),cur_tar(cur_pose);
	cur_tar.topRightCorner(3, 1) = cur_tar.topRightCorner(3, 1) * 1e-3;

	Eigen::Vector6d dT, drr,dg2;
	Eigen::Vector14d dguess;
	double energy_sum(0), energy_before(0.0), energy_after(0.0), lambda(1e-6), yita(1e-3);
	diff_method->shootForwardOptimization(guess_cur, Rsd_last, qa, energy_before, cur_T);
	drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);
	double err = 10.0;
	int guess_size = dguess.size();
	while (iter < max_iter)
	{
		/*std::cout << std::endl << std::endl << std::endl << std::endl <<"----->>>>>" << iter << std::endl << std::endl;
		std::cout << "->" << guess_cur[0].head(21).transpose() << std::endl;
		std::cout << "->" << guess_cur[1].head(21).transpose() << std::endl;*/
		
		for (size_t i = 0; i < guess_size; i++)
		{
			guess_cur2 = guess_cur;
			del = d_guess0[i];
			int tem = i - guess_size / 2;
			if (i < 4 || (tem > -1 && tem < 4))
				continue;
			if (i < guess_size / 2)
			{
				guess_cur2[0][i + 14] += del;
			}
			else {
				guess_cur2[1][tem + 14] += del;
			}
			energy_after = 0.0;
			tem_forward = Eigen::Matrix4d::Identity();
			diff_method->shootForwardOptimization(guess_cur2, Rsd_last, qa, energy_after, tem_forward);
			dT = Keith::calcDeviationFrom2T(cur_T, tem_forward);
			J.col(i) = dT / del;
			//if (i < guess_size / 2)
			//{
			//	guess_cur[0][i + 14] -= del;
			//}
			//else {
			//	guess_cur[1][tem + 14] -= del;
			//}
		}
#if 0		
		J_ = J.transpose() * Keith::invScale(J * J.transpose() + lambda * I6);
		
		dguess = J_ * drr;

		
		Eigen::Vector4d m_dg2 = dguess.head(4);
		double max_ele = m_dg2.cwiseAbs().maxCoeff();
		if (max_ele > 0.1) {
			dguess = dguess / max_ele * 1e-1;
			//dguess.head(4) = m_dg2;
		}
		Eigen::Vector3d m_dg = dguess.tail(3);
		max_ele = m_dg.cwiseAbs().maxCoeff();
		if (max_ele > 2e-3) {
			dguess = dguess / max_ele * 2e-3;
			//dguess.tail(3) = m_dg;
		}
		m_dg = dguess.segment(4,3);
		max_ele = m_dg.cwiseAbs().maxCoeff();
		if (max_ele > 0.2) {
			dguess = dguess / max_ele * 0.2;
			//dguess.segment(4, 3) = m_dg;
		}
		m_dg2 = dguess.segment(7,4);
		max_ele = m_dg2.cwiseAbs().maxCoeff();
		if (max_ele > 0.1) {
			dguess = dguess / max_ele * 1e-1;
			//dguess.segment(7, 4) = m_dg2;
		}
		guess_cur[0].segment(14, 7) = guess_cur[0].segment(14, 7) + dguess.head(7);
		guess_cur[1].segment(14, 7) = guess_cur[1].segment(14, 7) + dguess.tail(7);

		for (size_t ig = 0; ig < 4; ig++) {
			if (guess_cur[0][14 + ig] < 1e-2)
				guess_cur[0][14 + ig] = 1e-2;
			if (guess_cur[1][14 + ig] < 1e-2)
				guess_cur[1][14 + ig] = 1e-2;
		}
#else
		J1.col(0) = J.col(4); J1.col(1) = J.col(5); J1.col(2) = J.col(6);
		J1.topRightCorner(6,3) = J.topRightCorner(6, 3);
		J1_ = J1.transpose() * Keith::invScale(J1 * J1.transpose() + lambda * I6);
		dg2 = J1_ * drr;
		
		
		Eigen::Vector3d m_dg2 = dg2.head(3);
		
		double max_ele = m_dg2.cwiseAbs().maxCoeff();
		if (max_ele > 1.0) {
			//std::cout << "before_scale: " << dg2.transpose() << std::endl;
			m_dg2 = m_dg2 / max_ele * 1.0;
			dg2.head(3) = m_dg2;
			//std::cout << "after_scale:  " << dg2.transpose() << std::endl;
		}
		Eigen::Vector3d m_dg = dg2.tail(3);
		max_ele = m_dg.cwiseAbs().maxCoeff();
		if (max_ele > 4e-3) {
			//std::cout << "before_scale: " << dg2.transpose() << std::endl;
			m_dg = m_dg / max_ele * 4e-3;
			dg2.tail(3) = m_dg;
			//std::cout << "after_scale:  " << dg2.transpose() << std::endl;
		}
		//std::cout << "->" << dg2.transpose() << std::endl;
		guess_cur[0].segment(18, 3) = guess_cur[0].segment(18, 3) + dg2.head(3);
		guess_cur[1].segment(18, 3) = guess_cur[1].segment(18, 3) + dg2.tail(3);
#endif
		Eigen::Vector6d tem_show;
		tem_show.head(3) = guess_cur[0].segment(18, 3);
		tem_show.tail(3) = guess_cur[1].segment(18, 3);
		std::cout << "->F&M: " << tem_show.transpose() << std::endl;
		//std::cout << std::endl << dguess.transpose() << std::endl << std::endl;
		diff_method->shootForwardOptimization(guess_cur, Rsd_last, qa, energy_after, cur_T);
		drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);

		double prr(drr.head(3).norm()), arr(drr.tail(3).norm());
		err = prr*1e3 + arr * 180 / Keith::PI;
		std::cout << "->position_error: " << prr * 1e3 << "\t\t" << "angular_error: " << arr * 180 / Keith::PI << std::endl;
		if (err < 1e-1) {
			guess = guess_cur;
			showInformation("第"+ QString::number(++it) +"次刚度残差为： "+QString::number(err,'f',3));
			std::cout << "cur_T: " << std::endl << cur_T << std::endl;
			return;
		}
		cter[iter] = err;
		guess_lib[iter] = guess_cur;
		iter += 1;
	}
	std::cout << iter << std::endl;
	auto minElementIter = std::min_element(cter, cter + max_iter);
	std::cout << minElementIter << std::endl;
	int minIndex = std::distance(cter, minElementIter);
	std::cout << minIndex << std::endl;
	if(err<3)
		guess = guess_lib[minIndex];
	showInformation("第" + QString::number(++it) + "次刚度残差为： " + QString::number(cter[minIndex], 'f', 3));
	std::cout << "cur_T: " << std::endl << cur_T << std::endl;
}

void DoM_pm::calcStiffnessObserver3Slot() {
	//cur_pose = cur_target;
	const int max_iter = 50;
	int iter = 0;
	static int it = 0;
	std::vector<std::vector<Eigen::Vector27d>> guess_lib(max_iter);
	static double cter[max_iter] = {};

	std::vector<Eigen::Vector6d> qa0 = getQafromUI();
	std::vector<Eigen::Vector6d> qa = reorderQa_get(qa0);
	std::vector<Eigen::Vector27d> guess_cur = guess;
	std::vector<Eigen::Vector27d> guess_cur2 = guess;

	Eigen::MatrixXd J(6, 12), J_(12, 6), J1(6, 6), J1_(6, 6), I6(6, 6), I12(12, 12);
	J = Eigen::MatrixXd::Zero(6, 12);
	J_ = J.transpose();
	I12 = Eigen::MatrixXd::Identity(12, 12);
	I6 = Eigen::MatrixXd::Identity(6, 6);
	Eigen::VectorXd d_guess0 = Eigen::VectorXd::Constant(12, 5e-6);
	//d_guess0.head(11) = d_guess0.head(11) * 1e2;
	double del = 1e-4;
	Eigen::Matrix4d cur_T(Eigen::Matrix4d::Identity()), tem_forward(Eigen::Matrix4d::Identity()), cur_tar(cur_pose);
	cur_tar.topRightCorner(3, 1) = cur_tar.topRightCorner(3, 1) * 1e-3;

	Eigen::Vector6d dT, drr, dg2;
	Eigen::Vector14d dguess;
	double energy_sum(0), energy_before(0.0), energy_after(0.0), lambda(1e-6), yita(1e-3);
	diff_method->shootForwardOptimization(guess_cur, Rsd_last, qa, energy_before, cur_T);
	drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);
	double err = 10.0;
	int guess_size = dguess.size();
	while (iter < max_iter)
	{
		/*std::cout << std::endl << std::endl << std::endl << std::endl <<"----->>>>>" << iter << std::endl << std::endl;
		std::cout << "->" << guess_cur[0].head(21).transpose() << std::endl;
		std::cout << "->" << guess_cur[1].head(21).transpose() << std::endl;*/

		for (size_t i = 0; i < guess_size; i++)
		{
			guess_cur2 = guess_cur;
			del = d_guess0[i];
			int tem = i - guess_size / 2;
			if (i < guess_size / 2)
			{
				guess_cur2[0][i + 21] += del;
				std::cout << "->0:    " << guess_cur[0].transpose() << std::endl;
			}
			else {
				guess_cur2[1][tem + 21] += del;
				std::cout << "->1:    " << guess_cur[1].transpose() << std::endl;
			}
			energy_after = 0.0;
			tem_forward = Eigen::Matrix4d::Identity();
			diff_method->shootForwardOptimization(guess_cur2, Rsd_last, qa, energy_after, tem_forward);
			dT = Keith::calcDeviationFrom2T(cur_T, tem_forward);
			J.col(i) = dT / del;
			//if (i < guess_size / 2)
			//{
			//	guess_cur[0][i + 14] -= del;
			//}
			//else {
			//	guess_cur[1][tem + 14] -= del;
			//}
		}
#if 1		
		J_ = J.transpose() * Keith::invScale(J * J.transpose() + lambda * I6);

		dguess = J_ * drr;


		Eigen::Vector3d m_dg2 = dguess.head(3);

		double max_ele = m_dg2.cwiseAbs().maxCoeff();
		if (max_ele > 1.0) {
			//std::cout << "before_scale: " << dg2.transpose() << std::endl;
			m_dg2 = m_dg2 / max_ele * 1.0;
			dguess.head(3) = m_dg2;
			//std::cout << "after_scale:  " << dg2.transpose() << std::endl;
		}
		Eigen::Vector3d m_dg = dguess.segment(3,3);
		max_ele = m_dg.cwiseAbs().maxCoeff();
		if (max_ele > 4e-3) {
			//std::cout << "before_scale: " << dg2.transpose() << std::endl;
			m_dg = m_dg / max_ele * 4e-3;
			dguess.segment(3, 3) = m_dg;
			//std::cout << "after_scale:  " << dg2.transpose() << std::endl;
		}

		m_dg2 = dguess.segment(6, 3);
		max_ele = m_dg2.cwiseAbs().maxCoeff();
		if (max_ele > 1.0) {
			//std::cout << "before_scale: " << dg2.transpose() << std::endl;
			m_dg2 = m_dg2 / max_ele * 1.0;
			dguess.segment(6, 3) = m_dg2;
			//std::cout << "after_scale:  " << dg2.transpose() << std::endl;
		}
		m_dg = dguess.segment(9, 3);
		max_ele = m_dg.cwiseAbs().maxCoeff();
		if (max_ele > 4e-3) {
			//std::cout << "before_scale: " << dg2.transpose() << std::endl;
			m_dg = m_dg / max_ele * 4e-3;
			dguess.tail(3) = m_dg;
			//std::cout << "after_scale:  " << dg2.transpose() << std::endl;
		}
		guess_cur[0].tail(6) = guess_cur[0].tail(6) + dguess.head(6);
		guess_cur[1].tail(6) = guess_cur[1].tail(6) + dguess.tail(6);

#else
		J1.col(0) = J.col(4); J1.col(1) = J.col(5); J1.col(2) = J.col(6);
		J1.topRightCorner(6, 3) = J.topRightCorner(6, 3);
		J1_ = J1.transpose() * Keith::invScale(J1 * J1.transpose() + lambda * I6);
		dg2 = J1_ * drr;


		Eigen::Vector3d m_dg2 = dg2.head(3);

		double max_ele = m_dg2.cwiseAbs().maxCoeff();
		if (max_ele > 1.0) {
			//std::cout << "before_scale: " << dg2.transpose() << std::endl;
			m_dg2 = m_dg2 / max_ele * 1.0;
			dg2.head(3) = m_dg2;
			//std::cout << "after_scale:  " << dg2.transpose() << std::endl;
		}
		Eigen::Vector3d m_dg = dg2.tail(3);
		max_ele = m_dg.cwiseAbs().maxCoeff();
		if (max_ele > 4e-3) {
			//std::cout << "before_scale: " << dg2.transpose() << std::endl;
			m_dg = m_dg / max_ele * 4e-3;
			dg2.tail(3) = m_dg;
			//std::cout << "after_scale:  " << dg2.transpose() << std::endl;
		}
		//std::cout << "->" << dg2.transpose() << std::endl;
		guess_cur[0].segment(18, 3) = guess_cur[0].segment(18, 3) + dg2.head(3);
		guess_cur[1].segment(18, 3) = guess_cur[1].segment(18, 3) + dg2.tail(3);
#endif

		std::cout << "->" << guess_cur[0].tail(6).transpose() << std::endl;
		std::cout << "->" << guess_cur[1].tail(6).transpose() << std::endl;
		//std::cout << std::endl << dguess.transpose() << std::endl << std::endl;
		diff_method->shootForwardOptimization(guess_cur, Rsd_last, qa, energy_after, cur_T);
		drr = Keith::calcDeviationFrom2T(cur_T, cur_tar);

		double prr(drr.head(3).norm()), arr(drr.tail(3).norm());
		err = prr * 1e3 + arr * 180 / Keith::PI;
		std::cout << "position_error: " << prr * 1e3 << "\t\t" << "angular_error: " << arr * 180 / Keith::PI << std::endl;
		if (err < 1e-1) {
			guess = guess_cur;
			showInformation("第" + QString::number(++it) + "次刚度残差为： " + QString::number(err, 'f', 3));
			std::cout << "cur_T: " << std::endl << cur_T << std::endl;
			return;
		}
		cter[iter] = err;
		guess_lib[iter] = guess_cur;
		iter += 1;
	}
	std::cout << iter << std::endl;
	auto minElementIter = std::min_element(cter, cter + max_iter);
	std::cout << minElementIter << std::endl;
	int minIndex = std::distance(cter, minElementIter);
	std::cout << minIndex << std::endl;
	if (err < 3)
		guess = guess_lib[minIndex];
	showInformation("第" + QString::number(++it) + "次刚度残差为： " + QString::number(cter[minIndex], 'f', 3));
	std::cout << "cur_T: " << std::endl << cur_T << std::endl;
}
