#include "DoM_pro.h"

#include <QLabel>
#include <QGroupBox>
#include <cmath>
#include <QDir>
#include <QDateTime>
#include <QFont>
#include <QMessageBox>
#include <QButtonGroup>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "difference_function.h"
#include "auxiliary_function.h"
#include "timer.h"

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

	Eigen::Vector12d guess0;
	guess0 << 0.0, 0.0, 0.0,										// 初始力
		0.0, 0.0, 0.0,												// 初始力矩
		0.000, 0.0, 0.0, 0.0,										// 丝的应力
		0.03, 0.0;													// 进给与旋转

	guess = guess0;
	Rsd_last = Eigen::Vector12d::Zero();
	cur_pose = Eigen::Matrix4d::Identity();
	cur_target = Eigen::Matrix4d::Identity();
	target = Eigen::Matrix4d::Identity();
	file_path = "";

	connect(this, SIGNAL(showInformationSig(QString)), this, SLOT(showInformationSlot(QString)), Qt::QueuedConnection);
	connect(motion_module[0], SIGNAL(toggled(bool)), this, SLOT(clickBtnInverseProblemslot()));
	connect(motion_module[1], SIGNAL(toggled(bool)), this, SLOT(clickBtnForwardProblemslot()));
	connect(motion_module[2], SIGNAL(toggled(bool)), this, SLOT(clickBtnKinematicsslot()));
	connect(motion_module[3], SIGNAL(toggled(bool)), this, SLOT(clickBtnKinetostaticsslot()));
	connect(control_button[0], SIGNAL(clicked()), this, SLOT(clickBtnOpenDeviceslot()));
	connect(is_given_wrech, SIGNAL(toggled(bool)), this, SLOT(clickBtnWrenchSectionSlot()));
	connect(control_button[1], SIGNAL(clicked()), this, SLOT(getCurrentPoseSlot()));//confirmParamsSlot
	connect(control_button[2], SIGNAL(clicked()), this, SLOT(connectDevicesSlot()));
	connect(control_button[3], SIGNAL(clicked()), this, SLOT(getQafromUI()));//singleStepSlot
	connect(control_button[4], SIGNAL(clicked()), this, SLOT(clickBtnUpdateParamsSlot()));
	connect(control_button[5], SIGNAL(clicked()), this, SLOT(clickBtnStartCalculationSlot()));
	//connect(control_button[6], SIGNAL(clicked()), this, SLOT(getCurrentPoseSlot()));
	////
	connect(control_button[7], SIGNAL(clicked()), this, SLOT(clickBtnTestForwardKinetostaticsSlot()));
	////
	connect(control_button[8], SIGNAL(clicked()), this, SLOT(clickBtnTestInverseKinetostaticsSlot()));

	//connect(arm_psi_button[2][0], SIGNAL(clicked()), this, SLOT(stopInverseProblemsSlot()));
	//connect(arm_psi_button[2][1], SIGNAL(clicked()), this, SLOT(multipleHitInverseProblems2Slot()));
	//connect(inverse2Timer, SIGNAL(timeout()), this, SLOT(continueInverseProblemsSlot()));


	//connect(arm_psi_button[0][0], SIGNAL(clicked()), this, SLOT(connectArm1Slot()));
	//connect(arm_psi_button[0][1], SIGNAL(clicked()), this, SLOT(sendCommandToArm1Slot()));
	//connect(arm_psi_button[1][0], SIGNAL(clicked()), this, SLOT(connectArm2Slot()));
	//connect(arm_psi_button[1][1], SIGNAL(clicked()), this, SLOT(sendCommandToArm2Slot()));
	//connect(arm_psi_button[3][0], SIGNAL(clicked()), this, SLOT(connectArm4Slot()));
	//connect(arm_psi_button[3][1], SIGNAL(clicked()), this, SLOT(sendCommandToArm4Slot()));

	connect(displayTimer, SIGNAL(timeout()), this, SLOT(displayVideoSlot()));
	connect(commandTimer, SIGNAL(timeout()), this, SLOT(checkCommandSlot()));

	std::string exe_name = "start_all.bat";
	system(exe_name.c_str());
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

void DoM_pm::clickBtnOpenDeviceslot()
{
	QDir currentDir = QDir::current();
	QString file_log_name = num_lineEdit->text();
	log_file_path = currentDir.absolutePath() + "/conf/result/test_" + file_log_name;
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

	// 初始化手术工具、辅助工具、以及鞘管位姿参数
	clickBtnUpdateParamsSlot();

	// 打开鞘管、目标位姿等参数；
	confirmParamsSlot();

	getDeviceParamsSlot();
	//打开的所有设备与参考文件

	if (file_log_name[0] == 'n' && file_log_name[1] == 'c') {
	}
	else {
		cameraOpenClicked();
		control_button[1]->setEnabled(true);
	}
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
		//"<p>小写的ϕ: &varphi;</p>"
		//"<p style='font-family: Times New Roman, sans-serif;'>小写的φ</p>"
		//"<p style='font-family: Times New Roman, serif;'>小写的ϕ</p>"
		"</body>"
		"</html>";
	view_label = new QLabel("-.-", frame_box);
	view_label->setGeometry(10, 10, 800, 450);
	view_label->setText(labelText);
	view_label->setFont(font_EN);

	// 运动控制
	QGroupBox* motion_box = new QGroupBox("运动控制", this);
	motion_box->setGeometry(850, 340, 474, 400);
	motion_box->setFont(font_CH);
	int top_left_x = 22;
	int top_left_y = 30;
	int line_width = 80;
	int line_height = 28;
	labelText = "逆问题";
	motion_module[0] = new QRadioButton(labelText, motion_box);
	motion_module[0]->setGeometry(top_left_x, top_left_y, line_width, line_height);
	motion_module[0]->setFont(font_CH);
	labelText = "正问题";
	motion_module[1] = new QRadioButton(labelText, motion_box); top_left_x += 70;
	motion_module[1]->setGeometry(top_left_x, top_left_y, line_width, line_height);
	motion_module[1]->setFont(font_CH);
	labelText = "运动学";
	motion_module[2] = new QRadioButton(labelText, motion_box); top_left_x += 110;
	motion_module[2]->setGeometry(top_left_x, top_left_y, line_width, line_height);
	motion_module[2]->setFont(font_CH);
	labelText = "静力学";
	motion_module[3] = new QRadioButton(labelText, motion_box); top_left_x += 70;
	motion_module[3]->setGeometry(top_left_x, top_left_y, line_width, line_height);
	motion_module[3]->setFont(font_CH);
	is_given_wrech = new QCheckBox("力", motion_box); top_left_x += 70;
	is_given_wrech->setGeometry(top_left_x, top_left_y, line_width, line_height);
	is_given_wrech->setFont(font_CH);
	is_given_wrech->setEnabled(false);
	arm_num = new QComboBox(motion_box); top_left_x = 380;
	arm_num->addItem("ARM_1");
	arm_num->addItem("ARM_2");
	arm_num->addItem("ARM_3");
	//arm_num_lineEdit = new QLineEdit("", motion_box); top_left_x = 380;
	arm_num->setGeometry(top_left_x, top_left_y, line_width, line_height);
	arm_num->setFont(font_EN);
	//arm_num_lineEdit->setPlaceholderText("鞘管通道");
	//arm_num_lineEdit->setToolTip("仅接受int数字1，2，3或4中的一个");
	//arm_num_lineEdit->setText("1");
	QButtonGroup* group1 = new QButtonGroup(motion_box);
	group1->addButton(motion_module[0]); group1->addButton(motion_module[1]);
	motion_module[0]->setChecked(true);
	QButtonGroup* group2 = new QButtonGroup(motion_box);
	group2->addButton(motion_module[2]); group2->addButton(motion_module[3]);
	motion_module[2]->setChecked(true);
	top_left_y += 40; top_left_x = 22;

	auto lambda_button = [font_CH, &top_left_x, &top_left_y, &line_width, &line_height, this](QPushButton* button, QString labelText) {
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
	num_lineEdit = new QLineEdit("", motion_box);
	num_lineEdit->setGeometry(top_left_x, top_left_y, line_width, line_height);
	num_lineEdit->setAlignment(Qt::AlignCenter);
	num_lineEdit->setFont(font_EN);
	num_lineEdit->setPlaceholderText("测试名称");
	for (size_t i = 0; i < 9; i++)
		control_button[i] = new QPushButton("-.-", motion_box);
	labelText = "连接设备";
	top_left_x += 90;
	lambda_button(control_button[0], labelText);
	labelText = "当前坐标";
	top_left_x += 90;
	lambda_button(control_button[1], labelText); control_button[1]->setEnabled(false);
	labelText = "开始运动";
	top_left_x += 90;
	lambda_button(control_button[2], labelText);
	labelText = "清除日志";
	top_left_x += 90;
	lambda_button(control_button[3], labelText);
	top_left_y += 40; top_left_x = 22;

	labelText = "更新数据";
	lambda_button(control_button[4], labelText);
	//control_button[4]->setEnabled(false);
	labelText = "问题求解";
	top_left_x += 90;
	lambda_button(control_button[5], labelText);
	//control_button[5]->setEnabled(false);
	labelText = "闲置按钮";
	top_left_x += 90;
	lambda_button(control_button[6], labelText);
	control_button[6]->setEnabled(false);
	labelText = "正静力学";
	top_left_x += 90;
	lambda_button(control_button[7], labelText);
	//control_button[7]->setEnabled(false);
	labelText = "逆静力学";
	top_left_x += 90;
	lambda_button(control_button[8], labelText);
	//control_button[8]->setEnabled(false);
	top_left_y += 40; top_left_x = 22;

	QLabel* name_label[12];

	auto lambda = [motion_box, &name_label, &top_left_x, &top_left_y, &line_width, &line_height](int n, QString labelText) {
		name_label[n] = new QLabel("-.-", motion_box);
		name_label[n]->setGeometry(top_left_x, top_left_y, line_width, line_height);
		name_label[n]->setText(labelText);
	};

	line_width = 64;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;font-style: italic;'>l</p>" "< / body>""< / html>";
	lambda(0, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;font-style: italic;'>φ</p>" "< / body>""< / html>";
	lambda(1, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>θ</i><sub>1</sub></p>" "< / body>""< / html>";
	lambda(2, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>δ</i><sub>1</sub></p>" "< / body>""< / html>";
	lambda(3, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>θ</i><sub>2</sub></p>" "< / body>""< / html>";
	lambda(4, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>δ</i><sub>2</sub></p>" "< / body>""< / html>";
	lambda(5, labelText);
	top_left_y += 40; top_left_x = 22;

	for (size_t i = 0; i < 3; i++)
		for (size_t j = 0; j < 6; j++)
			psi_lineEdit[i][j] = new QLineEdit("-.-", motion_box);
	QString dimensions_varialbes1[] = { "mm", "degree", "degree", "degree", "degree", "degree", "degree", "sec" };
	for (size_t i = 0; i < 6; i++) {
		labelText = "0.000";
		lambda_lineEdit(psi_lineEdit[0][i], labelText, i);
		psi_lineEdit[0][i]->setPlaceholderText(dimensions_varialbes1[i]);
		psi_lineEdit[0][i]->setEnabled(false);
		top_left_x += 74;
	}
	psi_lineEdit[0][0]->setText("30.000");
	top_left_y += 40; top_left_x = 22;

	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>p<sub>x</sub></i></p>" "< / body>""< / html>";
	lambda(6, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>p<sub>y</sub></i></p>" "< / body>""< / html>";
	lambda(7, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>p<sub>z</sub></i></p>" "< / body>""< / html>";
	lambda(8, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>roll</i></p>" "< / body>""< / html>";
	lambda(9, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>pitch</i></p>" "< / body>""< / html>";
	lambda(10, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>yaw</i></p>" "< / body>""< / html>";
	lambda(11, labelText);
	top_left_y += 40; top_left_x = 22;

	QString dimensions_varialbes2[] = { "mm", "mm", "mm", "degree", "degree", "degree" };
	for (size_t i = 0; i < 6; i++) {
		labelText = "0.000";
		lambda_lineEdit(psi_lineEdit[1][i], labelText, i);
		psi_lineEdit[1][i]->setPlaceholderText(dimensions_varialbes2[i]);
		top_left_x += 74;
	}
	psi_lineEdit[1][2]->setText("30.000");
	top_left_y += 40; top_left_x = 22;

	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>F<sub>x</sub></i></p>" "< / body>""< / html>";
	lambda(6, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>F<sub>y</sub></i></p>" "< / body>""< / html>";
	lambda(7, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>F<sub>z</sub></i></p>" "< / body>""< / html>";
	lambda(8, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>M<sub>x</sub></i></p>" "< / body>""< / html>";
	lambda(9, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>M<sub>y</sub></i></p>" "< / body>""< / html>";
	lambda(10, labelText);
	top_left_x += 74;
	labelText = "<html>""<body>""<p style='text-align: center; font-family: Times New Roman, sans-serif;font-size: 12pt;'><i>M<sub>z</sub></i></p>" "< / body>""< / html>";
	lambda(11, labelText);
	top_left_y += 40; top_left_x = 22;

	QString dimensions_varialbes3[] = { "N", "N", "N", "Nmm", "Nmm", "Nmm" };
	for (size_t i = 0; i < 6; i++) {
		labelText = "0.000";
		lambda_lineEdit(psi_lineEdit[2][i], labelText, i);
		psi_lineEdit[2][i]->setPlaceholderText(dimensions_varialbes3[i]);
		psi_lineEdit[2][i]->setEnabled(false);
		top_left_x += 74;
	}

	// 工具参数
	QGroupBox* params_tool_box = new QGroupBox("手术工具参数设置", this);
	params_tool_box->setGeometry(850, 20, 474, 110);	//320
	params_tool_box->setFont(font_CH);
	top_left_x = 22;
	top_left_y = 30;
	line_width = 60;
	line_height = 30;
	//QLabel* name_label_params[15];

	for (size_t i = 0; i < 2; i++)
		for (size_t j = 0; j < 6; j++) {
			params_tool_lineEdit[i][j] = new QLineEdit("-.-", params_tool_box);
		}
	QString dimensions_varialbes4[] = { "mm", "mm", "mm", "mm", "无量纲", "无量纲" };
	QString dimensions_varialbes40[] = { "100.0", "10.0", "20.0", "15.0", "5.0", "0.6" };
	QString dimensions_varialbes41[] = { "手术工具第一柔性段的长度",
		"手术工具刚性段段的长度", "手术工具第二柔性段的长度",
		"手术工具从第二柔性段末端到啮合位置的长度",
		"手术工具第一柔性段的刚度参数",
		"手术工具第二柔性段的刚度参数" };
	for (size_t i = 0; i < 6; i++) {
		labelText = dimensions_varialbes40[i];
		lambda_lineEdit(params_tool_lineEdit[0][i], labelText, i);
		params_tool_lineEdit[0][i]->setPlaceholderText(dimensions_varialbes4[i]);
		params_tool_lineEdit[0][i]->setToolTip(dimensions_varialbes41[i]);
		top_left_x += 74;
	}
	top_left_y += 40; top_left_x = 22;
	QString dimensions_varialbes5[] = { "mm", "无量纲", "Pa", "×××", "×××", "×××" };
	QString dimensions_varialbes50[] = { "600.0", "0.1", "50e9", "-", "-", "-" };
	QString dimensions_varialbes51[] = { "手术工具半刚性段的长度",
		"手术工具半刚性段与第一柔性段刚度的关系", "镍钛合金丝的弹性模量",
		"",
		"",
		"" };
	for (size_t i = 0; i < 6; i++) {
		labelText = dimensions_varialbes50[i];
		lambda_lineEdit(params_tool_lineEdit[1][i], labelText, i);
		params_tool_lineEdit[1][i]->setPlaceholderText(dimensions_varialbes5[i]);
		params_tool_lineEdit[1][i]->setToolTip(dimensions_varialbes51[i]);
		top_left_x += 74;
		if (i > 2)
			params_tool_lineEdit[1][i]->setEnabled(false);
	}

	// 辅助孔参数
	QGroupBox* params_auxiliary_port_box = new QGroupBox("辅助超声刀参数设置", this);
	params_auxiliary_port_box->setGeometry(850, 140, 474, 70);	//320
	params_auxiliary_port_box->setFont(font_CH);
	top_left_x = 22;
	top_left_y = 30;
	line_width = 60;
	line_height = 30;

	for (size_t j = 0; j < 6; j++) {
		params_auxiliary_port_lineEdit[j] = new QLineEdit("-.-", params_auxiliary_port_box);
	}
	QString dimensions_varialbes6[] = { "N", "mm", "N", "mm", "无量纲", "无量纲" };
	QString dimensions_varialbes60[] = { "2.0", "500.0", "4.0", "600.0", "0.0", "0.0" };
	QString dimensions_varialbes61[] = { "相机坐标系下，辅助鞘管的重量",
		"相机坐标系下，辅助鞘管重心与啮合位置之间的距离", "相机坐标系下，超声刀的重量",
		"相机坐标系下，超声刀重心与啮合位置之间的距离",
		"相机坐标系下，重力方向向量的X值",
		"相机坐标系下，重力方向向量的Y值" };
	for (size_t i = 0; i < 6; i++) {
		labelText = dimensions_varialbes60[i];
		lambda_lineEdit(params_auxiliary_port_lineEdit[i], labelText, i);
		params_auxiliary_port_lineEdit[i]->setPlaceholderText(dimensions_varialbes6[i]);
		params_auxiliary_port_lineEdit[i]->setToolTip(dimensions_varialbes61[i]);
		top_left_x += 74;
	}

	// 鞘管位姿参数
	QGroupBox* params_trocar_pose_box = new QGroupBox("鞘管位姿参数设置", this);
	params_trocar_pose_box->setGeometry(850, 220, 474, 110);	//320
	params_trocar_pose_box->setFont(font_CH);
	top_left_x = 22;
	top_left_y = 30;
	line_width = 60;
	line_height = 30;
	for (size_t i = 0; i < 2; i++)
		for (size_t j = 0; j < 6; j++) {
			params_trocar_pose_lineEdit[i][j] = new QLineEdit("-.-", params_trocar_pose_box);
		}
	QString dimensions_varialbes7[] = { "mm", "mm", "mm", "无量纲", "无量纲", "无量纲" };
	QString dimensions_varialbes70[] = { "0.0", "0.0", "0.0", "0.0", "0.0", "0.0" };
	QString dimensions_varialbes71[] = { "相机坐标系下，选定工具鞘管出口的X坐标",
		"相机坐标系下，选定工具鞘管出口的Y坐标", "相机坐标系下，选定工具鞘管出口的Z坐标",
		"相机坐标系下，选定工具鞘管出口的姿态轴角表示后轴(3X1单位向量)角(degree)相乘后的第一位",
		"相机坐标系下，选定工具鞘管出口的姿态轴角表示后轴(3X1单位向量)角(degree)相乘后的第二位",
		"相机坐标系下，选定工具鞘管出口的姿态轴角表示后轴(3X1单位向量)角(degree)相乘后的第三位" };
	for (size_t i = 0; i < 6; i++) {
		labelText = dimensions_varialbes70[i];
		lambda_lineEdit(params_trocar_pose_lineEdit[0][i], labelText, i);
		params_trocar_pose_lineEdit[0][i]->setPlaceholderText(dimensions_varialbes7[i]);
		params_trocar_pose_lineEdit[0][i]->setToolTip(dimensions_varialbes71[i]);
		top_left_x += 74;
	}
	top_left_y += 40; top_left_x = 22;
	QString dimensions_varialbes72[] = { "相机坐标系下，辅助鞘管出口的X坐标",
		"相机坐标系下，辅助鞘管出口的Y坐标", "相机坐标系下，选定工具鞘管出口的Z坐标",
		"相机坐标系下，辅助鞘管出口的姿态轴角表示后轴(3X1单位向量)角(degree)相乘后的第一位",
		"相机坐标系下，辅助鞘管出口的姿态轴角表示后轴(3X1单位向量)角(degree)相乘后的第二位",
		"相机坐标系下，辅助鞘管出口的姿态轴角表示后轴(3X1单位向量)角(degree)相乘后的第三位" };
	for (size_t i = 0; i < 6; i++) {
		labelText = dimensions_varialbes70[i];
		lambda_lineEdit(params_trocar_pose_lineEdit[1][i], labelText, i);
		params_trocar_pose_lineEdit[1][i]->setPlaceholderText(dimensions_varialbes7[i]);
		params_trocar_pose_lineEdit[1][i]->setToolTip(dimensions_varialbes72[i]);
		top_left_x += 74;
	}





	//params_tool_lineEdit[0][1]->setToolTip("IRUS的总长度，mm");
	//params_tool_lineEdit[1][1]->setToolTip("IRUS上marker原点位置与啮合位置之间的距离的长度，mm");
	//params_tool_lineEdit[2][1]->setToolTip("IRUS的重心位置与啮合位置之间的距离，mm");
	//params_tool_lineEdit[3][1]->setToolTip("IRUS的总重量，N");
	//params_tool_lineEdit[4][1]->setToolTip("腹腔受力方向，X方向");
	//params_tool_lineEdit[5][1]->setToolTip("腹腔受力方向，Y方向");

	//auto lambda_params = [params_box, &name_label_params, &top_left_x, &top_left_y, &line_width, &line_height](int n, QString labelText) {
	//	name_label_params[n] = new QLabel("-.-", params_box);
	//	name_label_params[n]->setGeometry(top_left_x, top_left_y, line_width, line_height);
	//	name_label_params[n]->setText(labelText);
	//};

	// 默认参数
	QGroupBox* log_box = new QGroupBox("日志信息", this);
	log_box->setGeometry(10, 500, 820, 240);
	log_box->setFont(font_CH);
	textBrowser = new QTextBrowser(log_box);
	textBrowser->setGeometry(10, 30, 800, 200);
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

}

void DoM_pm::connectSecondProgramme()
{

}

std::vector<double> DoM_pm::getPose()
{
	std::vector<double> res;
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
	
}
void DoM_pm::openSecondPrograme()
{
	
}

void DoM_pm::checkCommandSlot() {
	
}

void DoM_pm::PushButtonClearLogSlot()
{
	textBrowser->clear();

	//QString str = num_lineEdit->text();
	//QChar lastChar = str.at(str.length() - 1);
	//int lastDigit = lastChar.digitValue();
	//if (lastDigit % 5 == 0){
	//	std::string filePath = "conf/public/psi.log"; // 替换为实际的日志文件路径

	//	std::vector<Eigen::Vector6d> qa;

	//	// 打开日志文件
	//	std::ifstream file(filePath);
	//	if (!file.is_open()) {
	//		std::cout << "无法打开日志文件 " << filePath << std::endl;
	//		return;
	//	}

	//	// 读取前三行非空文本
	//	std::vector<std::string> lines;
	//	std::string line;
	//	int lineCount = 0;
	//	while (getline(file, line) && lineCount < 3) {
	//		if (!line.empty()) {
	//			lines.push_back(line);
	//			lineCount++;
	//		}
	//	}

	//	// 解析每行文本中的数字
	//	for (const std::string& line : lines) {
	//		std::stringstream ss(line);
	//		Eigen::Vector6d numbers;
	//		double number;
	//		int numberCount = 0;
	//		while (ss >> number) {
	//			numbers[numberCount] = number;
	//			numberCount++;
	//			if (numberCount >= 6)
	//				break;
	//		}
	//		qa.push_back(numbers);
	//		if (numberCount == 6) {
	//			std::cout << "解析成功，数字为: ";
	//			for (int ii = 0; ii < numbers.size(); ii++) {
	//				std::cout << numbers[ii] << " ";
	//			}
	//			std::cout << std::endl;
	//		}
	//		else {
	//			std::cout << "解析失败，每行文本不包含六个数字" << std::endl;
	//		}
	//	}

	//	// 关闭文件
	//	file.close();
	//	//setQaToUI(qa);
	//}

}

void DoM_pm::sendCommandToArm1Slot()
{
	////Eigen::Vector6d psi = getUIPsi();
	//Eigen::Vector6d qa = Eigen::Vector6d::Zero();
	//Eigen::Vector6d psi1 = getPsiFromTrocar(qa, 0);
	//float psi[6];
	//for (size_t i = 0; i < 6; i++)
	//	psi[i] = (float)psi1[i];
	//clientMotionPara_Arm1.send((S8*)&psi, sizeof(psi));
}
void DoM_pm::sendCommandToArm2Slot()
{
	//std::vector<Eigen::Vector6d> qa = getQafromUI();
	//Eigen::Vector6d psi1 = getPsiFromTrocar(qa[1], 1);
	//float psi[6];
	//for (size_t i = 0; i < 6; i++)
	//	psi[i] = (float)psi1[i];
	//clientMotionPara_Arm2.send((S8*)&psi, sizeof(psi));
}
void DoM_pm::sendCommandToArm4Slot()
{
	//std::vector<Eigen::Vector6d> qa = getQafromUI();
	//Eigen::Vector6d psi1 = getPsiFromTrocar(qa[2], 2);
	//float psi[6];
	//for (size_t i = 0; i < 6; i++)
	//	psi[i] = (float)psi1[i];
	//bool rc = clientMotionPara_Arm4.send((S8*)&psi, sizeof(psi));
}

void DoM_pm::confirmParamsSlot() {

	setTrocar();		// 经过trocar的psi
	//getTrocarBase();	// trocar出口位姿
}

void DoM_pm::getDeviceParamsSlot() {
	commandTimer->setSingleShot(true);
	commandTimer->start(15);
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

	psi_trocar[3] -= psi_main[1] * 180.0 / Keith::PI;//?

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

}

void DoM_pm::connectDevicesSlot() {
	//static int pic_num = 0;
	//std::string num = std::to_string(pic_num++);
	//std::ostringstream oss;
	//oss << std::setfill('0') << std::setw(3) << num;
	//std::string file_name = log_file_path.toStdString() + "/M" + oss.str() + ".jpg";
	//cv::imwrite(file_name, frame);
	//if (is_new) {
	//	pic_num = 0;
	//	is_new = false;
	//}
	//getCurrentPoseSlot();

	//Eigen::Vector6d psi = getUIPsi();
	//Eigen::Vector3d L12zeta = diff_method->manipulators->getL12Zeta();
	//diff_method->manipulators->setLso(psi(0));

	//Eigen::Vector6d qa = Keith::Psi2Actuation_keith(psi, L12zeta, diff_method->manipulators->getGc());
	//Eigen::Vector6d psi1;
	//float psi0[6];
	//switch (arm_num->currentIndex()) {
	//case 0:
	//	psi1 = getPsiFromTrocar(qa, 0);
	//	break;
	//case 1:
	//	psi1 = getPsiFromTrocar(qa, 1);
	//	break;
	//case 2:
	//	psi1 = getPsiFromTrocar(qa, 2);
	//	break;
	//}
	//for (size_t i = 0; i < 6; i++)
	//	psi0[i] = (float)psi1[i];
	//clientMotionPara_Arm1.send((S8*)&psi0, sizeof(psi));
}

void DoM_pm::getTissueParamsSlot() {

}

void DoM_pm::connectArm1Slot() {



}

void DoM_pm::connectArm2Slot() {
	
}

void DoM_pm::connectArm4Slot() {


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
				params_tool_lineEdit[j][i * 2]->setText(QString::number(trocar_base[6 * i + j], 'f', 3));
			}
		}
	}

}
void DoM_pm::singleStepSlot() {
	getCurrentPoseSlot();
	connectDevicesSlot();


}

void DoM_pm::singleHitInverseProblems2Slot() {
	//getCurrentPoseSlot();
	//connectDevicesSlot();
}
void DoM_pm::multipleHitInverseProblems2Slot() {
	inverse2Timer->start(5000);
}
void DoM_pm::stopInverseProblemsSlot() {
	inverse2Timer->stop();
}

void DoM_pm::continueInverseProblemsSlot() {
	singleStepSlot();
}

void DoM_pm::clickBtnInverseProblemslot() {
	if (motion_module[2]->isChecked()) {	//kinematics
		solver = Solver::INVERSE_KINEMATICS;
		for (size_t i = 0; i < 6; i++) {
			psi_lineEdit[0][i]->setEnabled(false);
			psi_lineEdit[1][i]->setEnabled(true);
			psi_lineEdit[2][i]->setEnabled(false);
		}
	}
	else {
		solver = Solver::INVERSE_KINETOSTATICS;
		for (size_t i = 0; i < 6; i++) {
			psi_lineEdit[0][i]->setEnabled(false);
			psi_lineEdit[1][i]->setEnabled(true);
			clickBtnWrenchSectionSlot();
		}
	}
}

void DoM_pm::clickBtnForwardProblemslot() {
	if (motion_module[2]->isChecked()) {	//kinematics
		solver = Solver::FORWARD_KINEMATICS;
		for (size_t i = 0; i < 6; i++) {
			psi_lineEdit[0][i]->setEnabled(true);
			psi_lineEdit[1][i]->setEnabled(false);
			psi_lineEdit[2][i]->setEnabled(false);
		}
	}
	else {
		solver = Solver::FORWARD_KINETOSTATICS;
		for (size_t i = 0; i < 6; i++) {
			psi_lineEdit[0][i]->setEnabled(true);
			psi_lineEdit[1][i]->setEnabled(false);
			clickBtnWrenchSectionSlot();
		}
	}
}

void DoM_pm::clickBtnKinematicsslot() {
	if (motion_module[0]->isChecked()) {
		solver = Solver::INVERSE_KINEMATICS;
		for (size_t i = 0; i < 6; i++) {
			psi_lineEdit[0][i]->setEnabled(false);
			psi_lineEdit[1][i]->setEnabled(true);
			psi_lineEdit[2][i]->setEnabled(false);
		}
	}
	else {
		solver = Solver::FORWARD_KINEMATICS;
		for (size_t i = 0; i < 6; i++) {
			psi_lineEdit[0][i]->setEnabled(true);
			psi_lineEdit[1][i]->setEnabled(false);
			psi_lineEdit[2][i]->setEnabled(false);
		}
	}
	is_given_wrech->setEnabled(false);
}

void DoM_pm::clickBtnKinetostaticsslot() {
	if (motion_module[0]->isChecked()) {
		solver = Solver::INVERSE_KINETOSTATICS;
		for (size_t i = 0; i < 6; i++) {
			psi_lineEdit[0][i]->setEnabled(false);
			psi_lineEdit[1][i]->setEnabled(true);
		}
	}
	else {
		solver = Solver::FORWARD_KINETOSTATICS;
		for (size_t i = 0; i < 6; i++) {
			psi_lineEdit[0][i]->setEnabled(true);
			psi_lineEdit[1][i]->setEnabled(false);
		}
	}
	is_given_wrech->setEnabled(true);
	clickBtnWrenchSectionSlot();
}

Eigen::Matrix4d DoM_pm::calcForwardKinematics(const Eigen::Vector6d psi) {
	Eigen::Vector4d L1r2g = diff_method->manipulators->getL1r2g();
	double zeta = diff_method->manipulators->getZeta();
	Eigen::Matrix4d mat = Keith::getForwardKinematics(psi, L1r2g, zeta);
	setUIPose(mat);
	return mat;
}

Eigen::Vector6d DoM_pm::calcInverseKinematics(const Eigen::Matrix4d target) {
	Eigen::Vector4d L1r2g = diff_method->manipulators->getL1r2g();
	double zeta = diff_method->manipulators->getZeta();
	Eigen::Vector6d psi = getUIPsi();
	double residue(0.0);
	psi = Keith::getInverseKinematics(target, L1r2g, zeta, psi, residue);
	if (residue > 1e-6)
		showInformation("错误::逆运动学求解残差为：" + QString::number(residue, 'f', 8) + "。"
			, InfoSymbol::INFO_ERROR);
	setUIPsi(psi);
	return psi;
}

Eigen::Matrix4d DoM_pm::calcForwardKinetostatics(const Eigen::Vector6d psi) {
	Eigen::Vector10d guess0, Rsd0;
	guess0 = guess.head(10);
	Rsd0 = Rsd_last.head(10);
	Eigen::Vector3d L12zeta = diff_method->manipulators->getL12Zeta();
	diff_method->manipulators->setLso(psi[0]);
	Eigen::Matrix46d Gc = diff_method->manipulators->getGc();
	Eigen::Vector6d qa = Keith::Psi2Actuation_keith(psi, L12zeta, Gc);
	qa(0) = qa(0) * 1e-3; //qa.tail(4) = qa.tail(4)*1e-3;
	Eigen::Matrix4d target;
	Eigen::Vector6d wrench = calcExternalWrench();
	diff_method->shootForwardOptimization(guess0, Rsd0, qa, target, wrench);
	setUIWrench(wrench);
	double err = Rsd0.norm();
	if (err > 1e-6)
		showInformation("错误::正静力学求解残差为：" + QString::number(err, 'f', 8) + "。"
			, InfoSymbol::INFO_ERROR);
	target.topRightCorner(3, 1) = target.topRightCorner(3, 1) * 1e3;
	setUIPose(target);
	return target;
}

Eigen::Vector6d DoM_pm::calcInverseKinetostatics(const Eigen::Matrix4d target) {
	Eigen::Vector6d psi;
	Eigen::Vector3d L12zeta = diff_method->manipulators->getL12Zeta();
	diff_method->manipulators->setLso(psi[0]);
	Eigen::Matrix46d Gc = diff_method->manipulators->getGc();
	Eigen::Vector6d qa = Keith::Psi2Actuation_keith(psi, L12zeta, Gc);
	qa(0) = qa(0) * 1e-3;
	Eigen::Vector6d wrench = calcExternalWrench();
	diff_method->shootInverseOptimization(guess, Rsd_last, qa, target, wrench);
	double err = Rsd_last.norm();
	if (err > 1e-6)
		showInformation("错误::逆静力学求解残差为：" + QString::number(err, 'f', 8) + "。"
			, InfoSymbol::INFO_ERROR);
	L12zeta = diff_method->manipulators->getL12Zeta();
	diff_method->manipulators->setLso(qa[0]);
	Gc = diff_method->manipulators->getGc();
	psi = Keith::Actuation2Psi_keith(qa, L12zeta, Gc);
	setUIPsi(psi);
	return psi;
}

Eigen::Vector6d DoM_pm::calcExternalWrench() {
	Eigen::Vector6d wrench = Eigen::Vector6d::Zero();
	if (is_given_wrech->isChecked()) {
		wrench = getUIWrench();
		diff_method->is_given_wrench = true;
	}
	else {
		Eigen::Matrix4d tar = getUIPose();
		diff_method->is_given_wrench = false;
		wrench = diff_method->getGripperWrench(tar, wrench, 0);
	}
	return wrench;
}

void DoM_pm::clickBtnStartCalculationSlot() {
	QString str;
	Eigen::Vector6d psi;
	Eigen::Matrix4d target;
	Eigen::Matrix3d rot;
	Eigen::Vector3d vec;
	switch (solver) {
	case Solver::NONE_SOLVER:
		showInformation("错误::未确认/更新参数");
		break;
	case Solver::FORWARD_KINEMATICS:
		psi = getUIPsi();
		target = calcForwardKinematics(psi);
		rot = target.topLeftCorner(3, 3);
		vec = rot.eulerAngles(2, 1, 0);
		str = "正运动学计算结果的位置为：[" + QString::number(target(0, 3), 'f', 3) + " " +
			QString::number(target(1, 3), 'f', 3) + " " + QString::number(target(2, 3), 'f', 3) +
			"]，姿态对应的欧拉角(rpy)为：[" + QString::number(vec(0), 'f', 3) + " " +
			QString::number(vec(1), 'f', 3) + " " + QString::number(vec(2), 'f', 3) + "]。";
		break;
	case Solver::INVERSE_KINEMATICS:
		target = getUIPose();
		psi = calcInverseKinematics(target);
		str = "逆运动学计算结果的构型空间变量为：[" + QString::number(psi(0), 'f', 3) + " " +
			QString::number(psi(1), 'f', 3) + " " + QString::number(psi(2), 'f', 3) + " " +
			QString::number(psi(3), 'f', 3) + " " + QString::number(psi(4), 'f', 3) + " " +
			QString::number(psi(5), 'f', 3) + "]。";
		break;
	case Solver::FORWARD_KINETOSTATICS:
		psi = getUIPsi();
		target = calcForwardKinetostatics(psi);
		rot = target.topLeftCorner(3, 3);
		vec = rot.eulerAngles(2, 1, 0);
		str = "正静力学计算结果的位置为：[" + QString::number(target(0, 3), 'f', 3) + " " +
			QString::number(target(1, 3), 'f', 3) + " " + QString::number(target(2, 3), 'f', 3) +
			"]，姿态对应的欧拉角(rpy)为：[" + QString::number(vec(0), 'f', 3) + " " +
			QString::number(vec(1), 'f', 3) + " " + QString::number(vec(2), 'f', 3) + "]。";
		break;
	case Solver::INVERSE_KINETOSTATICS:
		target = getUIPose();
		target.topRightCorner(3, 1) = target.topRightCorner(3, 1) * 1e-3;
		psi = calcInverseKinetostatics(target);
		str = "逆运动学计算结果的构型空间变量为：[" + QString::number(psi(0), 'f', 3) + " " +
			QString::number(psi(1), 'f', 3) + " " + QString::number(psi(2), 'f', 3) + " " +
			QString::number(psi(3), 'f', 3) + " " + QString::number(psi(4), 'f', 3) + " " +
			QString::number(psi(5), 'f', 3) + "]。";
		break;
	}
	showInformation(str);
}

int DoM_pm::convertAndSet(QLineEdit* lineEdit, const double default_value, double& result) {
	bool rc_ok = false;
	double number = lineEdit->text().toDouble(&rc_ok);
	if (rc_ok) {
		result = number;
		return 1;
	}
	else {
		result = default_value;
		//showInformation("错误::填写参数",InfoSymbol::INFO_ERROR);
		return 0;
	}
}

void DoM_pm::clickBtnUpdateParamsSlot() {
	// 初始化手术工具参数
	int x_inc = 0;
	Eigen::Vector10d SL;
	double elastic_module(0.0);
	int flag = convertAndSet(params_tool_lineEdit[0][0], 100.0, SL[0]);
	flag += convertAndSet(params_tool_lineEdit[0][1], 10.0, SL[1]);
	flag += convertAndSet(params_tool_lineEdit[0][2], 20.0, SL[2]);
	flag += convertAndSet(params_tool_lineEdit[0][3], 15.0, SL[3]);
	flag += convertAndSet(params_tool_lineEdit[1][1], 0.1, SL[4]);
	flag += convertAndSet(params_tool_lineEdit[0][4], 5.0, SL[5]);
	flag += convertAndSet(params_tool_lineEdit[0][5], 0.6, SL[6]);
	flag += convertAndSet(params_tool_lineEdit[1][0], 600.0, SL[8]);
	flag += convertAndSet(params_tool_lineEdit[1][2], 30e9, elastic_module);
	SL[7] = 0.0; SL[9] = 0.0;
	if (9 == flag) {
		x_inc++;
	}
	else {
		showInformation("错误::填写手术工具参数", InfoSymbol::INFO_ERROR);
	}
	diff_method->manipulators->setCalibrationPara(SL);
	diff_method->manipulators->setE(elastic_module);


	// 初始化鞘管参数
	flag = 0;
	double tem;
	Eigen::Vector3d gravity_direction;
	flag += convertAndSet(params_auxiliary_port_lineEdit[0], 2.0, tem);
	diff_method->setIRUSAuxiliarySheathWeight(tem);
	flag += convertAndSet(params_auxiliary_port_lineEdit[1], 500.0, tem);
	diff_method->setIRUSAuxiliarySheathBarycenterLength(tem);
	flag += convertAndSet(params_auxiliary_port_lineEdit[2], 4.0, tem);
	diff_method->setIRUSWeight(tem);
	flag += convertAndSet(params_auxiliary_port_lineEdit[3], 600.0, tem);
	diff_method->setIRUSBarycenterLength(tem);
	flag += convertAndSet(params_auxiliary_port_lineEdit[4], 0.0, gravity_direction[0]);
	flag += convertAndSet(params_auxiliary_port_lineEdit[5], 0.0, gravity_direction[1]);
	gravity_direction[2] = 1.0 - std::pow((std::pow(gravity_direction[0], 2) +
		std::pow(gravity_direction[1], 2)), 0.5);
	diff_method->setGravityDirection(gravity_direction);

	if (6 == flag) {
		x_inc++;
	}
	else {
		showInformation("错误::填写辅助工具参数", InfoSymbol::INFO_ERROR);
	}
	// 初始化出口位姿参数
	flag = 0;
	Eigen::Vector12d paras;
	flag += convertAndSet(params_trocar_pose_lineEdit[0][0], 0.0, paras[0]);
	flag += convertAndSet(params_trocar_pose_lineEdit[0][1], 0.0, paras[1]);
	flag += convertAndSet(params_trocar_pose_lineEdit[0][2], 0.0, paras[2]);
	flag += convertAndSet(params_trocar_pose_lineEdit[0][3], 0.0, paras[3]);
	flag += convertAndSet(params_trocar_pose_lineEdit[0][4], 0.0, paras[4]);
	flag += convertAndSet(params_trocar_pose_lineEdit[0][5], 0.0, paras[5]);
	flag += convertAndSet(params_trocar_pose_lineEdit[1][0], 0.0, paras[6]);
	flag += convertAndSet(params_trocar_pose_lineEdit[1][1], 0.0, paras[7]);
	flag += convertAndSet(params_trocar_pose_lineEdit[1][2], 0.0, paras[8]);
	flag += convertAndSet(params_trocar_pose_lineEdit[1][3], 0.0, paras[9]);
	flag += convertAndSet(params_trocar_pose_lineEdit[1][4], 0.0, paras[10]);
	flag += convertAndSet(params_trocar_pose_lineEdit[1][5], 0.0, paras[11]);
	diff_method->setParams(paras);

	if (12 == flag) {
		x_inc++;
	}
	else {
		showInformation("错误::填写辅助工具参数", InfoSymbol::INFO_ERROR);
	}
	if (3 == x_inc)
		showInformation("成功::更新手术工具、辅助工具和鞘管位姿参数", InfoSymbol::INFO_SUCCESS);
}

void DoM_pm::setUIPsi(const Eigen::Vector6d psi) {
	Eigen::Vector6d revised_psi(psi);
	while (revised_psi[1] > 180.0) {
		revised_psi[1] -= 360.0;
	}
	while (revised_psi[1] < -180.0) {
		revised_psi[1] += 360.0;
	}
	if (revised_psi[2] < 0) {
		revised_psi[2] = -revised_psi[2];
		revised_psi[3] = 180.0 + revised_psi[3];
	}
	if (revised_psi[4] < 0) {
		revised_psi[4] = -revised_psi[4];
		revised_psi[5] = 180.0 + revised_psi[5];
	}
	while (revised_psi[3] > 180.0) {
		revised_psi[3] -= 360.0;
	}
	while (revised_psi[3] < -180.0) {
		revised_psi[3] += 360.0;
	}
	while (revised_psi[5] > 180.0) {
		revised_psi[5] -= 360.0;
	}
	while (revised_psi[5] < -180.0) {
		revised_psi[5] += 360.0;
	}
	for (size_t i = 0; i < 6; i++)
		psi_lineEdit[0][i]->setText(QString::number(revised_psi[i], 'f', 3));
}
void DoM_pm::setUIWrench(const Eigen::Vector6d wrench) {
	for (size_t i = 0; i < 6; i++)
		psi_lineEdit[2][i]->setText(QString::number(wrench[i], 'f', 3));
}
void DoM_pm::setUIPose(const Eigen::Matrix4d pose) {
	Eigen::Vector6d posev6;
	Eigen::Matrix3d rot = pose.topLeftCorner(3, 3);
	Eigen::Vector3d vec = rot.eulerAngles(2, 1, 0);
	posev6.head(3) = pose.topRightCorner(3, 1);
	posev6.tail(3) = vec;
	for (size_t i = 0; i < 6; i++)
		psi_lineEdit[1][i]->setText(QString::number(posev6[i], 'f', 3));
}
Eigen::Vector6d DoM_pm::getUIPsi() {
	Eigen::Vector6d psi;
	for (size_t i = 0; i < 6; i++)
		psi[i] = psi_lineEdit[0][i]->text().toDouble();
	return psi;
}
Eigen::Vector6d DoM_pm::getUIWrench() {
	Eigen::Vector6d force;
	for (size_t i = 0; i < 6; i++)
		force[i] = psi_lineEdit[2][i]->text().toDouble();
	return force;
}
Eigen::Matrix4d DoM_pm::getUIPose() {
	Eigen::Vector6d posev6;
	for (size_t i = 0; i < 6; i++)
		posev6[i] = psi_lineEdit[1][i]->text().toDouble();
	Eigen::Matrix3d rot = Keith::rpy2Rotation(posev6.tail(3));
	Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
	mat.topLeftCorner(3, 3) = rot;
	mat.topRightCorner(3, 1) = posev6.head(3);
	std::cout << mat << std::endl;
	return mat;
}

void DoM_pm::clickBtnWrenchSectionSlot() {
	if (is_given_wrech->isChecked()) {
		for (size_t i = 0; i < 6; i++)
			psi_lineEdit[2][i]->setEnabled(true);
	}
	else {
		for (size_t i = 0; i < 6; i++)
			psi_lineEdit[2][i]->setEnabled(false);
	}
}

void DoM_pm::clickBtnTestInverseKinetostaticsSlot()
{
	Eigen::Vector12d guess0(guess), Rsd0;
	Eigen::Vector6d psi, tar,qa;
	Eigen::Vector3d L12zeta = diff_method->manipulators->getL12Zeta();
	Eigen::Matrix46d Gc;
	Eigen::Matrix4d target;
	std::ifstream infile("./conf/private/input_inv.log"); // 替换为你的 log 文件名
	std::vector<std::vector<double>> data;

	if (!infile) {
		std::cerr << "cannot open file input.log!" << std::endl;
		return;
	}

	std::string line;
	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		std::vector<double> row(6); // 创建一个包含6个元素的行

		// 读取6个数字到row中
		for (size_t i = 0; i < 6; ++i) {
			if (!(iss >> row[i])) {
				std::cerr << "data style error in line : " << data.size() + 1 << std::endl;
				return; // 读取失败，退出程序
			}
		}

		data.push_back(row); // 将当前行添加到数据中
	}

	infile.close(); // 关闭文件


	// 打开文件以写入数据
	std::ofstream outfile("./conf/private/output_inv.log"); // 替换为你想要的文件名
	if (!outfile) {
		std::cerr << "cannot open file output.log!" << std::endl;
		return;
	}
	std::vector<std::vector<double>> output_data(100000, std::vector<double>(8));

	int block_size = data.size();

	Eigen::Vector8d resu;
	double err(10.0);
	float cost_time(10.0);
	psi << 55.0, 0.0, 0.0, 0.0, 0.0, 0.0;
	//static Eigen::Vector6d qaa = psi;
	
	Eigen::Vector6d wrench = calcExternalWrench();
	for (size_t j = 19; j < block_size; j++) {
		for (size_t i = 0; i < 6; i++)
			tar[i] = data[j][i];
		target = Keith::fromX2T(tar);
		target.topRightCorner(3, 1) *= 1e-3;
		std::cout << target << std::endl;
		//diff_method->manipulators->setLso(feed_length);
		//Gc = diff_method->manipulators->getGc();
		//qa = Keith::Psi2Actuation_keith(psi, L12zeta, Gc);
		//qa(0) = qa(0) * 1e-3;
		auto tic = mmath::timer::getCurrentTimePoint();
		diff_method->shootInverseOptimization(guess, Rsd0, qa, target, wrench);
		cost_time = mmath::timer::getDurationSince(tic);
		//guess = guess0;
		err = Rsd0.norm();
		target.topRightCorner(3, 1) = 1000 * target.topRightCorner(3, 1);
		//tar = Keith::fromT2X(target);
		resu.head(6) = qa;
		resu[6] = err;
		resu[7] = cost_time;

		for (size_t m = 0; m < resu.size(); ++m) {
			outfile << resu[m];
			if (m < resu.size() - 1) {
				outfile << " "; // 在数字之间添加空格
			}
		}
		outfile << std::endl; // 每行写入后换行
	}
	outfile.close(); // 关闭文件
}

void DoM_pm::clickBtnTestForwardKinetostaticsSlot()
{
	Eigen::Vector10d guess0, Rsd0;
	Eigen::Vector6d psi, tar;
	guess0 = guess.head(10);
	Rsd0 = Rsd_last.head(10);
	Eigen::Vector3d L12zeta = diff_method->manipulators->getL12Zeta();
	diff_method->manipulators->setLso(psi[0]);
	Eigen::Matrix46d Gc = diff_method->manipulators->getGc();
	Eigen::Vector6d qa = Keith::Psi2Actuation_keith(psi, L12zeta, Gc);
	qa(0) = qa(0) * 1e-3; //qa.tail(4) = qa.tail(4)*1e-3;
	Eigen::Matrix4d target;
	Eigen::Vector6d wrench = calcExternalWrench();

	std::ifstream infile("./conf/private/input_inv.log"); // 替换为你的 log 文件名
	std::vector<std::vector<double>> data;

	if (!infile) {
		std::cerr << "cannot open file input_inv.log!" << std::endl;
		return;
	}

	std::string line;
	while (std::getline(infile, line)) {
		std::istringstream iss(line);
		std::vector<double> row(6); // 创建一个包含6个元素的行

		// 读取6个数字到row中
		for (size_t i = 0; i < 6; ++i) {
			if (!(iss >> row[i])) {
				std::cerr << "data style error in line : " << data.size() + 1 << std::endl;
				return; // 读取失败，退出程序
			}
		}

		data.push_back(row); // 将当前行添加到数据中
	}

	infile.close(); // 关闭文件


	// 打开文件以写入数据
	std::ofstream outfile("./conf/private/output_inv.log"); // 替换为你想要的文件名
	if (!outfile) {
		std::cerr << "cannot open file output_inv.log!" << std::endl;
		return;
	}
	std::vector<std::vector<double>> output_data(100000, std::vector<double>(8));

	int block_size = data.size();
	
	Eigen::Vector8d resu;
	double err(10.0);
	float cost_time(10.0);
	for (size_t j = 0; j < block_size; j++) {
		guess0 = guess.head(10);
		for (size_t i = 0; i < 6; i++)
			psi[i] = data[j][i];
		diff_method->manipulators->setLso(psi[0]);
		Gc = diff_method->manipulators->getGc();
		qa = Keith::Psi2Actuation_keith(psi, L12zeta, Gc);
		qa(0) = qa(0) * 1e-3;
		auto tic = mmath::timer::getCurrentTimePoint();
		diff_method->shootForwardOptimization(guess0, Rsd0, qa, target, wrench);
		cost_time = mmath::timer::getDurationSince(tic);
		guess.head(10) = guess0;
		err = Rsd0.norm();
		target.topRightCorner(3, 1) = 1000 * target.topRightCorner(3, 1);
		tar = Keith::fromT2X(target);
		resu.head(6) = tar;
		resu[6] = err;
		resu[7] = cost_time;

		for (size_t m = 0; m < resu.size(); ++m) {
			outfile << resu[m];
			if (m < resu.size() - 1) {
				outfile << " "; // 在数字之间添加空格
			}
		}
		outfile << std::endl; // 每行写入后换行
	}
	outfile.close(); // 关闭文件
}
