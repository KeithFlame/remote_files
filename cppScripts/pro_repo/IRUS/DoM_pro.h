#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_DoM_pro.h"
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets\QRadioButton>
#include <QtWidgets\QCheckBox>
#include <Qtwidgets\QComboBox>
#include <windows.h>
#include <QtCore/QTimer>
#include <QFile>
#include <QTextStream>
#include <QThread>
#include <opencv2\opencv.hpp>
#include <opencv2/imgproc/types_c.h>
#include <string>

#include "info_define.h"
#include "difference_function.h"
//#include "sr_mbx.h"

#include "trocar.h"
#include "conf_read.h"

const int ARM_NUM = 3;

//using namespace sr;


class DoM_pm : public QMainWindow
{
	Q_OBJECT

public:
	DoM_pm(QWidget* parent = nullptr);
	~DoM_pm();
	void showInformation(QString, InfoSymbol is = InfoSymbol::INFO_NORMAL);	                    // 显示日志信息

	QPushButton* arm_psi_button[4][2];                                                          // 按钮控件
	QLineEdit* psi_lineEdit[3][6];                                                              // 控制流中输入框控件
	QLineEdit* params_tool_lineEdit[2][6];                                                      // 结构参数中输入框控件
	QTextBrowser* textBrowser;                                                                  // 日志信息
	QPushButton* control_button[9];                                                             // 按钮控件
	QLineEdit* num_lineEdit;                                                                    // 测试名称用
	QLabel* view_label;																		// view

	// new
	QRadioButton* motion_module[4];
	QLineEdit* params_auxiliary_port_lineEdit[6];                                                      // 结构参数中输入框控件
	QLineEdit* params_trocar_pose_lineEdit[2][6];                                                      // 结构参数中输入框控件
	//QLineEdit* arm_num_lineEdit;                                                                    // 测试名称用
	QCheckBox* is_given_wrech;																	//给定外力，还是内部计算外力
	QComboBox* arm_num;																			// 不同的定位臂编号

	double toolPose[16];
	Eigen::Matrix4d cur_pose;
	Eigen::Matrix4d cur_target;
	Eigen::Matrix4d target;
	Eigen::Matrix4d target_inv;
	Eigen::Matrix4d tb[ARM_NUM];
	Eigen::Matrix4d dev_mat = Eigen::Matrix4d::Identity();

private:    // 私有成员变量
	Ui::DoM_proClass ui;
	HANDLE h_mutex;															                    // 用于传递信息至日志，具体生效方式暂未知
	QTimer* display_timer;													                    // frame刷新时间
	QFile* log_file_handle;													                    // 创建文档
	QTextStream* log_stream;												                    // 实时传输信息的流
	QString log_file_path = "";												                    // 日志文件保存位置
	DifferenceMethod* diff_method;

	//CSrMbx serverImage;
	//CSrMbx serverMotionPara;
	//CSrMbx clientVisionCommand;

	//CSrMbx clientMotionPara_Arm1;
	//CSrMbx clientMotionPara_Arm2;
	////CSrMbx clientMotionPara_Endo;
	//CSrMbx clientMotionPara_Arm4;
	//CSrMbx serverInfo_Arm1;
	//CSrMbx serverInfo_Arm2;
	////CSrMbx serverInfo_Endo;
	//CSrMbx serverInfo_Arm4;
	cv::Mat frame;
	QTimer* displayTimer;
	QTimer* commandTimer;
	QTimer* inverse2Timer;

	Trocar* trocar[3];
	SurgicalContinuumManipulator* CC_tool;
	SurgicalContinuumManipulator* standard_tool;

	Eigen::Vector12d guess;
	std::vector<Eigen::Vector27d> guess_tem;
	Eigen::Vector12d Rsd_last;
	QString file_path;
	bool is_new = false;
	Solver solver = Solver::NONE_SOLVER;

private:    // 私有成员函数                                                                                        

	void createUI();                                                                            // 创建UI
	void setStopOptimal();
	void setStartOptimal();
	std::vector<double> getPose();
	void connectSecondProgramme();
	void cameraOpenClicked();																	// 打开相机
	void openSecondPrograme();
	void createSecondPrograme();
	//const std::vector<Eigen::Vector6d> getQafromUI();
	//const std::vector<Eigen::Vector6d> reorderQa_get(std::vector<Eigen::Vector6d>& qa);
	//const std::vector<Eigen::Vector6d> reorderQa_set(std::vector<Eigen::Vector6d>& qa);

	//void setQaToUI(const std::vector<Eigen::Vector6d>& qa);
	const std::vector<Eigen::Vector12d> getParams();
	const Eigen::Matrix4d getTarget();
	const Eigen::Matrix4d getCurrentTarget();
	void setTrocar();
	const Eigen::Vector6d getPsiFromTrocar(const Eigen::Vector6d& psi, const int arm_num);
	void getTrocarBase();
private slots:
	void showInformationSlot(QString);										                    // 输出信息相关
	void clickBtnOpenDeviceslot();											                    // 确认输入的工具信息
	void displayVideoSlot();																	// 开始显示
	void checkCommandSlot();
	void PushButtonClearLogSlot();
	void sendCommandToArm1Slot();
	void sendCommandToArm2Slot();
	void sendCommandToArm4Slot();
	//void calcInverseProblemsSlot();
	void confirmParamsSlot();
	void getCurrentPoseSlot();
	void calcStiffnessObserverSlot();
	//void temTestSlot();
	void connectDevicesSlot();
	void getDeviceParamsSlot();
	void getTissueParamsSlot();
	void connectArm1Slot();
	void connectArm2Slot();
	void connectArm4Slot();
	void singleStepSlot();
	void singleHitInverseProblems2Slot();
	void multipleHitInverseProblems2Slot();
	void continueInverseProblemsSlot();
	void stopInverseProblemsSlot();

	// new
	void clickBtnInverseProblemslot();											                    // 确认问题种类
	void clickBtnForwardProblemslot();											                    // 确认问题种类
	void clickBtnKinematicsslot();											                    // 确认是否为运动学
	void clickBtnKinetostaticsslot();											                    // 确认是否为静力学
	void clickBtnUpdateParamsSlot();																// 更新UI上的手术工具、辅助工具、鞘管位姿参数
	void clickBtnStartCalculationSlot();															// 开始计算并下发结果
	void clickBtnWrenchSectionSlot();

	void clickBtnTestInverseKinetostaticsSlot();
	void clickBtnTestForwardKinetostaticsSlot();
	// new
private:
	Eigen::Matrix4d calcForwardKinematics(const Eigen::Vector6d);								// 正运动学
	Eigen::Vector6d calcInverseKinematics(const Eigen::Matrix4d);								// 逆运动学
	Eigen::Matrix4d calcForwardKinetostatics(const Eigen::Vector6d);							// 正静力学
	Eigen::Vector6d calcInverseKinetostatics(const Eigen::Matrix4d);							// 逆静力学
	Eigen::Vector6d calcExternalWrench();
	int convertAndSet(QLineEdit* lineEdit, const double default_value, double& resule);
	void setUIPsi(const Eigen::Vector6d);
	void setUIWrench(const Eigen::Vector6d);
	void setUIPose(const Eigen::Matrix4d);
	Eigen::Vector6d getUIPsi();
	Eigen::Vector6d getUIWrench();
	Eigen::Matrix4d getUIPose();

signals:
	void showInformationSig(QString);										                    // 显示log
};

