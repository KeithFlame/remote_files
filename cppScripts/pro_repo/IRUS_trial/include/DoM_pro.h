#pragma once

#include <QtWidgets/QMainWindow>
#include "ui_DoM_pro.h"
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QTextBrowser>
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

//using namespace sr;


class DoM_pm : public QMainWindow
{
	Q_OBJECT

public:
	DoM_pm(QWidget* parent = nullptr);
	~DoM_pm();
	void showInformation(QString, InfoSymbol is = InfoSymbol::INFO_NORMAL);	                    // ��ʾ��־��Ϣ

	QPushButton* arm_psi_button[4][2];                                                          // ��ť�ؼ�
	QLineEdit* psi_lineEdit[4][8];                                                              // �������������ؼ�
	QLineEdit* params_lineEdit[6][6];                                                           // �ṹ�����������ؼ�
	QTextBrowser* textBrowser;                                                                  // ��־��Ϣ
	QPushButton* control_button[9];                                                             // ��ť�ؼ�
	QLineEdit* num_lineEdit;                                                                    // ����������
	QLabel* view_label;																		// view
	double toolPose[16];
	Eigen::Matrix4d cur_pose;
	Eigen::Matrix4d cur_target;
	Eigen::Matrix4d target;
	Eigen::Matrix4d target_inv;
	Eigen::Matrix4d tb[ARM_NUM];
	Eigen::Matrix4d dev_mat = Eigen::Matrix4d::Identity();

private:    // ˽�г�Ա����
	Ui::DoM_proClass ui;
	HANDLE h_mutex;															                    // ���ڴ�����Ϣ����־��������Ч��ʽ��δ֪
	QTimer* display_timer;													                    // frameˢ��ʱ��
	QFile* log_file_handle;													                    // �����ĵ�
	QTextStream* log_stream;												                    // ʵʱ������Ϣ����
	QString log_file_path = "";												                    // ��־�ļ�����λ��
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

	std::vector<Eigen::Vector27d> guess;
	std::vector<Eigen::Vector27d> guess_tem;
	Eigen::Vector26d Rsd_last;
	QString file_path;
	bool is_new = false;

private:    // ˽�г�Ա����                                                                                        

	void createUI();                                                                            // ����UI
	void setStopOptimal();
	void setStartOptimal();
	std::vector<double> getPose();
	void connectSecondProgramme();
	void cameraOpenClicked();																	// �����
	void openSecondPrograme();
	void createSecondPrograme();
	const std::vector<Eigen::Vector6d> getQafromUI();
	const std::vector<Eigen::Vector6d> reorderQa_get(std::vector<Eigen::Vector6d>& qa);
	const std::vector<Eigen::Vector6d> reorderQa_set(std::vector<Eigen::Vector6d>& qa);

	void setQaToUI(const std::vector<Eigen::Vector6d>& qa);
	const std::vector<Eigen::Vector12d> getParams();
	const Eigen::Matrix4d getTarget();
	const Eigen::Matrix4d getCurrentTarget();
	void setTrocar();
	const Eigen::Vector6d getPsiFromTrocar(const Eigen::Vector6d& psi, const int arm_num);
	void getTissuePose(const Eigen::Matrix4d& feature_pose, const Eigen::Vector6d& psi, const int arm_num);
	void getTrocarBase();
private slots:
	void showInformationSlot(QString);										                    // �����Ϣ���
	void clickBtnOpenCameraslot();											                    // ȷ������Ĺ�����Ϣ
	void displayVideoSlot();																	// ��ʼ��ʾ
	void checkCommandSlot();
	void PushButtonClearLogSlot();
	void sendCommandToArm1Slot();
	void sendCommandToArm2Slot();
	void sendCommandToArm4Slot();
	void calcInverseProblemsSlot();
	void confirmParamsSlot();
	void getCurrentPoseSlot();
	void calcStiffnessObserverSlot();
	void temTestSlot();
	void connectDevicesSlot();
	void getDeviceParamsSlot();
	void getTissueParamsSlot();
	void connectArm1Slot();
	void connectArm2Slot();
	void connectArm4Slot();
	void singleStepSlot();
	void calcInverseProblems2Slot();
	void singleHitInverseProblems2Slot();
	void multipleHitInverseProblems2Slot();
	void continueInverseProblemsSlot();
	void stopInverseProblemsSlot();

	void calcInverseProblems3Slot();
	void calcInverseProblems4Slot();
	void calcInverseProblems5Slot();
	void calcStiffnessObserver2Slot();
	void calcStiffnessObserver3Slot();
signals:
	void showInformationSig(QString);										                    // ��ʾlog
};
