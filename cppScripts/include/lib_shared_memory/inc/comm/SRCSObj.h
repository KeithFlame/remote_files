#ifndef  SRCS_OBJ_H
#define  SRCS_OBJ_H
#include "MoveDefine.h"
#include "errorDef.h"
#include "pack_define.h"
#include "SRCSObj_define.h"

/*MACRO Definition*/
const uchar Mode_BroadCast = 0;
const uchar Mode_Direct = 1;
const uchar Mode_System = 2;

const uchar Mode_R_Broadcast = 8;
const uchar Mode_R_Direct = 9;
#define TRANSIT_MODE_MASK 0x01
#define TRANSIT_ROUT_MASK 0x08

const uchar FuncGroup1 = 1;
const uchar FuncGroup2  = 2;
const uchar FuncGroupAll = 0xff;
const ushort MAX_BUFFER = 512;
const ushort MAX_APP_BUFFER_SZ = 520;
const ushort MAX_LENGTH = 532;
const ushort HEADER_LENGTH = 20;
const ushort HEADER_PP_LENGTH = 12;//Seq Number, CRC[2] coast
const ushort HEADER_APP_LENGTH = 8; //sizeof(SRCS_APP_Header)

const uchar SWITCH_ON = 1;
const uchar SWITCH_OFF = 0;


const uchar g_left_combx_id = 1;
const uchar g_right_combx_id = 2;


#define MAX_SR_NODE_SUPPORT 16 //only cares the SRN Client connect to the ROOT
#define MAX_SR_IN_LIST      16 //Each Node support broadcast received count in most.

#define SR_COMP_SIZE 16


#ifdef SR_WIN_NT_SUPPORT
#pragma pack (push, 1)
#endif

typedef struct
{
	uchar Sender;
	uchar Target;
	ushort srcsObject;
	uchar mode;
	uchar group;
	ushort length;

	ulong SeqNumber;
	union 
	{
		ulong CRC64[2];
		uchar SysPktData[8];
	}FlexHeader;
}SRCS_DEF_PACKED SRCS_Header_t;

typedef struct
{
	SRCS_Header_t header;
	uchar data[MAX_BUFFER];
}SRCS_DEF_PACKED SRCS_Frame_t;

#define SRNODE_MCUE       0x20 //MCU take Motion control of Endoscope Arm
#define SRNODE_MCU1       0x21 //MCU take Motion control of Arm 1
#define SRNODE_MCU2       0x22 //MCU take Motion control of Arm 2
#define SRNODE_MCU3       0x23 //MCU take Motion control of Arm 3
#define SRNODE_MONITORVD 0x30 //Take 4 Monitor Board and 1 Safe GuardBoard
#define SRNODE_MASTER_DEV_MANAGER  0x31 //master dev manager
#define SRNODE_IO_BOARDVD 0x31
#define SRNODE_RRALG     0x40 //ResoRate Algorithm
#define SRNODE_STATUSDISPLAYUI 0x41 // HMI Running on Master PC
#define SRNODE_TOUCHSCREENUI 0x42 // HMI Running on Master PC
#define SRNODE_UI1       0x42 // HMI Running on Master PC
#define SRNODE_UI2       0x43 //HMI Running on Slave PC
#define SRNODE_UIADAPTER 0x44 //HMI Running on Slave PC
#define SRNODE_VS        0x45 //Virtual Simulation Show
#define SRNODE_SETUPTOOL 0x46 //
#define SRNODE_MHI       0x50 //OMIN Device Collection
#define SRNODE_NDI       0x70 //NDI Device Management
#define SRNODE_SERVICE   0x60 //Service 
#define SRNODE_SRNSOS    0xE0 //0xE0~0xED is for SRNSOS
#define SRNODE_SRNREMOTESOS 0xEF //0xEE~0xEF is for SRNREMOTESOS

#define SRNODE_WRITEFILE   0x80 //Record Data For Test
#define SRNODE_CANADAPTER  0x90

#define SRNODE_PROCESSMANAGER_M 0xB0
#define SRNODE_PROCESSMANAGER_S 0xB1

#define SRNODE_SC          0xA0 //System Control Unit
#define SRNODE_ROOT_M      0xF0//SRNL Root on Master PC
#define SRNODE_ROOT_S      0xF1//SRNL Root on Slave PC
#define SRNODE_INVALID     0x00 
#define SRNODE_VISION      0xC0

typedef ulong SysStatusEnum;
//status not ready may be group of one or more reason

const SysStatusEnum SysStatusNoInfo    = 0x00000000; //NoInfo, usually used for intemediate handle, will not transit out via ISYSTEMINFO
const SysStatusEnum SysStatusReady     = 0x00000001;
const SysStatusEnum SysStatusNoMatch   = 0x00000002;
const SysStatusEnum SysStatusTOBuild   = 0x00000004; //Not Ready because of TO is Build
const SysStatusEnum SysStatusNoToolArm = 0x00000008; //Not Ready because of None ToolArm Setup, or ToolArm Lost.
const SysStatusEnum SysStatusMCUComm   = 0x00000010; //Not Ready because of MCU COMM Lost.
const SysStatusEnum SysStatusToolArm   = 0x00000020;   //Not Ready because of ToolArm Lost Connection or Fail.

typedef SREnum ArmWorkingType;
const ArmWorkingType ArmWorkingUnknown            = 0;  //Install
const ArmWorkingType ArmWorkingCommCheck          = 1;  //Communication Check PASS
const ArmWorkingType ArmWorkingHoming             = 2;  //Homing Finish
const ArmWorkingType ArmWorkingSetupToolAntiB     = 3;  //tool AntiB setup ok
const ArmWorkingType ArmWorkingSetupTrocarAntiB   = 4;  //Trocar AntiB setup ok
const ArmWorkingType ArmWorkingSetupToolMarker    = 5;  //Marker setup ok
const ArmWorkingType ArmWorkingSetupTrocar        = 6;  //Trocar setup ok
const ArmWorkingType ArmWorkingSetupArmTool       = 7;  //tool setup ok
const ArmWorkingType ArmWorkingFinished           = 8;//if Finished 
const ArmWorkingType ArmWorkingError              = 14;
const ArmWorkingType ArmWorkingWarning            = 15;

const ArmWorkingType ArmWorkingMovementEna  = 16;//0 indicates movement disable active
const ArmWorkingType ArmWorkingTracking     = 17;//0 indicates Lost Tracking
const ArmWorkingType ArmWorkingDmg = 18;//0 indicates Disable Dmg 1 enable Dmg
const ArmWorkingType ArmWorkingEmergencyStop = 19;//EmergencyStop
const ArmWorkingType ArmWorkingTrundle = 20;//Trundle Statuas 0 Not fold  1 fold

const ArmWorkingType ArmWorkingEnergyCut    = 24;
const ArmWorkingType ArmWorkingEnergyCoag   = 25;

typedef enum16 FunctionCode;

const FunctionCode ArmMovingStatus    = 51;
const FunctionCode ArmMoveTargetPos   = 52;
const FunctionCode ArmLockStatus      = 53;
const FunctionCode ArmMcuCommStatus   = 54;
const FunctionCode ArmToolSetUp       = 55;
const FunctionCode ArmMovementDisable = 56;
const FunctionCode ArmToolBarrierStatus = 57;
const FunctionCode ArmToolIOStatus = 58;
const FunctionCode ArmTorcarStatus = 59;
const FunctionCode ArmTorcarBarrierStatus = 60;
const FunctionCode ArmMarkerStatus = 61;
const FunctionCode ArmPrepareStatus = 62;

const FunctionCode ArmDmgStatuas = 70;
const FunctionCode ArmEmergencyStop = 71;
const FunctionCode ArmTrundleStatus = 72;
const FunctionCode ArmMEmergencyStop = 73;
const FunctionCode ArmSEmergencyStop = 74;
const FunctionCode ArmSStop = 74;

const FunctionCode SetArmError        = 80;
const FunctionCode SetArmWarning      = 81;

const FunctionCode fcControlCamera    = 101;
const FunctionCode fcControlTeleOper  = 105;
const FunctionCode fcControlSwapArmL  = 117;
const FunctionCode fcControlSwapArmR  = 118;
const FunctionCode fc2ndEnegerLeft    = 121;
const FunctionCode fc2ndEnegerRight   = 127;
const FunctionCode fc1stEnegerLeft    = 132;
const FunctionCode fc1stEnegerRight   = 141;
const FunctionCode fcFeeding          = 150;
const FunctionCode HapDevL            = 151;
const FunctionCode HapDevR            = 152;
const FunctionCode ArmMotorMultiFunc  = 160;
const FunctionCode ArmMotorLock       = 161;
const FunctionCode ArmInnerEnaMove    = 170;
const FunctionCode ArmOutToggleMove   = 171;
const FunctionCode ArmToggleMoveLock  = 172;//Active/De-Active Arm Lock Function(Movement Disable), paramer should be e_ARM_E~e_ARM_3

// Operation Type
typedef SREnum OperationType;

const OperationType OperTypeUnknown = 0xff;
const OperationType OperType1 = 0x00;
const OperationType OperType2 = 0x01;
const OperationType OperType3 = 0x02;
const OperationType OperType4 = 0x03;
const OperationType OperType5 = 0x04;
const OperationType OperTypeSize = 0x20; //32
/*   ConfirmError:UI Adatpter->System Control to tell user confirm the error
     consoleType = ConsoleMasterUI,
     controlElementType = PUSH_BUTTON
     param is not care
*/
const FunctionCode ConfirmError      = 200;


/********** Select Arm Move Function, UI Adapter -> System Control ************/
//consoleType=ConsoleMasterUI,controlElementType=PUSH_BUTTON,
//param=0,1,2,3 indicate arm type

typedef SREnum ArmMoveStatus;
const ArmMoveStatus ArmReached = 1;
const ArmMoveStatus ArmStop    = 2;
const ArmMoveStatus ArmRunning = 3;
const ArmMoveStatus ArmError   = 0xF;

/****************** Error Category Definition *********************************/
const SREnum cate_Warning      = 0x01;   //00000001b
const SREnum cate_Error        = 0x02;   //00000010b
const SREnum cate_FatalError   = 0x03;   //00000110b
const SREnum cate_UserInfor    = 0x05;   //00000101b
const SREnum cate_SmartUInfor  = 0x09;   //00001001b
const SREnum reset_Error       = 0x12;
const SREnum reset_UserInfor   = 0x15;
const SREnum reset_SmartUInfor = 0x19;

const SREnum cate_EventLogWarning         = 0x21;
const SREnum cate_EventLogError           = 0x22;   //00000010b
const SREnum cate_EventLogFatalError      = 0x23;   //00000110b
const SREnum cate_EventLogUserInfor       = 0x25;   //00000101b
const SREnum cate_EventLogSmartUInfor     = 0x29;   //00001001b
const SREnum reset_EventLogEventLogError  = 0x32;
const SREnum reset_EventLogUserInfor      = 0x35;
const SREnum reset_EventLogSmartUInfor    = 0x39;

/****************** Error Category Definition End******************************/


/************************ MailBoxName   ***************************************/
//#define SR_MAIL_BOX_PRE "/sr_server_mbx_"
#define SR_MAIL_BOX_FORMAT "/sr_server_mbx_%02x"
//"/sr_server_mbx_0x40" for RRAlg Receive
//"/sr_server_mbx_0x30" for MonitorVD Receive
//#define RRAlg_SERVER_MBX_NAME "/sr_rralg_server_mbx"
//#define Opengl_SERVER_MBX_NAME "/sr_opengl_server_mbx"
/************************ MailBoxName     END***********************************/

#define ACONNECT 0x0001
#define IACCEPT  0x0002
#define ICONFIRM 0x0003
#define ACHECKALIVE 0x0004
#define IACK 0x0005
#define SRESETCOMM 0x0006
#define IHEARTBEAT 0x0009
#define TURN_ON 0x0007
typedef struct
{
	uchar          NodeID; /* NODEID */
	uchar          dummy[3];
}SRCS_DEF_PACKED TurnOn_t;

#define TURN_OFF 0x0008
typedef struct
{
	uchar          NodeID; /* NODEID */
	uchar          dummy[3];
}SRCS_DEF_PACKED TurnOff_t;
#define SRCS_SEND_SUCCEED       0x0015
#define SRCS_SEND_FAILURE       0x0016
#define SRCS_SEND_BROADCAST     0x0017



enum MonitorMsgType{
	SendFailedMsg = 0,
	SendSuccessMsg = 1,
	BroadCastMsg = 2
};


#define ROOT_COMM    0x1000
#define ROOT_SYNCSEND_COMM    0x1006
#define ROOT_SYNCCONFRIM_COMM    0x1007

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: >2 Byte   Number of params: 2						  */
/*																			  */
/*    Root->SRSOS(Mode==system) for Update Node List,						  */
/*    produce once one SRSOS Node Join	into the SR	,						  */
/*    or the Link Node Status changed.										  */
/*############################################################################*/
#define ICONNECTNODE 0x1001

typedef struct
{
	ushort        no_objects;       /* number of objects */
} SRCS_DEF_PACKED IConnectNodeCnt;

typedef struct
{
	uchar          NodeID; /* NODEID */
	uchar          dummy;
	//ushort         InListObj[MAX_SR_IN_LIST];
} SRCS_DEF_PACKED IConnectNodeList;

typedef struct  
{
	SRCS_APP_Header  header;
	IConnectNodeCnt  count;
	IConnectNodeList nodeList[MAX_SR_NODE_SUPPORT];
}SRCS_DEF_PACKED IConnectNodeFrame_t;


#define ROOT_HEARTBEAT 0x1002
typedef struct
{
	SRCS_APP_Header  header;
}SRCS_DEF_PACKED IHeartBeatFrame_t;

#define ROOT_SUCCESSMSG_MONITOR 0x1003
#define ROOT_FAILEDMSG_MONITOR 0x1004
#define ROOT_BROADCASTMSG_MONITOR 0x1005



#define  AARMPOS 0x0034

typedef struct
{
	e_ARM_Type       ArmType;
	SRCS_DUMMY       dummy;	
}SRCS_DEF_PACKED AArmPos;

typedef struct
{
	SRCS_APP_Header header;
	AArmPos         data;
}SRCS_DEF_PACKED AArmPosFrame_t;

#define ACHECKCOMPSTATUS 0x0043

typedef struct
{
	uchar recNode;
	uchar Dummy;
}SRCS_DEF_PACKED ACheckCompStatus;

typedef struct
{
	SRCS_APP_Header  header;
	ACheckCompStatus data;
}SRCS_DEF_PACKED ACheckCompStatusFrame_t;

#define SCONNECT 0x0100

typedef struct
{
	ushort        no_objects;       /* number of objects */
} SRCS_DEF_PACKED sconnect_t1;

typedef struct
{
	ushort          object_id; /* ID of the object */
} SRCS_DEF_PACKED sconnect_t2;

typedef uchar CompStatusType;
const CompStatusType CompStatusHardwareOK     = 1;
//const CompStatusType CompStatusCommPass       = 2;
const CompStatusType CompStatusOperational    = 10;
#define ICOMPSTATUS 0x0010
typedef struct  
{
	uchar      nodeID;
	uchar      status;
	uchar      addtionalInfo[2];//if nodeID == 0x30, then addtionalInfo[0] indicates arm Type
}SRCS_DEF_PACKED ICompStatus;

typedef struct  
{
	SRCS_APP_Header header;
	ICompStatus     data;
}SRCS_DEF_PACKED ICompStatusFrame_t;

#define ACHECKSTATUS 0x0011
typedef struct  
{
	SREnum SRNNode;
	SRCS_DUMMY dummy[3];
}SRCS_DEF_PACKED ACheckStatus;

typedef struct  
{
	SRCS_APP_Header header;
	ACheckStatus    data;
}SRCS_DEF_PACKED ACheckStatusFrame_t;

#define IARMWORKING 0x0012
const SREnum StepWorking  = 1;
const SREnum StepWorkFail = 2;

typedef struct
{
	e_ARM_Type     arm;
	ArmWorkingType workingStep[3];
	SREnum         CurrentStatus;
	uchar          detailInfo[2];
	uchar          dummy;
}SRCS_DEF_PACKED IArmWorking;

typedef struct
{
	SRCS_APP_Header   header;
	IArmWorking       data;
}SRCS_DEF_PACKED IArmWorkingFrame_t;


/******************************************************************************/
/*																			  */
/*																			  */
/*				Length: 4    Number of params: 3	    					  */
/*																			  */
/*              BROADCAST MODE, unit MonitorVD->                              */
/******************************************************************************/
#define IARMGENSTATUS 0x0013
typedef struct
{
	e_ARM_Type        arm;		
	uchar             dummy[3];
	ulong             itemCareMASK;//bit definition refer to ArmWorkingType
	ulong             actualStatus;//bit definition refer to ArmWorkingType	
}SRCS_DEF_PACKED IArmGenStatus_t;

typedef struct
{
	SRCS_APP_Header   header;
	IArmGenStatus_t   data;
}SRCS_DEF_PACKED IArmGenStatusFrame_t;

#define SSYSTEMTRUNOFFCMD 0x0044

typedef SREnum s_SYS_Type;
const s_SYS_Type MASTER_SYSTEM = 1;
const s_SYS_Type SLAVE_SYSTEM = 2;

const SREnum READY_FOR_TRUNOFF = 1;
const SREnum WAIT_TRUNOFF_CONFRIM = 2;
const SREnum CONFRIM_FOR_TRUNOFF = 3;
const SREnum CANCEL_TRUNOFF_CMD = 4;
typedef struct
{
	s_SYS_Type        system;
	uchar             cmd;
	uchar             time;
	uchar             Dummy;
}SRCS_DEF_PACKED SSystemTrunOffCmd_t;


typedef struct
{
	SRCS_APP_Header   header;
	SSystemTrunOffCmd_t   data;
}SRCS_DEF_PACKED SSystemTrunOffCmdFrame_t;

#define ISYSTEMTRUNOFF 0x0045



#define ITOOLINFOCONFRIM 0x0046
const SREnum TOOL_INFO_INVALID = 0x00;
const SREnum TOOL_INFO_VALID = 0x0f;

typedef struct
{
	e_ARM_Type        arm;
	uchar             Value;
}SRCS_DEF_PACKED iToolInfoConfrim_t;

typedef struct
{
	SRCS_APP_Header      header;
	iToolInfoConfrim_t   data;
}SRCS_DEF_PACKED iToolInfoConfrimFrame_t;


#define IREACHEDINNERLIMIT 0x0047
typedef struct
{
	e_ARM_Type        arm;
	uchar             Dummy;
}SRCS_DEF_PACKED iReachedInnerlimit_t;

typedef struct
{
	SRCS_APP_Header      header;
	iReachedInnerlimit_t   data;
}SRCS_DEF_PACKED iReachedInnerlimitFrame_t;

/******************************************************************************/
/*																			  */
/*																			  */
/*				Length: 6     Number of params: 6	    					  */
/*																			  */
/*              BROADCAST MODE, Generator Source: unit MonitorVD              */
/******************************************************************************/

#define IELECUTANDCOAGSTATUS 0x0048
const uchar ELE_CUT_START = 0x01;
const uchar ELE_CUT_STOP = 0x02;
const uchar ELE_COAG_START = 0x03;
const uchar ELE_COAG_STOP = 0x04;
typedef struct
{
	e_ARM_Type        arm;
	uchar             Status;
}SRCS_DEF_PACKED ieleCutAndCoagStatus_t;

typedef struct
{
	SRCS_APP_Header          header;
	ieleCutAndCoagStatus_t   data;
}SRCS_DEF_PACKED ieleCutAndCoagStatusFrame_t;


/******************************************************************************/
/*																			  */
/*																			  */
/*				Length: 6     Number of params: 6	    					  */
/*																			  */
/*              BROADCAST MODE, Generator Source: unit MonitorVD              */
/******************************************************************************/
#define IARMPREPAREFINISHED 0x0049
const uchar ARM_PREPARENOTFINISHED = 0x00;
const uchar ARM_PREPAREFINISHED = 0x01;
typedef struct
{
	e_ARM_Type        arm;
	uchar             Status;
}SRCS_DEF_PACKED iArmPrepareFinish_t;

typedef struct
{
	SRCS_APP_Header          header;
	iArmPrepareFinish_t   data;
}SRCS_DEF_PACKED iArmPrepareFinishFrame_t;


/******************************************************************************/
/*																			  */
/*																			  */
/*				Length: 6     Number of params: 6	    					  */
/*																			  */
/*              BROADCAST MODE, Generator Source: unit MonitorVD              */
/******************************************************************************/
#define ITOOLSTATUS 0x0014

typedef SREnum ArmType_t;
const ArmType_t ArmTypeUnknown = 0;
const ArmType_t ArmTypeE1 = 1;
const ArmType_t ArmTypeNorminal = 6;
const ArmType_t ArmTypeLong = 7;
const ArmType_t ArmTypeNorminal_NewEd = 8;

typedef SREnum ExecuteToolType;
const ExecuteToolType ExecuteToolUnknown = 0;
const ExecuteToolType ExecuteToolGrip = 1;
const ExecuteToolType CHI_ZHEN_QI = 1; 
const ExecuteToolType WU_CHUANG_JIA_QIAN = 2;
const ExecuteToolType FEN_LI_QIAN = 3;
const ExecuteToolType SHUANG_DONG_WAN_JIAN = 4;
const ExecuteToolType SHUANG_DONG_ZHI_JIAN = 5;
const ExecuteToolType NEI_ZANG_ZHUA_QU_QIAN = 6;
const ExecuteToolType SHU_YA_JIA_QIAN = 7;
const ExecuteToolType CHANG_HE_ZHUA_QU_QIAN = 8;
const ExecuteToolType CHANG_ZHUA_QIAN = 9;
const ExecuteToolType DAN_DONG_WAN_JIAN = 10;
const ExecuteToolType TAI_JIA_QIAN = 11;
const ExecuteToolType DAN_JI_DIAN_GOU = 12;
const ExecuteToolType DAN_JI_WAN_JIAN = 13;
const ExecuteToolType SHUANG_JI_WAN_FEN_LI = 14;
const ExecuteToolType SHUANG_JI_WU_CHUANG_ZHUA_QIAN = 15;
const ExecuteToolType SHAUNG_JI_MA_LI_LAN_QIAN = 16;
const ExecuteToolType FU_QIANG_JING = 17;

const SREnum RestrainGripperPair = 0;
const SREnum AllowGripperPair = 1;
typedef struct
{
	e_ARM_Type        arm;
	ArmType_t         armType;
	ExecuteToolType   toolType;
	SREnum            supportCut;   //1 indicates Support, 0 indicates Not Support
	SREnum			  supportCoag;  //1 indicates Support, 0 indicates Not Support
	SREnum            usedCount;
	SREnum            SupportGripperPair;
	SREnum            Dummy;
}iToolStatus_t;

typedef struct
{
	SRCS_APP_Header   header;
	iToolStatus_t     data;	
}SRCS_DEF_PACKED iToolStatusFrame_t;



const ulong UNKNOWN_TOOL_NUM = 0;
const uchar UNKNOWN_TOOL_FACTORY = 0;
const uchar SUPPORTGRIPPERPAIR_BITINDEX = 15;
const uchar SUPPORTCUT_BITINDEX = 14;
const uchar SUPPORTCOAG_BITINDEX = 13;
const uchar IS_MULTIPORT_OR_SINGLEPORT_BITINDEX = 12; //0 is singleport 1 is multiport
const uchar SUPPORTELETOOL_BITINDEX = 11;
const uchar UNIPOLAR_OR_BIPOLAR_BITINDEX = 10;

#define SMOTPOS    0x0020
typedef struct
{
	T_MOVE_AXIS_TYPE axis;
	SRCS_DUMMY       dummy[2];
	SRCS_LONG        position;       //unit is incremental Encoder count
	SRCS_LONG        profileVelocity;//unit is rpm Halt the motor right now if maxVelocity = 0
}SRCS_DEF_PACKED ControlData;

typedef struct  
{
	SRCS_DCOUNT  itemCount;      //<=e_MOVE_AXIS_COUNT
}SRCS_DEF_PACKED controlDataCount;

typedef struct
{
	SRCS_APP_Header   header;
	controlDataCount  count;
	ControlData       bufferData[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED SMotPosFrame_t;

#define SMOVECMD  0x0025
typedef struct
{
	e_ARM_Type      ArmType;
	ArmPos          MoveTargetPos;
	ArmMoveCmdType  cmd;
	SRCS_DUMMY      dummy;
}SRCS_DEF_PACKED SMoveCmd_t;

typedef struct
{
	SRCS_APP_Header header;
	SMoveCmd_t      data;
}SRCS_DEF_PACKED SMoveCmdFrame_t;

#define SMOVEENA  0x0027
typedef struct
{
	e_ARM_Type      ArmType;	
	ArmMoveCmdType  cmd;     //ArmMoveCmdEna,ArmMoveCmdDisable
	SRCS_DUMMY      dummy[2];
}SRCS_DEF_PACKED SMoveEna_t;

typedef struct
{
	SRCS_APP_Header header;
	SMoveEna_t      data;
}SRCS_DEF_PACKED SMoveEnaFrame_t;

#define IMOTPOS        0x0021
typedef struct{
	e_ARM_Type       ArmType;
	SRCS_DUMMY       dummy;
	T_MOVE_AXIS_TYPE axis;	
	SRCS_LONG        actPosition; //unit is incremental Encoder count
	SRCS_LONG        actVelocity; //unit is rpm
}SRCS_DEF_PACKED DrvStatusData;

typedef struct{
	SRCS_DCOUNT  controlDataCount;      //<=e_MOVE_AXIS_COUNT
	DrvStatusData bufferData[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED IPos_t;

typedef struct
{
	SRCS_APP_Header header;
	IPos_t          data;
}SRCS_DEF_PACKED IPosFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 4    Number of params: 3	    					  */
/*																			  */
/*    BROADCAST MODE, unit MonitorVD-> SystemControl--UIAdapter               */
/*############################################################################*/
#define IMOVECMD 0x0026
typedef struct
{
	e_ARM_Type      ArmType;
	ArmPos          MoveTargetPos;
	ArmMoveCmdType  cmd;
	SRCS_DUMMY      dummy;
}SRCS_DEF_PACKED IMoveCmd_t;

typedef struct
{
	SRCS_APP_Header header;
	IMoveCmd_t      data;
}SRCS_DEF_PACKED IMoveCmdFrame_t;


#define IMOVECMDANSWER 0x0126
typedef struct
{
	e_ARM_Type      ArmType;
	ArmPos          MoveTargetPos;
	ArmMoveCmdType  cmd;
	SRCS_DUMMY      dummy;
}SRCS_DEF_PACKED IMoveCmdAnswer_t;

typedef struct
{
	SRCS_APP_Header       header;
	controlDataCount      itemCount;
	IMoveCmdAnswer_t      data[e_ARM_SIZE];
}SRCS_DEF_PACKED IMoveCmdAnswerFrame_t;


#define TARGETRP 0x0051	
typedef struct {
	SRCS_FLOAT32 PosArray[3];
	SRCS_FLOAT32 OrientArray[9];
}SRCS_DEF_PACKED TargetRP_Data_t;

typedef struct {
	e_ARM_Type ArmType;
	TargetRP_Data_t target_data;
}SRCS_DEF_PACKED TargetRP_t;

typedef struct
{
	SRCS_APP_Header   header;
	TargetRP_t   data;
	float GripperAngleData;
}SRCS_DEF_PACKED TargetRPFrame_t;


#define TARGETRPInMatched 0x0065	

#define TARGETDeltaRP 0x0064
//Same as target rp


#define CURRENTRP 0x0053	
typedef struct {
	SRCS_FLOAT32 PosArray[3];
	SRCS_FLOAT32 OrientArray[9];
}SRCS_DEF_PACKED CurrentRP_Data_t;

typedef struct {
	uchar combx_id;
	CurrentRP_Data_t current_data;
}SRCS_DEF_PACKED CurrentRP_t;

typedef struct
{
	SRCS_APP_Header   header;
	CurrentRP_t   data;
}SRCS_DEF_PACKED CurrentRPFrame_t;

#define NewTARGETRP 0x0054	
typedef struct {
	SRCS_FLOAT32 PosArray[3];
	SRCS_FLOAT32 OrientArray[9];
}SRCS_DEF_PACKED NewTargetRP_Data_t;

typedef struct {
	FunctionCode combx_id;
	NewTargetRP_Data_t target_data;
}SRCS_DEF_PACKED NewTargetRP_t;

typedef struct
{
	SRCS_APP_Header   header;
	NewTargetRP_t   data;
	float GripperAngleData;
}SRCS_DEF_PACKED NewTargetRPFrame_t;

//#define Tracker_TransformMatrix 0x0055
//typedef struct {
//	e_ARM_Type ArmType;
//	SRCS_FLOAT32 OrientArray[9];
//	SRCS_FLOAT32 PosArray[3];
//}SRCS_DEF_PACKED TransformMatrix_Data_t;
//
//typedef struct {
//	SRCS_DCOUNT ArmCount;
//}SRCS_DEF_PACKED TransformMatrixCount_t;
//
//typedef struct
//{
//	SRCS_APP_Header   header;
//	TransformMatrixCount_t   count;
//	TransformMatrix_Data_t TransformArray[3];
//}SRCS_DEF_PACKED TransformMatrixFrame_t;

#define AbsoluteTransformMatrix 0x0074
typedef struct
{
	SRCS_APP_Header   header;
	TransformMatrixCount_t   count;
	TransformMatrix_Data_t TransformArray[4];
}SRCS_DEF_PACKED AbsoluteTransformMatrixFrame_t;

//0x0056 ×¢Òâ×Ö½Ú¶ÔÆë
#define ARMHEIGHTOFFSET 0x0056
typedef struct{
	SRCS_APP_Header   header;
	float Arm_Height_OffSet;
	e_ARM_Type arm_type;
}SRCS_DEF_PACKED ArmHeightOffSetFrame_t;

#define UPDATEUIILIMIT 0x0073
typedef struct{
	SRCS_APP_Header   header;
	uchar LimitStatus;//1Lmin 2Lmax 3Phimin 4Phimax 5Theta1 6Theta2
	e_ARM_Type arm_type;
}SRCS_DEF_PACKED UpdateUiLimit_Frame_t;

#define SARMINNERMOVING 0x0057
typedef struct {
	e_ARM_Type ArmType;
	ArmInnerMoveCmdType cmd;
}SRCS_DEF_PACKED sArmInnerMoving_t;

typedef struct
{
	SRCS_APP_Header   header;
	sArmInnerMoving_t   data;
}SRCS_DEF_PACKED sArmInnerMovingFrame_t;
//
//typedef struct{
//	e_ARM_Type ArmType;
//	GripperAngle_t gripperAngle;
//}SRCS_DEF_PACKED Gripper_t;
//
//typedef struct
//{
//	SRCS_APP_Header header;
//	Gripper_t   data;
//}SRCS_DEF_PACKED GripperFrame_t;

//#define CombXGripperAngleRange 0x0058
//typedef struct{
//	FunctionCode  HapDevType;
//	float touchXUpLimitAngle;
//	float touchXDownLimitAngle;
//}SRCS_DEF_PACKED CombXGripperAngleRange_t;
//typedef struct
//{
//	SRCS_APP_Header header;
//    CombXGripperAngleRange_t combx_gripper_anglerange;
//}SRCS_DEF_PACKED CombXGripperAngleRangeFrame_t;
#define HapticStateChange 0x0058
typedef struct{
	FunctionCode  HapDevType;
	uchar haptic_state; //0 state1,1 state2,2 state3
}SRCS_DEF_PACKED HapticStateChange_t;
typedef struct
{
	SRCS_APP_Header header;
	HapticStateChange_t haptic_state_change;
}SRCS_DEF_PACKED HapticStateChangeFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 6 Byte   Number of params: 4						  */
/*																			  */
/*    BROADCAST MODE, unit RR-->SC--UIAdapter                                 */
/*############################################################################*/
#define IMATCHSTATUS 0x0059

typedef SREnum TeleOperMatchStatus;
const TeleOperMatchStatus TeleOperMatchBuild = 1;
const TeleOperMatchStatus TeleOperMatchLost  = 2;
const TeleOperMatchStatus TeleOperMatching = 3;
const TeleOperMatchStatus TeleOperMatched = 4;
const TeleOperMatchStatus TeleOperMatchErr   = 0xE0;

typedef struct
{
	e_ARM_Type          arm;
	TeleOperMatchStatus matchstatus;
	sshort              target_rotation;//0.1 Degree
	sshort              current_rotation;//0.1 Degree
	sshort              target_gripper; //0.1 Degree
	sshort              current_gripper; //0.1 Degree
}SRCS_DEF_PACKED IMatchStatus_t;

typedef struct
{
	SRCS_APP_Header header;
	IMatchStatus_t   data;
}SRCS_DEF_PACKED IMatchStatusFrame_t;

/*############################################################################*/
/*											    							  */
/*																			  */
/*				Length: 4 Byte   Number of params: 3						  */
/*																			  */
/*    BROADCAST MODE, unit SystemControl-->MONITORVD--RR--HMI                 */
/*############################################################################*/
#define ITELEOPERATE 0x0060

typedef SREnum TeleOperCtrlCmd;
const TeleOperCtrlCmd TeleOperCtrlStop    = 0;
const TeleOperCtrlCmd TeleOperCtrlEna     = 1;
const TeleOperCtrlCmd TeleOperCtrlStart   = 2;
const TeleOperCtrlCmd TeleOperCtrlActive  = 3;

typedef struct
{
	e_ARM_Type          arm;
	TeleOperCtrlCmd     ctrlCmd;
	SRCS_DUMMY          dummy[2];
}SRCS_DEF_PACKED ITeleOperate_t;

typedef struct
{
	SRCS_APP_Header header;
	ITeleOperate_t   data;
}SRCS_DEF_PACKED ITeleOperateFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 4 Byte   Number of params: 2						  */
/*																			  */
/*    BROADCAST MODE, unit SystemControl-->MONITORVD--RR--HMI--UIAdapter      */
/*############################################################################*/
#define ITELEOPEROPTION 0x0061

//const FunctionCode HapDevL           = 151;
//const FunctionCode HapDevR           = 152;

const SREnum SREnumYes = 1;
const SREnum SREnumNo  = 0;

typedef struct
{
	SREnum              option;//SREnumYes/SREnumNo
	e_ARM_Type          arm;	
}SRCS_DEF_PACKED ITeleOperOption_t;

typedef struct
{
	SRCS_APP_Header    header;
	ITeleOperOption_t   data;
}SRCS_DEF_PACKED ITeleOperOptionFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: ------   Number of params: 2						  */
/*																			  */
/*    BROADCAST MODE, unit SystemControl--RR--HMI--UIAdapter                  */
/*############################################################################*/
#define ITELEOPERCONFIG 0x0062

//const FunctionCode HapDevL           = 151;
//const FunctionCode HapDevR           = 152;

typedef struct
{	
	e_ARM_Type          arm;	
	SREnum              selection;
	FunctionCode        HapDevType;//Value is enum {HapDevL,HapDevR}
}SRCS_DEF_PACKED ITeleOperConfig_t;

typedef struct
{
	SRCS_APP_Header    header;
	controlDataCount   count;
	ITeleOperConfig_t  data[e_ARM_SIZE];
}SRCS_DEF_PACKED ITeleOperConfigFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: ------   Number of params: 2						  */
/*																			  */
/*    BROADCAST MODE, unit SystemControl--RR--HMI--UIAdapter                  */
/*############################################################################*/
#define ITELEOPERSTATUS 0x0067
const uchar NormalOperate = 0;
const uchar AbNormalOperate = 1;


typedef struct
{
	e_ARM_Type          arm;
	uchar        Status;
}SRCS_DEF_PACKED ITeleStatus_t;

typedef struct
{
	SRCS_APP_Header    header;
	ITeleStatus_t  data;
}SRCS_DEF_PACKED ITeleStatusFrame_t;
/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: ------   Number of params: 2						  */
/*																			  */
/*    BROADCAST MODE, unit SystemControl--RR--HMI--UIAdapter                  */
/*############################################################################*/
#define SENTERSERVICEMODE 0x0068
typedef struct
{
	SRCS_APP_Header    header;
}SRCS_DEF_PACKED sEnterServiceModeFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: ------   Number of params: 2						  */
/*																			  */
/*    BROADCAST MODE, unit UIAdapter--SystemControl                           */
/*############################################################################*/
#define STELEOPERSPEEDMODE 0x0069

const uchar TELEOPERSPEED_MODE_FAST = 1;
const uchar TELEOPERSPEED_MODE_NORMAL = 2;
const uchar TELEOPERSPEED_MODE_EXACT = 3;
typedef struct
{
	SREnum mode;
	SREnum Dummy;
}SRCS_DEF_PACKED sTeleoperSpeedMode_t;

typedef struct
{
	SRCS_APP_Header      header;
	sTeleoperSpeedMode_t     data;
}SRCS_DEF_PACKED sTeleoperSpeedModeFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: ------   Number of params: 2						  */
/*																			  */
/*    BROADCAST MODE, unit SystemControl -- UIAdapter--MHI                    */
/*############################################################################*/

#define ITELEOPERSPEEDMODE 0x0072
typedef struct
{
	SREnum mode;
	SREnum Dummy;
}SRCS_DEF_PACKED iTeleoperSpeedMode_t;

typedef struct
{
	SRCS_APP_Header      header;
	iTeleoperSpeedMode_t     data;
}SRCS_DEF_PACKED iTeleoperSpeedModeFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: ------   Number of params: 2						  */
/*																			  */
/*    DIRECT    MODE, unit UIAdapter--RR--HMI--SystemControl                  */
/*############################################################################*/
#define STELEOPERCONFIG 0x0066


typedef struct
{
	e_ARM_Type          arm;
	SREnum              dummy;
	FunctionCode        HapDevType;//Value is enum {HapDevL,HapDevR}
}SRCS_DEF_PACKED STeleOperConfig_t;

typedef struct
{
	SRCS_APP_Header    header;
	controlDataCount   count;
	STeleOperConfig_t  data[e_ARM_SIZE];
}SRCS_DEF_PACKED STeleOperConfigFrame_t;


/*############################################################################*/
/*																			  */
/*			                      SMOTQS			   						  */
/*				Length: ------   Number of params: >=2						  */
/*																			  */
/*    DIRECT   MODE, Receiver is MonitorVD or MCU. Stop the Moving AXIS       */
/*############################################################################*/
#define SMOTQS    0x0022

typedef struct
{
	e_ARM_Type		 ArmType;
	SRCS_COUNT       itemCount; 			 
}SRCS_DEF_PACKED SMotQsHeader;

typedef struct
{
	T_MOVE_AXIS_TYPE axis;
}SRCS_DEF_PACKED SMotQsItem;

typedef struct
{
	SRCS_APP_Header   header;
	SMotQsHeader      MotQsHeader;
	SMotQsItem        item[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED SMOTQsFrame_t;


//Receiver is MonitorVD(SRNODE_MONITORVD)
#define SRESETINCENCCNT    0x0023
typedef struct
{
	e_ARM_Type          arm;
	T_MOVE_AXIS_TYPE    axis;
	SRCS_DUMMY          dummy;
	slong               ExtOffset;
}SRCS_DEF_PACKED RstMotIncEncCnt;
typedef struct{
	SRCS_DCOUNT      controlDataCount;      //<=e_MOVE_AXIS_COUNT
	RstMotIncEncCnt  axisOperate[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED SRstIncEncCnt_t;

typedef struct
{
	SRCS_APP_Header   header;
	SRstIncEncCnt_t   data;
}SRCS_DEF_PACKED SRstIncEncCntFrame_t;

#define SMOTHALT    0x0024
typedef struct{
	SRCS_DCOUNT      controlDataCount;      //<=e_MOVE_AXIS_COUNT
	T_MOVE_AXIS_TYPE axisOperate[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED SMOTHalt_t;

typedef struct
{
	SRCS_APP_Header header;
	SMOTHalt_t      data;
}SRCS_DEF_PACKED SMOTHaltFrame_t;


#define ACHKCURRENT 0x0031
typedef struct{
	SRCS_DCOUNT toBeCheckCount;
	T_MOVE_AXIS_TYPE axis[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED ACheckCurrent_t;

typedef struct
{
	SRCS_APP_Header header;
	ACheckCurrent_t data;
}SRCS_DEF_PACKED ACheckCurrentFrame_t;


/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 4    Number of params: 3	    					  */
/*																			  */
/*    BROADCAST MODE, unit MonitorVD-> SystemControl--UIAdapter               */
/*############################################################################*/
#define IARMPOS  0x0033

typedef struct
{
	e_ARM_Type       ArmType;
	ArmPos           pos;
	ArmMoveStatus    MoveStatus;
	SRCS_DUMMY       dummy;
}SRCS_DEF_PACKED IArmPos;

typedef struct
{
	SRCS_APP_Header header;
	IArmPos         data;
}SRCS_DEF_PACKED IArmPosFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 4    Number of params: 3	    					  */
/*																			  */
/*    Director MODE, Receiver is MonitorVD                                    */
/*############################################################################*/
#define SARMMOVING  0x0035
const SREnum ArmMovingTypeUn      = 0x00;//used as default value
const SREnum ArmMovingTypeForward = 0x55;
const SREnum ArmMovingTypeBack    = 0x66;
const SREnum ArmMovingTypeAuto    = 0xAA;//Decided by system, most case

const SREnum MoveCmdUndefine = 0x00;
const SREnum MoveCmdStart    = 0x55;
const SREnum MoveCmdStop     = 0xCD;

typedef struct
{
	e_ARM_Type       ArmType;
	SREnum           ArmMovingType;	
	SREnum           MoveCmd;
	SRCS_DUMMY       dummy;
}SRCS_DEF_PACKED SArmMoving;

typedef struct
{
	SRCS_APP_Header header;
	SArmMoving      data;
}SRCS_DEF_PACKED SArmMovingFrame_t;

#define SHOMING 0x0036

typedef struct
{	
	e_ARM_Type		 ArmType;	
	SRCS_COUNT       itemCount; 			 //0 indicates all the axis in the Arm shall perform Homing
}SRCS_DEF_PACKED SHomingHeader;

typedef struct
{
	T_MOVE_AXIS_TYPE axis;
}SRCS_DEF_PACKED SHomingItem;


typedef struct
{
	SRCS_APP_Header  header;
	SHomingHeader    homingHeader;
	SHomingItem      item[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED SHomingFrame_t;


/*############################################################################*/
/*																			  */
/*							IHOMING 										  */
/*				Length: 4    Number of params: 3	    					  */
/*																			  */
/*    Broadcast MODE,Receiver is SystemControlUnit, UIAdapter                 */
/*############################################################################*/
#define IHOMING 0x0037
const SREnum HomingStart    = 1;
const SREnum HomingComplete = 2;
const SREnum HomingFail     = 0xF;

typedef struct
{
	e_ARM_Type		 ArmType;	
	T_MOVE_AXIS_TYPE axis;
	SREnum           status;
}SRCS_DEF_PACKED IHoming_t;

typedef struct
{
	SRCS_APP_Header  header;
	IHoming_t        iHoming;
}SRCS_DEF_PACKED IHomingFrame_t;


/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 4    Number of params: 3	    					  */
/*																			  */
/*    Director MODE, Receiver is MonitorVD                                    */
/*############################################################################*/

#define SLOCK 0x0038

const SREnum ArmLockCMDUn   = 0x00;//used as default value
const SREnum ArmLockCMDOn   = 0x55;
const SREnum ArmLockCMDOff  = 0x66;
const SREnum ArmLockCMDAuto = 0xAA;//Decided by system, most case 

typedef struct
{
	e_ARM_Type		 ArmType;	
	SREnum           lock;
	SREnum           lockPolicy;//reserved for future use
	SREnum           dummy;
}SRCS_DEF_PACKED SLock_t;

typedef struct
{
	SRCS_APP_Header  header;
	SLock_t          sLock;
}SRCS_DEF_PACKED SLockFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 4    Number of params: 3	    					  */
/*																			  */
/*    Broadcast MODE, MonitorVD to other components                           */
/*############################################################################*/

#define ILOCK 0x0039

const SREnum LOCK_ST_LOCKED = 2;
const SREnum LOCK_ST_UNLOCKED = 0;
const SREnum LOCK_ST_INTERMEDIATE = 1;

typedef struct
{
	e_ARM_Type		 ArmType;	
	SREnum           lockPos;   
	//SREnum           lockStatus;//reached, or stop
	SREnum           dummy[2];
}SRCS_DEF_PACKED ILock_t;

typedef struct
{
	SRCS_APP_Header  header;
	ILock_t          iLock;
}SRCS_DEF_PACKED ILockFrame_t;

/*############################################################################*/
/*																			  */
/*							IMCUCOMM 										  */
/*				Length: 4    Number of params: 2	    					  */
/*																			  */
/*    Direct MODE,Receiver is SystemControlUnit			                      */
/*############################################################################*/
#define IMCUCOMM 0x0040
const SREnum MCUCOMMBuild = 1;
const SREnum MCUCOMMLost  = 2;

typedef struct
{
	e_ARM_Type		 ArmType;
	SREnum           status;
	SRCS_DUMMY       dummy[2];
}SRCS_DEF_PACKED IMCUComm_t;

typedef struct
{
	SRCS_APP_Header  header;
	IMCUComm_t       iMCUComm;
}SRCS_DEF_PACKED IMCUCommFrame_t;


#define IARMPOSANSWER  0x0133

typedef struct
{
	e_ARM_Type       ArmType;
	ArmPos           pos;
	ArmMoveStatus    MoveStatus;
	SRCS_DUMMY       dummy;
}SRCS_DEF_PACKED IArmPosAnswer;

typedef struct
{
	SRCS_APP_Header  header;
	controlDataCount itemCount;
	IArmPosAnswer    data[e_ARM_SIZE];
}SRCS_DEF_PACKED IArmPosAnswerFrame_t;
	
#define IMOTCURRENT 0x0032
typedef struct{
	T_MOVE_AXIS_TYPE axis;       //indicates Axis Type
	sshort           actCurrent; //actual current value, unit is mA
}SRCS_DEF_PACKED CurrentInfo;

typedef struct{
	SRCS_DCOUNT  toBeCheckCount;
	CurrentInfo currentInfo[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED IMotCurrent_t;

typedef struct
{
	SRCS_APP_Header header;
	IMotCurrent_t   data;
}SRCS_DEF_PACKED IMotCurrentFrame_t;


#define IHOMINGSTATUS 0x0041
const SREnum HomingMethodPositive = 1;//Negative Move-->Positve Move-->Homing Finshed, thus there is no Backlash when motor in Positve Move
const SREnum HomingMethodNegative = 2;//Positve Move-->Negative Move-->Homing Finshed, thus there is no Backlash when motor in Negative Move


typedef SREnum MotMoveState;
const MotMoveState MotStart   = 1;
const MotMoveState MotRunning = 2;
const MotMoveState MotReached = 3;
const MotMoveState MotFailed  = 4;


typedef struct
{
	T_MOVE_AXIS_TYPE axis;       //indicates Axis Type
	SREnum           method;     //HomingMethodPositive, HomingMethodNegative
	MotMoveState     status;
}SRCS_DEF_PACKED HomingStatus_t;

typedef struct
{
	SRCS_DCOUNT DataCount;
	HomingStatus_t status[e_MOVE_AXIS_COUNT];
}SRCS_DEF_PACKED IHomingStatus_t;

typedef struct
{
	SRCS_APP_Header   header;
	IHomingStatus_t   data;
}SRCS_DEF_PACKED IHomingStatusFrame_t;

#define SGENERALCMD 0x0042
typedef SREnum ConsoleType; //Indicates the Type of Console.
const ConsoleType ConsoleMasterPedel   = 5;
const ConsoleType ConsoleMasterUI      = 6;
const ConsoleType ConsoleMasterControl = 7;
const ConsoleType ConsoleSlaveUI       = 10;
const ConsoleType ConsoleHapDev        = 11;
const ConsoleType ConsoleArmE          = 16;
const ConsoleType ConsoleArm1		   = 17;
const ConsoleType ConsoleArm2		   = 18;
const ConsoleType ConsoleArm3		   = 19;

const ushort ConstrolConsoleParamPress   = 0x5555;
const ushort ConstrolConsoleParamRelease = 0x0;

typedef SREnum ControlElementType; //Indicates the Type of Control ELement.
const ControlElementType ON_OFF_BUTTON           = 1;
const ControlElementType PUSH_BUTTON             = 2;
const ControlElementType PLUS_MINUS_BUTTON       = 3;
const ControlElementType SELECTION_BUTTON        = 4;
const ControlElementType ON_OFF_LEVEL_BUTTON     = 5;//param =0x5555(PRESS) or 0(RELASE)
const ControlElementType POTENTIONMETER_BUTTON   = 6;


typedef struct
{
	ConsoleType 	   consoleType;
	ControlElementType controlElementType;
	FunctionCode       functionCode;
	ushort             param;
	SRCS_DUMMY         dummy[2];
}SRCS_DEF_PACKED SGeneralCmd;

typedef struct
{
	SRCS_APP_Header   header;
	SGeneralCmd       data;
}SRCS_DEF_PACKED SGeneralCmdFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 8 Byte                                                */
/*              Broadcast Mode                                                */
/*																			  */
/*############################################################################*/
#define ISYSTEMINFO 0x0063

typedef SREnum SystemNrType;

const SystemNrType SystemNrLEnd = 1;
const SystemNrType SystenNrREnd = 2;

typedef struct
{
	SystemNrType    systemNr;
	SRCS_DUMMY      dummy[3];
	SysStatusEnum   systemStatus;	
}SRCS_DEF_PACKED ISystemInfo_t;

typedef struct  
{
	SRCS_APP_Header  header;
	ISystemInfo_t    data;
}SRCS_DEF_PACKED ISystemInfoFrame_t;


/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 8 Byte                          					  */
/*																			  */
/*############################################################################*/

#define ITRACKSTATUS 0x0050
//System config not include this arm
const uchar         trackObjNotConfig  = 0;
const uchar         trackObjOK         = 1;
const uchar         trackObjFail       = 2;
const uchar			trackObjWarning    = 3;
//Tracked, but position doesn't match the config
const uchar         trackObjSetupError = 9;

typedef struct
{
	e_ARM_Type arm;
	uchar      trackStatus;
}SRCS_DEF_PACKED trackObjStatus;

typedef struct
{
	trackObjStatus trackObj[e_ARM_SIZE];
}SRCS_DEF_PACKED trackObjStatus_t;

typedef struct
{
	SRCS_APP_Header  header;
	trackObjStatus_t data;
}SRCS_DEF_PACKED trackObjStatus_Frame_t;
/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 2 Byte                          					  */
/*	  action:		CALIBRATION_START/	CALIBRATION_STOP					  */
/*############################################################################*/
#define SCALIBRATIONSTATUS 0x0070  //calibration
const uchar CALIBRATION_ACTION_START = 0x01;
const uchar CALIBRATION_ACTION_STOP = 0x00;
const uchar TOUCHX_1 = 0x00;
const uchar TOUCHX_2 = 0x01;
const uchar TOUCHX_3 = 0x02;
const uchar TOUCHX_4 = 0x03;
const uchar TOUCHX_SIZE = 0x04;

typedef struct
{
	SREnum action;
	uchar      dummy;
}SRCS_DEF_PACKED scalibrationStatus;

typedef struct
{
	SRCS_APP_Header  header;
	scalibrationStatus data;
}SRCS_DEF_PACKED scalibrationStatus_Frame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 6 Byte                          					  */
/*	  Status:  CALIBRATION_STATUS_IDLE                                        */
/*             CALIBRATION_STATUS_OK                                          */
/*			   CALIBRATION_STATUS_ERROR				                          */
/*############################################################################*/
#define ICALIBRATIONSTATUS 0x0071  //calibration

const uchar CALIBRATION_STATUS_IDLE = 0x00;
const uchar CALIBRATION_STATUS_OK = 0x01;
const uchar CALIBRATION_STATUS_ERROR = 0x02;

typedef struct
{
	SREnum     Status[4];
	uchar      dummy[2];
}SRCS_DEF_PACKED icalibrationStatus;

typedef struct
{
	SRCS_APP_Header  header;
	icalibrationStatus data;
}SRCS_DEF_PACKED icalibrationStatus_Frame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 16 Byte   Number of params: 3						  */
/*																			  */
/*    error from a component sent to System Control in order to log and or    */
/*    display errors, warnings, information etc.                              */
/*    Using the same telegram an error can be set and also reset by component.*/ 
/*    For usual an error - if displayed is confirmed by user and			  */
/*    reset by SC sending SRESETERROR										  */
/*    REMARK: Definition of category please refer  Error Category Definition  */
/*############################################################################*/

#define IERROR 0x0101
typedef struct
{
	enum16 errorNumber;
	SREnum category;
	uchar  addtionalInfo[MAX_ADDI_INFO_SIZE];
}SRCS_DEF_PACKED IError;

typedef struct
{
	SRCS_APP_Header   header;
	IError            data;
}SRCS_DEF_PACKED IErrorFrame_t;

/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 16 Byte   Number of params: 3						  */
/*																			  */
/*    error from a component sent to System Control in order to log and or    */
/*    display errors, warnings, information etc.                              */
/*    Using the same telegram an error can be set and also reset by component.*/ 
/*    For usual an error - if displayed is confirmed by user and			  */
/*    reset by SC sending SRESETERROR										  */
/*############################################################################*/
#define IERRORANSWER 0x0102
#define MAX_SUPPORT_ASCII_LEN 64
typedef struct  
{
	enum16    errorNumber;
	uchar     SRNodeID;
	SREnum    category;
	uchar  addtionalInfo[MAX_ADDI_INFO_SIZE+1];
}SRCS_DEF_PACKED IErrorAnswer;

typedef struct
{
	SRCS_APP_Header   header;
	IErrorAnswer       data;
}SRCS_DEF_PACKED IErrorAnswerFrame_t;
/*############################################################################*/
/*																			  */
/*																			  */
/*				Length: 0    Number of params: 0	    					  */
/*																			  */
/*    BROADCAST MODE, unit SystemControl-> OTHER NODES                        */
/*############################################################################*/

#define SRESETERROR 0x0103
typedef struct
{	
}SRCS_DEF_PACKED SResetError;

typedef struct
{
	SRCS_APP_Header   header;
	SResetError       data;
}SRCS_DEF_PACKED SResetErrorFrame_t;

#define IRECCONFRIM 0x0104

#define TMOTSWITCHER 0x2001

typedef struct
{
	SREnum ifStart;
	uchar joinTestCnt;
	ushort testCycle;
	slong  testVelocity;
}SRCS_DEF_PACKED TMotSwitcher;

typedef struct
{
	SRCS_APP_Header   header;
	TMotSwitcher   data;
}SRCS_DEF_PACKED TMotSwitcherFrame_t;


#define IUI_RADIOBUTTON_CMD 0x0231
const enum16 UIFUNC_UNDEFINE      = 0xFFFF;
const enum16 UIFUNC_ENDOSCOPE_MOVE_FUNC  = 0x1000;
const enum16 UIFUNC_ARM1_MOVE_FUNC       = 0x1001;
const enum16 UIFUNC_ARM2_MOVE_FUNC 	 	 = 0x1002;
const enum16 UIFUNC_ARM3_MOVE_FUNC 		 = 0x1003;	

const ushort UIParamMButtonInit         = 0;
const ushort UIParamMButtonFeeding      = 1;
const ushort UIParamMButtonExitExChange = 2;
const ushort UIParamMButtonStraight     = 3;
const ushort UIParamMButtonExiting      = 4;
typedef struct 
{
	enum16 FuncID;
	ushort Param;	
}SRCS_DEF_PACKED UiRadioButtonCmd;

typedef struct
{
	SRCS_APP_Header   header;
	UiRadioButtonCmd  data;
}SRCS_DEF_PACKED UiRadioButtonCmdFrame_t;


#define IUI_TABCARD_CMD 0x0232


#define IUI_IMAGEVIEW_CMD 0x0233
typedef struct
{
	enum16 FuncID;
	ushort Param;
}SRCS_DEF_PACKED UiImageViewCmd;

typedef struct
{
	SRCS_APP_Header   header;
	UiImageViewCmd  data;
}SRCS_DEF_PACKED UiImageViewCmdFrame_t;

#define IUI_EVENTLOGWIDGET_CMD 0x0234
typedef struct
{
	enum16 FuncID;
	ushort Param;
}SRCS_DEF_PACKED UiEventLogWidgetCmd;

typedef struct
{
	SRCS_APP_Header   header;
	UiEventLogWidgetCmd  data;
}SRCS_DEF_PACKED UiEventLogWidgetCmdFrame_t;
typedef enum
{
	UI_TABCARD_REGISTER = 0x5001,
	UI_TABCARD_PREPAR = 0x5002,
	UI_TABCARD_OPERATE = 0x5003,
}UI_TABCARD_LIST;

typedef struct
{
	UI_TABCARD_LIST tabCard;
}SRCS_DEF_PACKED UiTabCardSel;

typedef struct
{
	SRCS_APP_Header   header;
	UiTabCardSel      data;
}SRCS_DEF_PACKED UiTabCardSelFrame_t;


#define IUI_LOGIN_CMD 0x0235
#define IUI_MAX_USERNAME_LEN 32
#define IUI_MAX_PSW_LEN 32
typedef struct
{
	enum16        FuncID;
	char          userName[IUI_MAX_USERNAME_LEN];
	char          passWord[IUI_MAX_PSW_LEN];
}SRCS_DEF_PACKED UiLoginCmd;

typedef struct
{
	SRCS_APP_Header   header;
	UiLoginCmd  data;
}SRCS_DEF_PACKED UiLoginCmdFrame_t;


#define IUI_COMBOBOXSELECT_CMD 0x0236
typedef struct
{
	enum16 FuncID;
	ushort Param;
}SRCS_DEF_PACKED UiComboBoxSelectCmd;
typedef struct
{
	SRCS_APP_Header   header;
	UiComboBoxSelectCmd  data;
}SRCS_DEF_PACKED UiComboBoxSelectCmdFrame_t;



#define IUI_SOUNDRECORDWIDGET_CMD 0x0237
typedef struct
{
	enum16 FuncID;
	ushort Volume;
	uchar  Status;
	uchar  Dummy;
}SRCS_DEF_PACKED UiSoundRecordWidgetCmd;
typedef struct
{
	SRCS_APP_Header   header;
	UiSoundRecordWidgetCmd  data;
}SRCS_DEF_PACKED UiSoundRecordWidgetCmdFrame_t;



#define IUI_WORKSPACEDWIDGET_CMD 0x0238
typedef struct
{
	enum16 FuncID;
	ushort Param;
}SRCS_DEF_PACKED UiWorkspaceWidgetCmd;
typedef struct
{
	SRCS_APP_Header   header;
	UiWorkspaceWidgetCmd  data;
}SRCS_DEF_PACKED UiWorkspaceWidgetCmdFrame_t;



#define IUI_WIDGET_SHOW 0x0201
#define UI_WIDGET_SIZE 16
const uchar UI_SHOW_ACK_ON = 1;
const uchar UI_SHOW_ACK_OFF = 0;

typedef uchar WidgetStatus;
const WidgetStatus WidgetStHide = 0x00;
const WidgetStatus WidgetStNotSelect = 0x01;
const WidgetStatus WidgetStSelect = 0x03;
const WidgetStatus WidgetStDimm = 0x05;
const WidgetStatus WidgetStReadonly = 0x07;
const WidgetStatus WidgetStFlicker = 0x0F;

typedef struct
{
	ushort obj_count;
	uchar  ui_ack;      //1 indicates ACK is necessary after UI receive the packet.Most case is 0.
	uchar  dummy;
}SRCS_DEF_PACKED IUiWidgetItemHeader;

typedef struct
{
	enum16        FuncID;
	sshort        Param0; //0.1 degree
	sshort        Param1;
	//uchar         dummy;
}SRCS_DEF_PACKED IUiWidgetItem;

typedef struct
{
	SRCS_APP_Header     header;
	IUiWidgetItemHeader ItemHeader;
	IUiWidgetItem       Item[UI_WIDGET_SIZE];
}SRCS_DEF_PACKED IUiWidgetShowFrame_t;


#define IUI_TABCARD_SHOW 0x0202

typedef enum
{
	UI_TABCARD_STATUS_SEL = 0x0001,
	UI_TABCARD_STATUS_DESEL = 0x0002,
	UI_TABCARD_STATUS_DIMM = 0x0004,
}UI_TABCARD_STATUS;

typedef struct
{
	UI_TABCARD_LIST   tabCard;
	UI_TABCARD_STATUS status;
}UiTabcardShow;

typedef struct
{
	SRCS_APP_Header   header;
	UiTabcardShow     data;
}SRCS_DEF_PACKED UiTabcardShowFrame_t;


#define IUI_RADIOBUTTON_SHOW 0x0203



typedef struct
{
	enum16        FuncID;
	ushort        Param; //0.1 degree
	WidgetStatus  Status;
	uchar         dummy;
}SRCS_DEF_PACKED IUiRadiobuttonItem;

typedef struct
{
	SRCS_APP_Header     header;
	IUiWidgetItemHeader ItemHeader;
	IUiRadiobuttonItem       Item[UI_WIDGET_SIZE];
}SRCS_DEF_PACKED IUiRadiobuttonShowFrame_t;


#define IUI_ORTMATCH_SHOW 0x0204
typedef struct
{
	enum16        FuncID;
	sshort        target; //0.1 degree
	sshort        current;
	TeleOperMatchStatus matchstatus;
	uchar         dummy;
}SRCS_DEF_PACKED IUiOrtMatchItem;

typedef struct
{
	SRCS_APP_Header     header;
	IUiWidgetItemHeader ItemHeader;
	IUiOrtMatchItem       Item[UI_WIDGET_SIZE];
}SRCS_DEF_PACKED IUiOrtMatchShowFrame_t;

#define IUI_GRPMATCH_SHOW 0x0205
typedef struct
{
	enum16        FuncID;
	sshort        target; //0.1 degree
	sshort        current;
	TeleOperMatchStatus matchstatus;
	uchar         dummy;
}SRCS_DEF_PACKED IUiGrpMatchItem;

typedef struct
{
	SRCS_APP_Header     header;
	IUiWidgetItemHeader ItemHeader;
	IUiGrpMatchItem       Item[UI_WIDGET_SIZE];
}SRCS_DEF_PACKED IUiGrpMatchShowFrame_t;




#define IUI_IMAGEVIEW_SHOW 0x0206
typedef struct
{
	enum16        FuncID;
	ushort        Param0;
	ushort        Param1;
}SRCS_DEF_PACKED IUiImageViewItem;

typedef struct
{
	SRCS_APP_Header     header;
	IUiWidgetItemHeader ItemHeader;
	IUiImageViewItem       Item[UI_WIDGET_SIZE];
}SRCS_DEF_PACKED IUiImageViewShowFrame_t;


#define IUI_ERROR_INFO 0x0208
#define IUI_MAX_SUPPORT_ASCII_LEN  MAX_SUPPORT_ASCII_LEN*2
typedef struct
{
	enum16        FuncID;
	enum16        errorNumber;
	uchar         SRNodeID;
	SREnum        category;
	char          text[IUI_MAX_SUPPORT_ASCII_LEN];
}SRCS_DEF_PACKED IUiErrorInfo;

typedef struct
{
	SRCS_APP_Header   header;
	IUiErrorInfo       data;
}SRCS_DEF_PACKED IUiErrorInfoFrame_t;

#define IUI_EVENTLOGDATAILS_SHOW 0x0209
typedef struct
{

}SRCS_DEF_PACKED IUiEventDatailsShow;

typedef struct
{
	SRCS_APP_Header   header;
	IUiEventDatailsShow       data;
}SRCS_DEF_PACKED IUiEventDatailsShowFrame_t;

#define IUI_STATUSBAR_SHOW 0x0210
#define STATUSBAR_READY 1
#define STATUSBAR_NOT_READY 0
typedef struct
{
	enum16        FuncID;
	uchar         ready;
	char          text[IUI_MAX_SUPPORT_ASCII_LEN];
	uchar         dummy;
}SRCS_DEF_PACKED IUiStatusbarShow;

typedef struct
{
	SRCS_APP_Header   header;
	IUiStatusbarShow  data;
}SRCS_DEF_PACKED IUiStatusbarShowFrame_t;

#define IUI_COMBOBOX_SHOW 0x0211
#define COMBOBOXITEMCLEAR 0x01
#define COMBOBOXITEMADD   0x02
#define COMBOBOXITEMCLEARBEFOREADD 0x03
#define COMBOBOXSETINDEX 0x04
#define COMBOBOXSETCURRENTTEXT 0x05
#define MAX_COMBOBOX_ITEMNAME_LEN 32
#define MAX_COMBOBOX_ITEM_COUNT 3
typedef struct{
	char Name[MAX_COMBOBOX_ITEMNAME_LEN];
}IUiComboBoxItemNmae;

typedef struct{
	enum16                FuncID;
	SREnum                Action;
	SREnum                Dummy;
	ushort                SeleIndex;	
	ushort                ItemCount;
	IUiComboBoxItemNmae   Names[MAX_COMBOBOX_ITEM_COUNT];
}IUiComboBoxShow;

typedef struct{
	SRCS_APP_Header   header;
	IUiComboBoxShow   data;
}IUiComboBoxShowFrame_t;


#define IUI_STATICLABLE_SHOW 0x0212
#define MAX_STATICLABLE_TEXT_LEN 64
#define MAX_WARNNING_STATICLABLE_TEXT_LEN 32

typedef struct{
	enum16                FuncID;
	char                  Text[MAX_STATICLABLE_TEXT_LEN];
}IUiStaticLableShow;


typedef struct{
	SRCS_APP_Header      header;
	IUiStaticLableShow   data;
}IUiStaticLableShowFrame_t;



//login widget show
#define IUI_LOGINWIDGET_SHOW 0x0213
const SREnum LOGIN_STATUS_OK = 0x01;
const SREnum LOGIN_STATUS_PASSWORDERORR = 0x02;
const SREnum LOGIN_STATUS_USERNAMEERORR = 0x03;

typedef struct{
	enum16                FuncID;
	SREnum                Status;
	SREnum                Dummy;
}IUiLoginwidegtShow;


typedef struct{
	SRCS_APP_Header   header;
	IUiLoginwidegtShow   data;
}IUiLoginwidegtShowFrame_t;


//event log widget show
#define IUI_EVENTLOGWIDGET_SHOW 0x0214
#define MAX_EVENTLOG_SUPORT
const SREnum EVENTLOG_CMD_STARTOPR = 0x01;
const SREnum EVENTLOG_CMD_INSERTMSG = 0x02;
const SREnum EVENTLOG_CMD_SETOPRNAME = 0x03;
typedef struct{
	enum16                FuncID;
	SREnum                Cmd;
	SREnum                category;
	char                  Msg[IUI_MAX_SUPPORT_ASCII_LEN];
}IUiEventLogwidegtShow;
typedef struct{
	SRCS_APP_Header   header;
	IUiEventLogwidegtShow   data;
}IUiEventLogwidegtShowFrame_t;


#define IUI_SOUNDRECORDWIDGET_SHOW 0x0215 
typedef struct
{
	enum16 FuncID;
	ushort Volume;
	uchar  Status;
	uchar  Dummy;
}SRCS_DEF_PACKED UiSoundRecordWidgetShow_t;
typedef struct
{
	SRCS_APP_Header   header;
	UiSoundRecordWidgetShow_t  data;
}SRCS_DEF_PACKED UiSoundRecordWidgetShowFrame_t;

#define IUI_WORKSPACEWIDGET_SHOW 0x0216 
typedef struct
{
	enum16 FuncID;
	TransformMatrixCount_t   count;
	TransformMatrix_Data_t TransformArray[3];
}SRCS_DEF_PACKED UiWorkSpaceWidgetShow_t;
typedef struct
{
	SRCS_APP_Header   header;
	UiWorkSpaceWidgetShow_t  data;
}SRCS_DEF_PACKED UiWorkSpaceWidgetShowFrame_t;


#define IUI_OPENGL3DWIDGET_SHOW 0x0217 
typedef struct
{
	enum16 FuncID;
	char   msg[MAX_BUFFER];
}SRCS_DEF_PACKED UiOpenGL3DWidgetShow_t;
typedef struct
{
	SRCS_APP_Header   header;
	UiOpenGL3DWidgetShow_t  data;
}SRCS_DEF_PACKED UiOpenGL3DWidgetShowFrame_t;
//StatusInfoID
#define READY_FOR_TELEOPERATION 0x0301
#define NO_ARM_IS_SELECTED 0x0302
#define MCU_CONNECT_STATE 0x0400
#define MCU_COMM_STATE 0x0401
#define ITELEOPER_STATE 0x0500
#define ARM_MOVE_DISABLE 0x0520
#define ARM_TRACKER_LOST 0x0521
#define ARM_DISABLE_DMG 0x0522
#define ARM_EMERGENCY_STOP_PRESSED 0x0523
#define ARM_TRUNDLE_NOTFOLD 0x0524
#ifdef SR_WIN_NT_SUPPORT
#pragma pack (pop)
#endif 

#endif /*SRCS_OBJ_H*/
