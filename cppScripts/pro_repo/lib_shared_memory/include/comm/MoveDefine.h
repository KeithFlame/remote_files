/*
* MoveDefine.h
*
*  Created on: 2017-04-31
*      Author: Gong Hui
*/

#ifndef MOVEDEFINE_H_
#define MOVEDEFINE_H_
#include "portab.h"
#if defined(SR_FREE_RTOS_SUPPORT)
#define MAX_SP_CO_NUMBER 9//Drive*9
#else
#if defined(SR_MASTERDEV_MANAGER)
#define MAX_SP_CO_NUMBER 1//Master dev manager
#else
#define MAX_SP_CO_NUMBER 4//MonitorVD*4
#endif
#endif

#include "arix_def.h"

#define DEV_MAX_FAIL_COUNT     12
const unsigned long INVALID_POS_ENC_VALUE = 0x7FFFFFFF;

#define ARM_T_AXIS_MASK 0xFF  //0x1FF if Grip 2 is config
#define ARM_E_AXIS_MASK 0x7F
#define GENERAL_ARM_AXIS_MASK(ARM) ((ARM==e_ARM_E)?ARM_E_AXIS_MASK:ARM_T_AXIS_MASK)
#define ARM_STATUS_RESET_MASK 0x80000000
#define ARM_STATUS_CLEAR_MASK 0x40000000   //if bit 30 is set, indicates reset this state
//define the System support motor move axis type
//typedef enum16

typedef SREnum e_ARM_Type;

const e_ARM_Type e_ARM_E                  = 0;//Endoscope Arm
const e_ARM_Type e_ARM_1                  = 1;//Tool Arm 1
const e_ARM_Type e_ARM_2                  = 2;//Tool Arm 2
const e_ARM_Type e_ARM_3                  = 3;//Tool Arm 3
const e_ARM_Type e_ARM_SIZE               = 4;//Support 4 Arms currently, also used as init value for variant
const e_ARM_Type e_ARM_Unify              = 0xFF;//Not certain arm, depends on state machine.


const SREnum SG_Board = 5;
const SREnum IO_Board = 6;

typedef SREnum ArmPos;

#define POSinPOINTMASK 0x01
const ArmPos ArmPosUncertain      = 0;
const ArmPos ArmRemoteEnd         = 1;    
const ArmPos ArmOutIntermediate   = 2;
const ArmPos ArmNearSkin          = 3;
const ArmPos ArmInnerIntermediate = 4;
const ArmPos ArmInnerLimit        = 5;
const ArmPos ArmInBody            = 7;
const ArmPos ArmStaright          = 6;
const ArmPos ArmReadyForTeleOper  = 8;
const ArmPos ArmTeleOper          = 10;
const ArmPos ArmMoveHalt          = 15;
const ArmPos ArmInitalValue       = 0xFF;



typedef SREnum ArmMoveCmdType;

//Currently Only support ArmMoveCmdEna, ArmMoveCmdDisable
const ArmMoveCmdType ArmMoveCmdUndefine = 0;
const ArmMoveCmdType ArmMoveCmdEna      = 1;
const ArmMoveCmdType ArmMoveCmdDisable  = 3;


typedef SREnum ArmInnerMoveCmdType;
const ArmInnerMoveCmdType ArmMoveCmdStart = 0x55;
const ArmInnerMoveCmdType ArmMoveCmdKeep = 0xAA;

#define ARM_PREPARE_ACTION ((1<<ArmWorkingUnknown)|(1<<ArmWorkingCommCheck)\
                           |(1<<ArmWorkingHoming)|(1<<ArmWorkingSetupToolAntiB)\
                           |(1<<ArmWorkingSetupTrocarAntiB)|(1<<ArmWorkingSetupToolMarker)\
						   |(1<<ArmWorkingSetupTrocar)|(1<<ArmWorkingSetupArmTool))

#endif /* MOVEDEFINE_H_ */
