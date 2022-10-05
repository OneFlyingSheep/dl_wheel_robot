#pragma once
#ifndef __SOCK_GENERAL_DEF_H__
#define __SOCK_GENERAL_DEF_H__

#define MSG_HEADER_LENGTH 16
#define ByteCast(x) ((uint8_t)(x))

#define MSG_SYNC_ID 0x5A
#define MSG_VERSION 0x01

#define QUERY_SESSION_ALLOWED 10
#define OTHER_SESSION_ALLOWED 1

#define GET_VARIABLE_STRING(name) #name

//#define MAX_BUFF_LENGTH 64 * 1024

#include <stdint.h>
#include <iostream>

#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>
#include <string>
#include <string.h>
#include <stdio.h>

#include <stdint.h>

extern uint16_t uint8_t_to_uint16( uint8_t *b);
extern void uint16_t_to_uint8( uint16_t b, uint8_t *p);
extern uint32_t uint8_t_to_uint32( uint8_t *b);
extern void uint32_t_to_uint8(uint32_t b, uint8_t *p);



struct    ProtocolHeader    {
    uint8_t         m_sync;
    uint8_t         m_version;
    uint16_t        m_number;
    uint32_t        m_length;
    uint16_t        m_type;
    uint8_t         m_reserved[6];
};

typedef enum {
    SOCK_TYPE_CLIENT,
    SOCK_TYPE_SERVER,

    SOCK_TYPE_SUM,
}E_SOCKET_TYPE;

typedef enum{
    BLOCK_ULTRASONIC   = 0,
    BLOCK_LASER        = 1,
    BLOCK_FALLING_DOWN = 2,
    BLOCK_COLLISION    = 3,
    BLOCK_INFRARED     = 4
}RobokitProtocolBlockReason;

typedef enum{
    TASK_NONE      = 0,
    TASK_WAITING   = 1,
    TASK_RUNNING   = 2,
    TASK_SUSPENDED = 3,
    TASK_COMPLETED = 4,
    TASK_FAILED    = 5,
    TASK_CANCELED  = 6
}RobokitProtocolTaskStatus;

typedef enum{
    NONE_TASK          = 0,
    FREE_TO_ANY        = 1,
    FREE_TO_STATION    = 2,
    REGULAR_TO_STATION = 3,
    PATROL_TASK        = 4,
    OTHER              = 100
}RobokitProtocolTaskType;

typedef enum{
    RELOC_FAILED    = 0,
    RELOC_SUCCESS   = 1,
    RELOC_RELOCING  = 2,
    RELOC_COMPLETED = 3
}RobokitProtocolRelocStatus;

typedef enum{
    LOAD_FAILED    = 0,
    LOAD_SUCCESS   = 1,
    LOAD_LOADING   = 2
}RobokitProtocolLoadmapStatus;

typedef enum{
    INIT_FAILED    = 0,
    INIT_SUCCESS   = 1,
    INIT_INITING   = 2
}RobokitProtocolInitStatus;

typedef enum{
    MODE_MANUAL  = 0,
    MODE_AUTO    = 1
}RobokitProtocolMode;

typedef enum{
    ROBOT_MOVE_STOP = 0,
    ROBOT_MOVE_FORWARD,
    ROBOT_MOVE_BACKWARD,
}RobotMoveMode;

typedef enum{
    LIFT_MOVE_STOP = 0,
    LIFT_MOVE_DOWN,
    LIFT_MOVE_UP,
}RobotLiftMode;

typedef enum{
    CAM_PTZ_MOVE_STOP = 0,
    CAM_PTZ_MOVE_UP,
    CAM_PTZ_MOVE_DOWN,
}RobotCamPtzMode;

typedef enum{
    ROBOT_BODY_MOVE_STOP = 0,
    ROBOT_BODY_MOVE_LEFT,
    ROBOT_BODY_MOVE_RIGHT,
}RobotBodyRotateMode;

typedef enum
{
    ROBOT_PD_STOP = 0,
    ROBOT_PD_COLLECT,
    ROBOT_PD_WITHDRAW,
}RobotPartialDischargeOper;

typedef enum{
    ROBOT_PD_MOVE_STOP = 0,
    ROBOT_PD_MOVE_REACHOUT,
    ROBOT_PD_MOVE_WITHDRAW,
}RobotBodyPDMode;

typedef enum {
    ROBOT_PD_ARM_STOP = 0,
    ROBOT_PD_ARM_UP,
    ROBOT_PD_ARM_DOWN,
}RobotBodyPDArm;

typedef enum {
    ROBOT_WARNLIGHT_STOP = 0,
    ROBOT_WARNLIGHT_GREEN,
    ROBOT_WARNLIGHT_RED,
    ROBOT_WARNLIGHT_BLUE,
}RobotBodyWarnLight;

typedef enum {
    ROBOT_CPU = 0,
    ROBOT_SENSOR,
}RobotBodyUpdateEmbeddedSoftwareType;

typedef enum{
    RobotStatusOther     = 0,   //No means, just for Log analysis
    RobotStatusInfo      = 1000,
    RobotStatusRun       = 1002,
    RobotStatusMode      = 1003,
    RobotStatusLoc       = 1004,
    RobotStatusSpeed     = 1005,
    RobotStatusBlock     = 1006,
    RobotStatusBattery   = 1007,
    RobotStatusBrake     = 1008,
    RobotStatusLaser     = 1009,
    RobotStatusPath      = 1010,
    RobotStatusAera      = 1011,
    RobotStatusEmergency = 1012,
    /*add by Gallon 20170515 start*/
    RobotStatusIo        = 1013,
    /*add by Gallon 20170515 end*/
    RobotStatusTask      = 1020,
    RobotStatusReloc     = 1021,
    RobotStatusLoadmap   = 1022,
    RobotStatusSlam      = 1025,
    RobotStatusAlarm     = 1050,
    RobotStatusAll1      = 1100,
    RobotStatusAll2      = 1101,
    RobotStatusAll3      = 1102,
    RobotStatusInit      = 1111,
    RobotStatusMap       = 1300,
    RobotStatusParams    = 1400
}RobokitProtocolStatusInfo;

typedef enum {
    /*Robot Status ControlCode*/
    Request_RobotStatusInfo = 1000,
    Request_RobotStatusRun = 1002,
    Request_RobotStatusMode = 1003,
    Request_RobotStatusLoc = 1004,
    Request_RobotStatusSpeed = 1005,
    Request_RobotStatusBlock = 1006,
    Request_RobotStatusBattery = 1007,
    Request_RobotStatusBrake = 1008,
    Request_RobotStatusLaser = 1009,
    Request_RobotStatusPath = 1010,
    Request_RobotStatusArea = 1011,
    Request_RobotStatusEmemergency = 1012,
    /*add by Gallon 20170515 start*/
    Request_RobotStatusIo = 1013,
    /*add by Gallon 20170515 end*/
    Request_RobotStatusTask = 1020,
    Request_RobotStatusReloc = 1021,
    Request_RobotStatusLoadmap = 1022,
    Request_RobotStatusTracking = 1024,
    Request_RobotStatusSlam = 1025,
    Request_RobotStatusAlarm = 1050,
    Request_RobotStatusAll1 = 1100,
    Request_RobotStatusAll2 = 1101,
    Request_RobotStatusAll3 = 1102,
    Request_RobotStatusInit = 1111,
    Request_RobotStatusMap = 1300,
    Request_RobotStatusParams = 1400,
    Request_RobotCurrentPoint = 1410,

    /*Robot Control ControlCode*/
    Request_RobotControlStop = 2000,
    Request_RobotControlGyrocal = 2001,
    Request_RobotControlReloc = 2002,
    Request_RobotControlComfirmloc = 2003,
	/*Robot cloud ControlCode*/
	Request_RobotControlPtzMotionReq = 2004,
	Request_RobotControlPtzAbsReq = 2005,

    Request_RobotControlMotion = 2010,
    Request_RobotControlGoTarget = 2011,
    Request_RobotControlTranslate = 2012,
    Request_RobotControlTurn = 2013,

    Request_RobotControlSlam = 2020,
    Request_RobotControlEndslam = 2021,
    Request_RobotControlLoadmap = 2022,
    Request_RobotControlLoadmapobj = 2023,

    Request_RobotControlSpeaker = 2030,
    Request_RobotControlMovePoint = 2039,
	
    /*Robot Task ControlCode*/
    Request_RobotTaskPause    =    3001,
    Request_RobotTaskResume    =    3002,
    Request_RobotTaskCancel    =    3003,

    Request_RobotTaskAssign = 3010,
    Request_RobotTaskDelete = 3011,

    Request_RobotTaskQueryCurrent = 3020,
    Request_RobotTaskQueryNext = 3021,
    Request_RobotTaskGohome    =    3030,
    Request_RobotTaskCharge    =    3040,
    Request_RobotTaskGopoint    =    3050,
    Request_RobotTaskGotarget    =    3051,
    /*add by Gallon 20170515 start*/
    Request_RobotTaskPatrol    =    3052,
    /*add by Gallon 20170515 end*/
    Request_RobotTaskTranslate    =    3055,
    Request_RobotTaskTurn    =    3056,
    Request_RobotTaskFollow    =    3070,

    /*Robot Config ControlCode*/
    Request_RobotConfigMode    =    4000,
    Request_RobotConfigUploadmap    =    4010,
    Request_RobotConfigDownloadmap    =    4011,
    Request_RobotConfigRemovemap    =    4012,
    Request_RobotConfigDownload2d    =    4020,
    Request_RobotConfigSetparams    =    4100,
    Request_RobotConfigSaveparams    =    4101,
    Request_RobotConfigReloadparams    =    4102,

    /*Robot Core ControlCode*/
    Request_RobotCoreShutdown    =    5000,
    Request_RobotCoreStop    =    5001,
    Request_RobotCoreStart    =    5002,
    Request_RobotCoreReboot    =    5003,
    Request_RobotCoreRunning    =    5004,
    /*add by Gallon 20170515 start*/
    Request_RobotCoreResetdsp    =    5005,
    /*add by Gallon 20170515 end*/
    Request_RobotCoreSearchScript    =    5010,
    Request_RobotCoreChangescript    =    5011,
    Request_RobotCoreUploadScript    =    5012,
    Request_RobotCoreDownloadScript    =    5013,

    /*Robot Other ControlCode*/
    Request_RobotOtherSpeaker    =    6000,

    /* hang robot Ctrl Request Code */
    Request_robot_ctrl_to_point_req = 7000,
    Request_robot_ctrl_move_abs_req = 7001,
    Request_robot_ctrl_move_req = 7002,
    Request_robot_ctrl_lift_abs_req = 7003,
    Request_robot_ctrl_lift_req = 7004,
    Request_robot_ctrl_cam_ptz_abs_req = 7005,
    Request_robot_ctrl_cam_ptz_req = 7006,
    Request_robot_ctrl_body_abs_req = 7007,
    Request_robot_ctrl_body_req = 7008,
    Request_robot_ctrl_partialdischarge_req = 7009,
    Request_robot_ctrl_man_pd_req = 7010,
    Request_robot_ctrl_man_to_point_req = 7011,
    Request_robot_ctrl_pd_ptz_req = 7012,
    Request_robot_ctrl_pd_collect_req = 7013,
    Request_robot_ctrl_status_light_req = 7014,
    Request_robot_ctrl_zero_req = 7015,
    Request_robot_ctrl_emergency_stop_req = 7016,
    Request_robot_ctrl_warning_light_req = 7017,
    Request_robot_ctrl_pd_reset_req = 7018,
	Request_robot_ctrl_walk_threshold = 7019,

    Request_robot_keep_alive = 9999,
}RobokitProtocolReqCode;

typedef enum
{
    Query_robot_status_on_node = 8000,
    Query_robot_status_offset = 8001,
    Query_robot_status_lift = 8002,
    Query_robot_status_cam_ptz = 8003,
    Query_robot_status_body_rotate = 8004,
    Query_robot_status_pd = 8005,
    Query_robot_status_snoise = 8006,
    Query_robot_status_speed = 8007,
    Query_robot_status_stopBtn = 8010,
    Query_robot_status_pd_arm = 8011,
    Query_robot_status_hardware_status = 8012,
    Query_robot_status_version = 8013,
    Query_robot_motor_status_debug,
    Query_robot_sick_status_debug,
}RobokitProtocolQueryCode;

typedef enum{
    ReqUnavailable    =    40000,
    ParamMissing    =    40001,
    ParamTypeError    =    40002,
    ParamIllegal    =    40003,
    ModeError    =    40004,
    IllegalMapName    =    40005,
    ProgrammingDsp    =    40006,
    ProgramDspError    =    40007,
    ShutdownError    =    40010,
    RebootError        =    40011,
    MapParseError    =    40050,
    MapNotExists    =    40051,
    LoadMapError    =    40052,
    LoadMapobjError    =    40053,
    EmptyMap    =    40054,
    ReqTimeout    =    40100,
    ReqForbidden    =    40101,
    RobotBusy    =    40102,
    RobotInternalError    =    40199,
    InitStatusError    =    41000,
    LoadmapStatusError    =    41001,
    RelocStatusError    =    41002,

    ParseError = 60000,
}RobokitProtocolRobotRetCode;

enum RobokitProtocolRobotAlarmCode
{
    LaserLost    =    50000,
    GyroLost    =    50001
};

enum RobokitProtocolRobotErrorCode
{
    RobotErrorWtype    =    60000,
    RobotErrorUtype    =    60001,
    RobotErrorData    =    60002,
    RobotErrorVersion    =    60003,
    RobotErrorVugedata    =    60004
};

typedef struct str_SocketCheckData
{
    int id;
    int type;
    time_t time;
    bool bRet;
}STRU_SOCK_CHECK_DATA;

#endif
