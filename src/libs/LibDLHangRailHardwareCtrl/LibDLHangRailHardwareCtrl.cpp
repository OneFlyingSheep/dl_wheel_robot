#include "LibDLHangRailHardwareCtrl.h"
#include "common/DLhangRailRobotFSM.hpp"


DLHangRailHardwareStatus::DLHangRailHardwareStatus()
{
    bDeveloperMode = false;
}

DLHangRailHardwareStatus::~DLHangRailHardwareStatus()
{

}

// void DLHangRailHardwareStatus::printRobotStatus()
// {
//     ROS_INFO("bSoftStop:%s, SensorBox:%s, WalkBox:%s, bBlock[0]:%s, bBlock[1]:%s, bBlock[2]:%s, bBlock[3]:%s, HcNetCamera:%s, Magnity:%s,\
//         sick:%d, horizon_motor:%d, vertical_motor:%d, lift_sensor:%d, body_rotate:%d, cam_rotate:%d, pd_rotate:%d, pd_stretch:%d",
//         m_robotSensorStatus.bSoftStop ? "true" : "false", m_robotSensorStatus.bSensorBoxConnected ? "true" : "false", m_robotSensorStatus.bWalkBoxConnected ? "true" : "false",
//         m_robotSensorStatus.bBlock[0] ? "true" : "false", m_robotSensorStatus.bBlock[1] ? "true" : "false", m_robotSensorStatus.bBlock[2] ? "true" : "false", m_robotSensorStatus.bBlock[3] ? "true" : "false",
//         m_robotSensorStatus.bHcNetCameraConnected ? "true" : "false", m_robotSensorStatus.bMagnityCameraConnected ? "true" : "false",
//         m_robotSensorStatus.hardwareStatus[ROBOT_HARDWARE_SICK],
//         m_robotSensorStatus.hardwareStatus[ROBOT_HARDWARE_HORIZON_MOTOR],
//         m_robotSensorStatus.hardwareStatus[ROBOT_HARDWARE_VERTICAL_MOTOR],
//         m_robotSensorStatus.hardwareStatus[ROBOT_HARDWARE_LIFT_SENSOR],
//         m_robotSensorStatus.hardwareStatus[ROBOT_HARDWARE_BODY_ROTATE_MOTOR],
//         m_robotSensorStatus.hardwareStatus[ROBOT_HARDWARE_CAM_ROTATE_MOTOR],
//         m_robotSensorStatus.hardwareStatus[ROBOT_HARDWARE_PD_ROTATE_MOTOR],
//         m_robotSensorStatus.hardwareStatus[ROBOT_HARDWARE_PD_STRETCH_MOTOR]);
// }

void DLHangRailHardwareStatus::robot_ctrl_to_point_req(int pointId)
{
    if (!HANG_ROBOT_FSM.bRobotStandAlone())
    {
        robot_ctrl_emergency_stop();
        ROS_ERROR("robot not standby");
        return;
    }

//  ROBOT_SOCKET_PRIVATE.robot_ctrl_to_point_req(pointId);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_to_point_req(pointId);
}

void DLHangRailHardwareStatus::robot_ctrl_move_abs_req(int offset, int speed /*= 1*/)
{
    if (!HANG_ROBOT_FSM.bRobotStandAlone())
    {
        robot_ctrl_emergency_stop();
        ROS_ERROR("robot not standby");
        return;
    }

//    ROBOT_SOCKET_PRIVATE.robot_ctrl_move_abs_req(offset, speed);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_move_abs_req(offset, speed);
}

void DLHangRailHardwareStatus::robot_ctrl_move_req(RobotMoveMode type, int speed)
{
    if (!bDeveloperMode)
    {
        if (!HANG_ROBOT_FSM.bRobotStandAlone())
        {
            robot_ctrl_emergency_stop();
            ROS_ERROR("robot not standby");
            return;
        }
    }

//    ROBOT_SOCKET_PRIVATE.robot_ctrl_move_req(type, speed);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_move_req(type, speed);
}

void DLHangRailHardwareStatus::robot_ctrl_lift_abs_req(int length)
{
    if (!HANG_ROBOT_FSM.bRobotStandAlone())
    {
        robot_ctrl_emergency_stop();
        ROS_ERROR("robot not standby");
        return;
    }

//    ROBOT_SOCKET_PRIVATE.robot_ctrl_lift_abs_req(length);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_lift_abs_req(length);
}

void DLHangRailHardwareStatus::robot_ctrl_lift_req(RobotLiftMode type)
{
    if (!bDeveloperMode)
    {
        if (!HANG_ROBOT_FSM.bRobotStandAlone())
        {
            robot_ctrl_emergency_stop();
            ROS_ERROR("robot not standby");
            return;
        }
    }
//    ROBOT_SOCKET_PRIVATE.robot_ctrl_lift_req(type);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_lift_req(type);
}

void DLHangRailHardwareStatus::robot_ctrl_cam_ptz_abs_req(int rotate)
{
    if (!HANG_ROBOT_FSM.bRobotStandAlone())
    {
        robot_ctrl_emergency_stop();
        ROS_ERROR("robot not standby");
        return;
    }
//    ROBOT_SOCKET_PRIVATE.robot_ctrl_cam_ptz_abs_req(rotate);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_cam_ptz_abs_req(rotate);
}

void DLHangRailHardwareStatus::robot_ctrl_cam_ptz_req(RobotCamPtzMode type)
{
    if (!bDeveloperMode)
    {
        if (!HANG_ROBOT_FSM.bRobotStandAlone())
        {
            robot_ctrl_emergency_stop();
            ROS_ERROR("robot not standby");
            return;
        }
    }
//    ROBOT_SOCKET_PRIVATE.robot_ctrl_cam_ptz_req(type);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_cam_ptz_req(type);
}

void DLHangRailHardwareStatus::robot_ctrl_body_abs_req(int rotate)
{
    if (!HANG_ROBOT_FSM.bRobotStandAlone())
    {
        robot_ctrl_emergency_stop();
        ROS_ERROR("robot not standby");
        return;
    }

//    ROBOT_SOCKET_PRIVATE.robot_ctrl_body_abs_req(rotate);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_body_abs_req(rotate);
}

void DLHangRailHardwareStatus::robot_ctrl_body_req(RobotBodyRotateMode type)
{
    if (!HANG_ROBOT_FSM.bRobotStandAlone())
    {
        robot_ctrl_emergency_stop();
        ROS_ERROR("robot not standby");
        return;
    }
//    ROBOT_SOCKET_PRIVATE.robot_ctrl_body_req(type);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_body_req(type);
}

void DLHangRailHardwareStatus::robot_ctrl_partialdischarge_req(RobotPartialDischargeOper type)
{
    if (!bDeveloperMode)
    {
        if (!HANG_ROBOT_FSM.bRobotStandAlone())
        {
            robot_ctrl_emergency_stop();
            ROS_ERROR("robot not standby");
            return;
        }
    }
//    ROBOT_SOCKET_PRIVATE.robot_ctrl_partialdischarge_req(type);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_partialdischarge_req(type);
}

void DLHangRailHardwareStatus::robot_ctrl_man_pd_req(RobotBodyPDMode type)
{
    if (!HANG_ROBOT_FSM.bRobotStandAlone())
    {
        robot_ctrl_emergency_stop();
        ROS_ERROR("robot not standby");
        return;
    }
//    ROBOT_SOCKET_PRIVATE.robot_ctrl_man_pd_req(type);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_man_pd_req(type);
}

void DLHangRailHardwareStatus::robot_ctrl_pd_ptz_req(RobotBodyPDArm type)
{
    if (!bDeveloperMode)
    {
        if (!HANG_ROBOT_FSM.bRobotStandAlone())
        {
            robot_ctrl_emergency_stop();
            ROS_ERROR("robot not standby");
            return;
        }
    }
//    ROBOT_SOCKET_PRIVATE.robot_ctrl_pd_ptz_req(type);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_pd_ptz_req(type);
}

void DLHangRailHardwareStatus::robot_ctrl_light_req(RobotBodyWarnLight type)
{
//    ROBOT_SOCKET_PRIVATE.robot_ctrl_light_req(type);
//	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_status_light_req(type);//”–Œ Ã‚
}

void DLHangRailHardwareStatus::robot_ctrl_emergency_stop()
{
//	ROBOT_SOCKET_PRIVATE.robot_ctrl_emergency_stop();
	//HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_emergency_stop_req();
}

void DLHangRailHardwareStatus::robot_ctrl_warning_light_flash(RobotBodyWarnLight type)
{
//	ROBOT_SOCKET_PRIVATE.robot_ctrl_warning_light_flash(type);
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_warning_light_flash_req(type);
}

void DLHangRailHardwareStatus::robot_ctrl_zero_lift()
{
//	ROBOT_SOCKET_PRIVATE.robot_ctrl_zero_lift();
	HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_zero_lift_req();
}

void DLHangRailHardwareStatus::robot_ctrl_touch_screen_req(int type)
{
    HangRail_BACK_TO_CORE_SOCKET.robot_ctrl_touch_screen_req(type);
}

void DLHangRailHardwareStatus::setDevelopMode(bool bStatus)
{
    bDeveloperMode = bStatus;
}

bool DLHangRailHardwareStatus::getDevelopMode()
{
    return bDeveloperMode;
}

// void DLHangRailHardwareStatus::loopStatusFunc()
// {
//     bool bHardwareOK = false;
//     while (bLoopStatusRunning)
//     {
//         Sleep(200);
//         m_robotSensorStatus.bSensorBoxConnected = bSensorBoxConnected();
//         m_robotSensorStatus.bWalkBoxConnected = bWalkBoxConnected();
//         robotQueryStatus tmpStatus = getCurrentStatus();
//         m_robotSensorStatus.bBlock[0] = tmpStatus.bBlock[0];
//         m_robotSensorStatus.bBlock[1] = tmpStatus.bBlock[1];
//         m_robotSensorStatus.bBlock[2] = tmpStatus.bBlock[2];
//         m_robotSensorStatus.bBlock[3] = tmpStatus.bBlock[3];
// 
//         if (m_robotSensorStatus.bSensorBoxConnected && m_robotSensorStatus.bWalkBoxConnected)
//         {
//             m_currentRobotStatus.robotStatusFSM(ROBOT_ACTION_CONNECTED);
//         }
//         else
//         {
//             m_currentRobotStatus.robotStatusFSM(ROBOT_ACTION_DISCONNECTED);
//         }
// 
//         if (!m_robotSensorStatus.bWalkBoxConnected)
//         {
//             for (int i = ROBOT_HARDWARE_SICK; i <= ROBOT_HARDWARE_LIFT_SENSOR; i++)
//             {
//                 ROBOTSTATUS.setHardwareStatus((RobotHardwareType)i, -1);
//             }
//         }
// 
//         if (!m_robotSensorStatus.bSensorBoxConnected)
//         {
//             for (int i = ROBOT_HARDWARE_BODY_ROTATE_MOTOR; i <= ROBOT_HARDWARE_PD_STRETCH_MOTOR; i++)
//             {
//                 ROBOTSTATUS.setHardwareStatus((RobotHardwareType)i, -1);
//             }
//         }
// 
//         m_robotSensorStatus.hardwareStatus = ROBOTSTATUS.getHardwareStatus();
// 
//         bool bHardware = true;
//         for (int i = (int)ROBOT_HARDWARE_SICK; i < ROBOT_HARDWARE_SUM; i++)
//         {
//             if (m_robotSensorStatus.hardwareStatus[(RobotHardwareType)i] != 0 )
//             {
//                 bHardware = false;
//                 break;
//             }
//         }
// 
//         if (bHardware && !m_robotSensorStatus.bSoftStop && !m_robotSensorStatus.bBlock[0] && !m_robotSensorStatus.bBlock[1] && !m_robotSensorStatus.bBlock[2] && !m_robotSensorStatus.bBlock[3])
//         {
//             m_currentRobotStatus.robotStatusFSM(ROBOT_ACTION_HARDWARE_NORMAL);
//         }
//         else
//         {
//             m_currentRobotStatus.robotStatusFSM(ROBOT_ACTION_HARDWARE_ERROR);
//         }
// 
//         printRobotStatus();
//     }
// }