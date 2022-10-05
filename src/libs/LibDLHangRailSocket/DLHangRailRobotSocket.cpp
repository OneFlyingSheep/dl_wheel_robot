#include "DLHangRailRobotSocket.h"
#include "LibDLHangRailConfigData/DLHangRailStationCfgData.h"
#include "LibDLHangRailConfigData/DLHangRailRobotStatusData.h"
#include "LibDLHangRailConfigData/DLHangRailCoreConfigStruct.h"

DLHangRailRobotSocket::DLHangRailRobotSocket()
{
    m_BackGroundMovingBoxCtrlMsg = new backGroundCtrlMsg();
    m_BackGroundMovingBoxQueryMsg = new backGroundQueryMsg();
    m_backGroundSensorBoxCtrlMsg = new backGroundCtrlMsg();
    m_BackGroundSensorBoxQueryMsg = new backGroundQueryMsg();

//	QString walkBox = ROBOTSTATIONCFG.getStationCfg().robotWalkBoxIpAddr;
	QString walkBox = HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotWalkBoxIpAddr;
//	QString sensorBox = ROBOTSTATIONCFG.getStationCfg().robotSensorIpAddr;
	QString sensorBox = HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotSensorIpAddr;

    m_ipReg = new QRegExp("((25[0-5]|2[0-4]\\d|((1\\d{2})|([1-9]?\\d)))\\.){3}(25[0-5]|2[0-4]\\d|((1\\d{2})|([1-9]?\\d)))");

    if (m_ipReg->exactMatch(walkBox))
    {
	//	m_BackGroundMovingBoxCtrlMsg->runMessageLoop(ROBOTSTATIONCFG.getStationCfg().robotWalkBoxIpAddr.toStdString(), ROBOTSTATIONCFG.getStationCfg().robotWalkBoxCtrlPort, SOCK_TYPE_CLIENT);
		m_BackGroundMovingBoxCtrlMsg->runMessageLoop(HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotWalkBoxIpAddr.toStdString(), HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotWalkBoxCtrlPort, SOCK_TYPE_CLIENT);
	//	m_BackGroundMovingBoxQueryMsg->runMessageLoop(ROBOTSTATIONCFG.getStationCfg().robotWalkBoxIpAddr.toStdString(), ROBOTSTATIONCFG.getStationCfg().robotWalkBoxQueryPort, SOCK_TYPE_CLIENT);
		m_BackGroundMovingBoxQueryMsg->runMessageLoop(HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotWalkBoxIpAddr.toStdString(), HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotWalkBoxQueryPort, SOCK_TYPE_CLIENT);
    }
    else
    {
        ROS_ERROR("walkBox ip error!!");
    }

    if (m_ipReg->exactMatch(sensorBox))
    {
        m_backGroundSensorBoxCtrlMsg->runMessageLoop(HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotSensorIpAddr.toStdString(), HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotSensorBoxCtrlPort, SOCK_TYPE_CLIENT);
        m_BackGroundSensorBoxQueryMsg->runMessageLoop(HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotSensorIpAddr.toStdString(), HANGRAIL_ROBOT_CORE_CONFIG.getCoreCfg().robotSensorBoxQueryPort, SOCK_TYPE_CLIENT);
    }
    else
    {
        ROS_ERROR("sensorBoxs ip error!!");
    }

    bLoopStatus = true;
    m_currentConnectStatusThread = new boost::thread(boost::bind(&DLHangRailRobotSocket::loopStatus, this));
}

DLHangRailRobotSocket::~DLHangRailRobotSocket()
{
    stopLoop();

    if (m_BackGroundMovingBoxCtrlMsg != NULL)
    {
        delete m_BackGroundMovingBoxCtrlMsg;
    }

    if (m_BackGroundMovingBoxQueryMsg != NULL)
    {
        delete m_BackGroundMovingBoxQueryMsg;
    }

    if (m_backGroundSensorBoxCtrlMsg != NULL)
    {
        delete m_backGroundSensorBoxCtrlMsg;
    }

    if (m_BackGroundSensorBoxQueryMsg != NULL)
    {
        delete m_BackGroundSensorBoxQueryMsg;
    }

    if (m_currentConnectStatusThread != NULL)
    {
        bLoopStatus = false;
        m_currentConnectStatusThread->join();
        delete m_currentConnectStatusThread;
    }
}

void DLHangRailRobotSocket::initSignal(boost::signals2::signal<void(QString, QString)>::slot_type motorSlot, boost::signals2::signal<void(unsigned char)>::slot_type sickSlot)
{
    m_BackGroundMovingBoxQueryMsg->signal_motorDebugInfo.connect(motorSlot);
    m_BackGroundMovingBoxQueryMsg->signal_sickDebugInfo.connect(sickSlot);
}

void DLHangRailRobotSocket::disconnectSignal()
{
    m_BackGroundMovingBoxQueryMsg->signal_motorDebugInfo.disconnect_all_slots();
    m_BackGroundMovingBoxQueryMsg->signal_sickDebugInfo.disconnect_all_slots();
}

void DLHangRailRobotSocket::robot_ctrl_to_point_req(int pointId)
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_to_point_req(pointId);
}

void DLHangRailRobotSocket::robot_ctrl_move_abs_req(int offset, int speed /* = 1*/)
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_move_abs_req(offset, speed);
}

void DLHangRailRobotSocket::robot_ctrl_move_req(RobotMoveMode type, int speed /* = 1*/)
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_move_req(type, speed);
}

void DLHangRailRobotSocket::robot_ctrl_lift_abs_req(int length)
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_lift_abs_req(length);
}

void DLHangRailRobotSocket::robot_ctrl_lift_req(RobotLiftMode type)
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_lift_req(type);
}

void DLHangRailRobotSocket::robot_ctrl_cam_ptz_abs_req(int rotate)
{
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_cam_ptz_abs_req(rotate);
}

void DLHangRailRobotSocket::robot_ctrl_cam_ptz_req(RobotCamPtzMode type)
{
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_cam_ptz_req(type);
}

void DLHangRailRobotSocket::robot_ctrl_body_abs_req(int rotate)
{
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_body_abs_req(rotate);
}

void DLHangRailRobotSocket::robot_ctrl_body_req(RobotBodyRotateMode type)
{
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_body_req(type);
}

void DLHangRailRobotSocket::robot_ctrl_partialdischarge_req(RobotPartialDischargeOper type)
{
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_partialdischarge_req(type);
}

void DLHangRailRobotSocket::robot_ctrl_man_pd_req(RobotBodyPDMode type)
{
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_man_pd_req(type);
}

void DLHangRailRobotSocket::robot_ctrl_pd_ptz_req(RobotBodyPDArm type)
{
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_pd_ptz_req(type);
}

void DLHangRailRobotSocket::robot_ctrl_light_req(RobotBodyWarnLight type)
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_status_light_req(type);
}

void DLHangRailRobotSocket::robot_ctrl_warning_light_flash(RobotBodyWarnLight type)
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_warning_light_flash(type);
}

void DLHangRailRobotSocket::robot_ctrl_zero_lift()
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_zero_lift();
}

void DLHangRailRobotSocket::robot_ctrl_emergency_stop()
{
    m_BackGroundMovingBoxCtrlMsg->robot_ctrl_emergency_stop();
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_emergency_stop();
}

void DLHangRailRobotSocket::robot_ctrl_touch_screen_req(int type)
{
    m_backGroundSensorBoxCtrlMsg->robot_ctrl_touch_screen_req(type);
}

bool DLHangRailRobotSocket::bSensorBoxConnected()
{
    try
    {
        if (m_backGroundSensorBoxCtrlMsg == NULL || m_BackGroundSensorBoxQueryMsg == NULL)
        {
            return false;
        }
        else
        {
            return m_backGroundSensorBoxCtrlMsg->isConnected() && m_BackGroundSensorBoxQueryMsg->isConnected();
        }
    }
    catch (...)
    {
        ROS_ERROR("bSensorBoxConnected exception error");
        return false;
    }
    
}

bool DLHangRailRobotSocket::bWalkBoxConnected()
{
    try
    {
        if (m_BackGroundMovingBoxCtrlMsg == NULL || m_BackGroundMovingBoxQueryMsg == NULL)
        {
            return false;
        }
        else
        {
            return m_BackGroundMovingBoxCtrlMsg->isConnected() && m_BackGroundMovingBoxQueryMsg->isConnected();
        }
    }
    catch (...)
    {
        ROS_ERROR("bSensorBoxConnected exception error");
        return false;
    }
    
}

void DLHangRailRobotSocket::stopLoop()
{
    if (NULL != m_currentConnectStatusThread)
    {
        bLoopStatus = false;
        m_currentConnectStatusThread->join();
        delete m_currentConnectStatusThread;
        m_currentConnectStatusThread = NULL;
    }
}

void DLHangRailRobotSocket::loopStatus()
{
    while (bLoopStatus)
    {
        Sleep(200);
        ROBOTSTATUS.setSensorBoxConnectStatus(bSensorBoxConnected());
        ROBOTSTATUS.setWalkBoxConnectStatus(bWalkBoxConnected());
    }
}