#include "DLHangRailRobotStatusData.h"
//#include "common/DLhangRailRobotFSM.hpp"

DLHangRailRobotStatusData::DLHangRailRobotStatusData()
{
    for (int i = 1; i < ROBOT_HARDWARE_SUM; i++)
    {
        m_robotHardwareStatus[(RobotHardwareType)i] = -1;
    }
}

DLHangRailRobotStatusData::~DLHangRailRobotStatusData()
{
}

void DLHangRailRobotStatusData::setRobotStatusOnNode(int nodeId)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.OnNode = nodeId;
}

void DLHangRailRobotStatusData::setRobotStatusOffset(int offset)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.robotOffset = offset;
}

void DLHangRailRobotStatusData::setRobotStatusLift(int lift)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.liftoffset = lift;
}

void DLHangRailRobotStatusData::setRobotStatusCamPtz(int rotate)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.ptzRotate = rotate;
}

void DLHangRailRobotStatusData::setRobotStatusBodyRotate(int rotate)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.robotRotate = rotate;
}

void DLHangRailRobotStatusData::setRobotStatusPDDB(int db)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.dbNum = db;
}

void DLHangRailRobotStatusData::setRobotStatusBeBlocked(int bBlock[4])
{
//     boost::mutex::scoped_lock lock(m_lock);
//     //memcpy(m_robotStatus.bBlock, bBlock, 4);
//     m_robotStatus.bBlock[0] = bBlock[0];
//     m_robotStatus.bBlock[1] = bBlock[1];
//     m_robotStatus.bBlock[2] = bBlock[2];
//     m_robotStatus.bBlock[3] = bBlock[3];

//     if ((bBlock[1] == 1 || bBlock[2] == 1) ||
//         ((bBlock[0] == 1 || bBlock[3] == 1) && ((m_robotStatus.robotRotate >= 1230 && m_robotStatus.robotRotate <= 1530) || (m_robotStatus.robotRotate >= 3030 && m_robotStatus.robotRotate <= 3330))))
//     {
//         hangRobotSoniseDetected();
//     }
}

void DLHangRailRobotStatusData::setRobotStatusSpeed(int speed)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.speed = speed;
}

void DLHangRailRobotStatusData::setRobotStatusStopBtn(bool stopBtn)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.stopBtn = stopBtn;
}

void DLHangRailRobotStatusData::setRobotStatusPDArm(RobotBodyPDArmStatus arm, int pdArmRotate)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.armStatus = arm;
    m_robotStatus.pdArmRotate = pdArmRotate;
}

void DLHangRailRobotStatusData::setHardwareStatus(RobotHardwareType type, int status)
{
    boost::mutex::scoped_lock lock(m_lock);
    if (m_robotHardwareStatus.find(type) != m_robotHardwareStatus.end())
    {
        m_robotHardwareStatus[type] = status;
    }
    else
    {
        ROS_ERROR("setHardwareStatus error, type:%d, status:%d", (int)type, status);
    }
}

void DLHangRailRobotStatusData::setSensorBoxConnectStatus(bool bConnected)
{
    m_SensorStatus.bSensorBoxConnected = bConnected;
}

void DLHangRailRobotStatusData::setWalkBoxConnectStatus(bool bConnected)
{
    m_SensorStatus.bWalkBoxConnected = bConnected;
}

void DLHangRailRobotStatusData::setHCNetCameraConnectionStatus(bool bConnected)
{
    m_SensorStatus.bHcNetCameraConnected = bConnected;
}

void DLHangRailRobotStatusData::setMagnityCameraConnectionStatus(bool bConnected)
{
    m_SensorStatus.bMagnityCameraConnected = bConnected;
}

robotQueryStatus DLHangRailRobotStatusData::getCurrentStatus()
{
    return m_robotStatus;
}

void DLHangRailRobotStatusData::setCurrentStatus(robotQueryStatus status)
{
    m_robotStatus = status;
}

std::map<RobotHardwareType, int> DLHangRailRobotStatusData::getHardwareStatus()
{
    boost::mutex::scoped_lock lock(m_lock);
    return m_robotHardwareStatus;
}

void DLHangRailRobotStatusData::setRobotStatusPtzZoomValue(int zoomValue)
{
    boost::mutex::scoped_lock lock(m_lock);
	m_robotStatus.visibleZoom = zoomValue;
}

void DLHangRailRobotStatusData::setRobotStatusFocusValue(int focus)
{
    boost::mutex::scoped_lock lock(m_lock);
    m_robotStatus.visibleFocus = focus;
}

bool DLHangRailRobotStatusData::bSensorBoxConnected()
{
    return m_SensorStatus.bSensorBoxConnected;
}

bool DLHangRailRobotStatusData::bWalkBoxConnected()
{
    return m_SensorStatus.bWalkBoxConnected;
}