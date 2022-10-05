#ifndef __DL_HANG_ROBOT_DATA_H__
#define __DL_HANG_ROBOT_DATA_H__

#include "common/DLHangRailRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>
#include <boost/signals2.hpp>
#include <boost/bind.hpp>

class DLHangRailRobotStatusData : public Singleton<DLHangRailRobotStatusData>
{
public:
    DLHangRailRobotStatusData();
    ~DLHangRailRobotStatusData();

    boost::signals2::signal<void()> hangRobotSoniseDetected; // »śĘ÷ČĖ³¬Éł²Ø×´Ģ¬

public:
    void setRobotStatusOnNode(int nodeId);

    void setRobotStatusOffset(int offset);

    void setRobotStatusLift(int lift);

    void setRobotStatusCamPtz(int rotate);

    void setRobotStatusBodyRotate(int rotate);

    void setRobotStatusPDDB(int db);

    void setRobotStatusBeBlocked(int bBlock[4]);

    void setRobotStatusSpeed(int speed);

    void setRobotStatusStopBtn(bool stopBtn);

    void setRobotStatusPDArm(RobotBodyPDArmStatus arm, int pdArmRotate);

    void setHardwareStatus(RobotHardwareType type, int status);

    void setSensorBoxConnectStatus(bool bConnected);

    void setWalkBoxConnectStatus(bool bConnected);

    void setHCNetCameraConnectionStatus(bool bConnected);

    void setMagnityCameraConnectionStatus(bool bConnected);

    robotQueryStatus getCurrentStatus();

    void setCurrentStatus(robotQueryStatus status);

    std::map<RobotHardwareType, int> getHardwareStatus();

	void setRobotStatusPtzZoomValue(int zoomValue);

    void setRobotStatusFocusValue(int focus);

    bool bSensorBoxConnected();

    bool bWalkBoxConnected();
private:
    robotQueryStatus m_robotStatus;
    std::map<RobotHardwareType, int> m_robotHardwareStatus;
    boost::mutex m_lock;

    robotSensorStruct m_SensorStatus;
};

#define ROBOTSTATUS DLHangRailRobotStatusData::GetSingleton()

#define ROBOTSTATUS_SET_ONNODE(nodeId) ROBOTSTATUS.setRobotStatusOnNode(nodeId)

#define ROBOTSTATUS_SET_OFFSET(offset) ROBOTSTATUS.setRobotStatusOffset(offset)

#define ROBOTSTATUS_SET_LIFT(lift) ROBOTSTATUS.setRobotStatusLift(lift)

#define ROBOTSTATUS_SET_CAM_PTZ(rotate) ROBOTSTATUS.setRobotStatusCamPtz(rotate)

#define ROBOTSTATUS_SET_BODY_ROTATE(rotate) ROBOTSTATUS.setRobotStatusBodyRotate(rotate)

#define ROBOTSTATUS_SET_PD_DB(dbm) ROBOTSTATUS.setRobotStatusPDDB(dbm)

#define ROBOTSTATUS_GET_STATUS ROBOTSTATUS.getCurrentStatus()
#define ROBOTSTATUS_SET_STATUS(status) ROBOTSTATUS.setCurrentStatus(status)

#endif


