#pragma once
#include "common/DLHangRailRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include "LibDLHangRailConfigData/DLHangRailRobotStatusData.h"
//#include "LibDLHangRailSocket/DLHangRailRobotSocket.h"
#include "LibDLHangRailRobotCoreSocket/LibDLHangRailRobotCoreSocket.h"
#include <boost/thread.hpp>

class DLHangRailHardwareStatus : public Singleton<DLHangRailHardwareStatus>
{
public:
    DLHangRailHardwareStatus();
    ~DLHangRailHardwareStatus();

public:
    void robot_ctrl_to_point_req(int pointId);

    void robot_ctrl_move_abs_req(int offset, int speed = 1);

    void robot_ctrl_move_req(RobotMoveMode type, int speed);

    void robot_ctrl_lift_abs_req(int length);

    void robot_ctrl_lift_req(RobotLiftMode type);

    void robot_ctrl_cam_ptz_abs_req(int rotate);

    void robot_ctrl_cam_ptz_req(RobotCamPtzMode type);

    void robot_ctrl_body_abs_req(int rotate);

    void robot_ctrl_body_req(RobotBodyRotateMode type);

    void robot_ctrl_partialdischarge_req(RobotPartialDischargeOper type);

    void robot_ctrl_man_pd_req(RobotBodyPDMode type);

    void robot_ctrl_pd_ptz_req(RobotBodyPDArm type);

    void robot_ctrl_light_req(RobotBodyWarnLight type);

    void robot_ctrl_emergency_stop();

    void robot_ctrl_warning_light_flash(RobotBodyWarnLight type);

    void robot_ctrl_zero_lift();

    void robot_ctrl_touch_screen_req(int type);

    void setDevelopMode(bool bStatus);

    bool getDevelopMode();

private:
    bool bDeveloperMode;
};

#define ROBOT_CTRL DLHangRailHardwareStatus::GetSingleton()




