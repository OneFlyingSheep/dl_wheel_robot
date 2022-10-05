#ifndef BACK_CTRL_MSG_H
#define BACK_CTRL_MSG_H
#include "LibSocket/baseMsgOperation.h"

class backGroundCtrlMsg :
    public baseMsgOperation
{
public:
    backGroundCtrlMsg();
    virtual ~backGroundCtrlMsg();

public:
    void registerHandles();
    void recon(boost::asio::ip::tcp::endpoint &end);

    //req: 7000(0x1B58)        res: 17000(0x4268)         robot_ctrl_to_point_req
    void robot_ctrl_to_point_req(int pointId);

    //req: 7001(0x1B59)        res: 17001(0x4269)         robot_ctrl_move_abs_req
    void robot_ctrl_move_abs_req(int offset, int speed = 1);

    //req: 7002(0x1B5a)        res: 17002(0x426a)         robot_ctrl_move_req
    void robot_ctrl_move_req(RobotMoveMode type, int speed);

    //req: 7003(0x1B5b)        res: 17003(0x426b)         robot_ctrl_lift_abs_req
    void robot_ctrl_lift_abs_req(int length);

    //req: 7004(0x1B5c)        res: 17004(0x426c)         robot_ctrl_lift_req
    void robot_ctrl_lift_req(RobotLiftMode type);

    //req: 7005(0x1B5d)        res: 17005(0x426d)         robot_ctrl_cam_ptz_abs_req
    void robot_ctrl_cam_ptz_abs_req(int rotate);

    //req: 7006(0x1B5e)        res: 17006(0x426e)         robot_ctrl_cam_ptz_req
    void robot_ctrl_cam_ptz_req(RobotCamPtzMode type);

    //req: 7007(0x1B5f)        res: 17007(0x426f)         robot_ctrl_body_abs_req
    void robot_ctrl_body_abs_req(int rotate);

    //req: 7008(0x1B60)        res: 17008(0x4270)         robot_ctrl_body_req
    void robot_ctrl_body_req(RobotBodyRotateMode type);

    //req: 7009(0x1B61)        res: 17009(0x4271)         robot_ctrl_partialdischarge_req
    void robot_ctrl_partialdischarge_req(RobotPartialDischargeOper type);

    //req: 7010(0x1B62)        res: 17010(0x4272)         robot_ctrl_man_pd_req
    void robot_ctrl_man_pd_req(RobotBodyPDMode type);

    //req: 7011(0x1B63)        res: 17011(0x4273)         robot_ctrl_man_to_point_req
    void robot_ctrl_man_to_point_req(int pointId, int speed);

    //req: 7012(0x1B64)        res: 17012(0x4274)         robot_ctrl_pd_ptz_req
    void robot_ctrl_pd_ptz_req(RobotBodyPDArm type);

    //req: 7013(0x1B65)        res: 17013(0x4275)         robot_ctrl_pd_collect_req
    void robot_ctrl_pd_collect_req();

    //req: 7014(0x1B66)        res: 17014(0x4276)         robot_ctrl_status_light_req
    void robot_ctrl_status_light_req(RobotBodyWarnLight type);

    //req: 7015(0x1B67)        res: 17015(0x4277)         robot_ctrl_zero_lift
    void robot_ctrl_zero_lift();

    //req: 7016(0x1B68)        res: 17016(0x4278)         robot_ctrl_emergency_stop
    void robot_ctrl_emergency_stop();

    //req: 7017(0x1B69)        res: 17017(0x4279)         robot_ctrl_warning_light_flash
    void robot_ctrl_warning_light_flash(RobotBodyWarnLight type);

    //req: 7018(0x1B6A)        res: 17018(0x427A)         robot_ctrl_touch_screen
    // type = 0 close;
    // type = 1 open;
    void robot_ctrl_touch_screen_req(int type);

private:
    uint16_t getMsgId();

    //resp: resp all
    void robot_ctrl_All_Resp(boost::shared_ptr<roboKitMsg> msg);

    //resp: req all
    void robot_ctrl_All_Req(boost::shared_ptr<roboKitMsg> msg);

    // heardbeat 
    bool bHeartBeatLoopRunning;
    boost::thread *m_heartBeatThread;
    void sendHeartBeatMsg();

private:
    boost::mutex getMsgIdMutex;


};


#endif
