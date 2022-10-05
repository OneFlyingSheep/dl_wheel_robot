#include "backGroundCtrlMsg.h"
static unsigned int backGroundMsgCtrlID = 0;

backGroundCtrlMsg::backGroundCtrlMsg()
{
    //startCheckDataTimer();

    bHeartBeatLoopRunning = true;
    m_heartBeatThread = new boost::thread(boost::bind(&backGroundCtrlMsg::sendHeartBeatMsg, this));
}

backGroundCtrlMsg::~backGroundCtrlMsg()
{
    ROS_WARN("backGroundCtrlMsg::backGroundCtrlMsg");
    if (NULL != m_heartBeatThread)
    {
        bHeartBeatLoopRunning = false;
        m_heartBeatThread->join();
        delete m_heartBeatThread;
    }
}

void backGroundCtrlMsg::registerHandles()
{
    ROS_WARN("backGroundCtrlMsg::registerHandles");
    for (int i = 0; i <= 19; i++)
    {
        //        ROS_INFO("registerHandles robot_ctrl_All_Resp:%d, 2:%d", i, (int)(i + Request_robot_ctrl_to_point_req + 10000));
        m_baseSocket->registerMsgHandle((i + Request_robot_ctrl_to_point_req + 10000),
                                    boost::bind(&backGroundCtrlMsg::robot_ctrl_All_Resp, this, _1));
    }

//    for (int i = 0; i <= 11; i++)
//    {
//        //        ROS_INFO("registerHandles robot_ctrl_All_Req:%d, 2:%d", i, (int)(i + Request_robot_ctrl_to_point_req));
//        m_socket->registerMsgHandle((i + Request_robot_ctrl_to_point_req),
//                                    boost::bind(&backGroundCtrlMsg::robot_ctrl_All_Req, this, _1));
//    }
}

void backGroundCtrlMsg::recon(boost::asio::ip::tcp::endpoint &end)
{
    m_baseSocket->reconnect(end);
}

//req: 7000(0x1B58)        res: 17000(0x4268)         robot_ctrl_to_point_req
void backGroundCtrlMsg::robot_ctrl_to_point_req(int pointId)
{
    ROS_INFO("backGroundCtrlMsg::robot_ctrl_to_point_req,id:%d", pointId);
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B58);

    body.jsonAppendElement("pointId", pointId);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7001(0x1B59)        res: 17001(0x4269)         robot_ctrl_move_abs_req
void backGroundCtrlMsg::robot_ctrl_move_abs_req(int offset, int speed /* = 1*/)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B59);

    body.jsonAppendElement("offset", offset);
    body.jsonAppendElement("speed", speed);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7002(0x1B5A)        res: 17002(0x426a)         robot_ctrl_move_req
void backGroundCtrlMsg::robot_ctrl_move_req(RobotMoveMode type, int speed)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B5A);

    body.jsonAppendElement("type", (int)type);
    body.jsonAppendElement("speed", speed);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7003(0x1B5B)        res: 17003(0x426b)         robot_ctrl_lift_abs_req
void backGroundCtrlMsg::robot_ctrl_lift_abs_req(int length)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B5B);

    body.jsonAppendElement("length", length);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7004(0x1B5C)        res: 17004(0x426c)         robot_ctrl_lift_req
void backGroundCtrlMsg::robot_ctrl_lift_req(RobotLiftMode type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B5C);

    body.jsonAppendElement("type", (int)type);    

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7005(0x1B5D)        res: 17005(0x426d)         robot_ctrl_cam_ptz_abs_req
void backGroundCtrlMsg::robot_ctrl_cam_ptz_abs_req(int rotate)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B5D);

    body.jsonAppendElement("rotate", rotate);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7006(0x1B5E)        res: 17006(0x426e)         robot_ctrl_cam_ptz_req
void backGroundCtrlMsg::robot_ctrl_cam_ptz_req(RobotCamPtzMode type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B5E);

    body.jsonAppendElement("type", (int)type);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7007(0x1B5F)        res: 17007(0x426f)         robot_ctrl_body_abs_req
void backGroundCtrlMsg::robot_ctrl_body_abs_req(int rotate)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B5F);

    body.jsonAppendElement("rotate", rotate);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7008(0x1B60)        res: 17008(0x4270)         robot_ctrl_body_req
void backGroundCtrlMsg::robot_ctrl_body_req(RobotBodyRotateMode type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B60);

    body.jsonAppendElement("type", (int)type);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7009(0x1B61)        res: 17009(0x4271)         robot_ctrl_partialdischarge_req
void backGroundCtrlMsg::robot_ctrl_partialdischarge_req(RobotPartialDischargeOper type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B61);

    body.jsonAppendElement("type", (int)type);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7010(0x1B62)        res: 17010(0x4272)         robot_ctrl_man_pd_req
void backGroundCtrlMsg::robot_ctrl_man_pd_req(RobotBodyPDMode type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B62);

    body.jsonAppendElement("type", (int)type);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7011(0x1B63)        res: 17011(0x4273)         robot_ctrl_man_to_point_req
void backGroundCtrlMsg::robot_ctrl_man_to_point_req(int pointId, int speed)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B63);

    body.jsonAppendElement("pointId", pointId);
    body.jsonAppendElement("speed", speed);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7012(0x1B64)        res: 17012(0x4274)         robot_ctrl_pd_ptz_req
void backGroundCtrlMsg::robot_ctrl_pd_ptz_req(RobotBodyPDArm type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B64);

    body.jsonAppendElement("type", (int)type);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7013(0x1B65)        res: 17013(0x4275)         robot_ctrl_pd_collect_req
void backGroundCtrlMsg::robot_ctrl_pd_collect_req()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B65);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7014(0x1B66)        res: 17014(0x4276)         robot_ctrl_status_light_req
void backGroundCtrlMsg::robot_ctrl_status_light_req(RobotBodyWarnLight type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B66);

    body.jsonAppendElement("type", (int)type);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7015(0x1B67)        res: 17015(0x4277)         robot_ctrl_zero_lift
void backGroundCtrlMsg::robot_ctrl_zero_lift()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B67);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7016(0x1B68)        res: 17016(0x4278)         robot_ctrl_emergency_stop
void backGroundCtrlMsg::robot_ctrl_emergency_stop()
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B68);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

//req: 7017(0x1B69)        res: 17017(0x4279)         robot_ctrl_warning_light_flash
void backGroundCtrlMsg::robot_ctrl_warning_light_flash(RobotBodyWarnLight type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B69);

    body.jsonAppendElement("type", (int)type);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}


//req: 7018(0x1B6A)        res: 17018(0x427A)         robot_ctrl_touch_screen_req
void backGroundCtrlMsg::robot_ctrl_touch_screen_req(int type)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(0x1B6A);

    body.jsonAppendElement("type", (int)type);

    baseMsgPostMsg(body);

    addToCheckSendMsgList(body);
}

uint16_t backGroundCtrlMsg::getMsgId()
{
    boost::mutex::scoped_lock lock(getMsgIdMutex);
    return backGroundMsgCtrlID++;
}


void backGroundCtrlMsg::robot_ctrl_All_Resp(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundCtrlMsg::robot_ctrl_All_Resp==msgType:%d  Str:%s", msg->msgHeader.getType(), msg->msgBody.getJsonString().c_str());
    int errCode;
    string errMsg;
    uint16_t type = msg->msgHeader.getType();

    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundCtrlMsg::robot_ctrl_All_Resp retVal == NULL");
    }

    if (val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_ERROR("backGroundCtrlMsg::robot_ctrl_All_Resp : msgType:%d", (int)type);
        updateSendMessageStatus(msg->msgHeader.getNumber(), msg->msgHeader.getType());
    }
    else
    {
        errCode = val["errCode"].asInt();
        errMsg = val["errMsg"].asString();

        ROS_ERROR("backGroundCtrlMsg::robot_ctrl_All_Resp : msgType:%d, errCode:%d, errMsg:%s", (int)type, errCode, errMsg);
        updateSendMessageStatus(msg->msgHeader.getNumber(), msg->msgHeader.getType());
    }
}

void backGroundCtrlMsg::robot_ctrl_All_Req(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundCtrlMsg::robot_ctrl_All_Req==msgType:%d, msgNum=%d, Str:%s", msg->msgHeader.getType(), msg->msgHeader.getNumber(), msg->msgBody.getJsonString().c_str());
    uint16_t type = msg->msgHeader.getType();

    jsonBody body;
    body.setMsgNumber(msg->msgHeader.getNumber());
    body.setMsgType(type + 10000);
    baseMsgPostMsg(body);
    switch(type)
    {
    case Request_robot_ctrl_to_point_req:
    {
        ROS_INFO("do something Request_robot_ctrl_to_point_req");
        break;
    }
    case Request_robot_ctrl_move_abs_req:
    {
        ROS_INFO("do something Request_robot_ctrl_move_abs_req");
        break;
    }
    case Request_robot_ctrl_move_req:
    {
        ROS_INFO("do something Request_robot_ctrl_move_req");
        break;
    }
    case Request_robot_ctrl_lift_abs_req:
    {
        ROS_INFO("do something Request_robot_ctrl_lift_abs_req");
        break;
    }
    case Request_robot_ctrl_lift_req:
    {
        ROS_INFO("do something Request_robot_ctrl_lift_req");
        break;
    }
    case Request_robot_ctrl_cam_ptz_abs_req:
    {
        ROS_INFO("do something Request_robot_ctrl_cam_ptz_abs_req");
        break;
    }
    case Request_robot_ctrl_cam_ptz_req:
    {
        ROS_INFO("do something Request_robot_ctrl_cam_ptz_req");
        break;
    }
    case Request_robot_ctrl_body_abs_req:
    {
        ROS_INFO("do something Request_robot_ctrl_body_abs_req");
        break;
    }
    case Request_robot_ctrl_body_req:
    {
        ROS_INFO("do something Request_robot_ctrl_body_req");
        break;
    }
    case Request_robot_ctrl_partialdischarge_req:
    {
        ROS_INFO("do something Request_robot_ctrl_partialdischarge_req");
        break;
    }
    case Request_robot_ctrl_man_pd_req:
    {
        ROS_INFO("do something Request_robot_ctrl_man_pd_req");
        break;
    }
    case Request_robot_ctrl_man_to_point_req:
    {
        ROS_INFO("do something Request_robot_ctrl_man_to_point_req");
        break;
    }
    default:
    {
        ROS_ERROR("NULL||backGroundCtrlMsg::robot_ctrl_All_Req DEFAULT, need disconnect?");
    }
    }
}

void backGroundCtrlMsg::sendHeartBeatMsg()
{
    while (bHeartBeatLoopRunning)
    {
        Sleep(500);
        if (!isConnected())
        {
            continue;
        }     
        jsonBody body;
        body.setMsgNumber(getMsgId());
        body.setMsgType(0x1F3F);

        baseMsgPostMsg(body);

        addToCheckSendMsgList(body);
    }
}

