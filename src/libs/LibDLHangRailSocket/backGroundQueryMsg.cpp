#include "backGroundQueryMsg.h"
#include "LibDLHangRailConfigData/DLHangRailRobotStatusData.h"
#include <QDateTime>
static unsigned int backGroundMsgQueryID = 0;

backGroundQueryMsg::backGroundQueryMsg()
{
    bStatusLoopRunning = true;
    
    motorStatusString.push_back("");
    motorStatusString.push_back("输入电压过高");
    motorStatusString.push_back("输出电流过高");
    motorStatusString.push_back("输入电压过低");
    motorStatusString.push_back("现场总线故障");
    motorStatusString.push_back("虽然锁闭功能已启用，电机仍向错误方向旋转");
    motorStatusString.push_back("仅CANopen : 在发送节点保护要求时，NMT主机需要的时间过长");
    motorStatusString.push_back("电气故障或硬件损坏造成编码器故障");
    motorStatusString.push_back("编码器故障；自动设置时未找到标识脉冲");
    motorStatusString.push_back("AB通道出错");
    motorStatusString.push_back("超过正向限位开关和公差范围");
    motorStatusString.push_back("超过负向限位开关和公差范围");
    motorStatusString.push_back("设备温度超过80度");
    motorStatusString.push_back("对象6065h(Following Error Window) 和 对 象 6066h(Following Error Time Out) 的值被超过，因此触发错误。该错误必须通过对象3202h中的位7进行激活。");
    motorStatusString.push_back("非易失性存储器已满，需要重启控制器进行清理工作。");
    motorStatusString.push_back("电机堵转");
    motorStatusString.push_back("非易失性存储器损坏，需要重启控制器进行清理工作。");
    motorStatusString.push_back("从机发送PDO消息所需时间过长。");
}

backGroundQueryMsg::~backGroundQueryMsg()
{
    if (m_statusLoopThread)
    {
        bStatusLoopRunning = false;
        m_statusLoopThread->join();
        delete m_statusLoopThread;
    }
}

void backGroundQueryMsg::registerHandles()
{
    ROS_WARN("backGroundQuerylMsg::registerHandles");
    m_baseSocket->registerMsgHandle(Query_robot_status_on_node, boost::bind(&backGroundQueryMsg::query_robot_status_on_node, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_offset, boost::bind(&backGroundQueryMsg::query_robot_status_offset, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_lift, boost::bind(&backGroundQueryMsg::query_robot_status_lift, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_cam_ptz, boost::bind(&backGroundQueryMsg::query_robot_status_cam_ptz, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_body_rotate, boost::bind(&backGroundQueryMsg::query_robot_status_body_rotate, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_pd, boost::bind(&backGroundQueryMsg::query_robot_status_pd, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_snoise, boost::bind(&backGroundQueryMsg::query_robot_status_snoise_status, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_speed, boost::bind(&backGroundQueryMsg::query_robot_status_speed, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_stopBtn, boost::bind(&backGroundQueryMsg::query_robot_status_stop_btn, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_pd_arm, boost::bind(&backGroundQueryMsg::query_robot_status_pd_arm_status, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_status_hardware_status, boost::bind(&backGroundQueryMsg::query_robot_hardware_status, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_motor_status_debug, boost::bind(&backGroundQueryMsg::query_robot_motor_status_debug, this, _1));
    m_baseSocket->registerMsgHandle(Query_robot_sick_status_debug, boost::bind(&backGroundQueryMsg::query_robot_sick_status_debug, this, _1));
    m_statusLoopThread = new boost::thread(boost::bind(&backGroundQueryMsg::getRobotQueryStatus, this));
}

void backGroundQueryMsg::recon(boost::asio::ip::tcp::endpoint &end)
{
    m_baseSocket->reconnect(end);
}

void backGroundQueryMsg::testOnNode(int id)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(8000);

    body.jsonAppendElement("OnNode", id);

    baseMsgPostMsg(body);
}

void backGroundQueryMsg::testOnOffset(float offset)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(8001);

    body.jsonAppendElement("offset", offset);

    baseMsgPostMsg(body);
}

void backGroundQueryMsg::testOnlift(float length)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(8002);

    body.jsonAppendElement("length", length);

    baseMsgPostMsg(body);
}

void backGroundQueryMsg::testCamPtz(float pan, float tilt)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(8003);

    body.jsonAppendElement("pan", pan);
    body.jsonAppendElement("tilt", tilt);

    baseMsgPostMsg(body);
}

void backGroundQueryMsg::testRotate(float Rotate)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(8004);

    body.jsonAppendElement("Rotate", Rotate);

    baseMsgPostMsg(body);
}

void backGroundQueryMsg::testPD(float pd)
{
    jsonBody body;
    body.setMsgNumber(getMsgId());
    body.setMsgType(8005);

    body.jsonAppendElement("pd", pd);

    baseMsgPostMsg(body);
}

void backGroundQueryMsg::getRobotQueryStatus()
{
    while (bStatusLoopRunning)
    {
        Sleep(500);
        if (!isConnected())
        {
            continue;
        }
        jsonBody body;
        body.setMsgNumber(getMsgId());
        body.setMsgType(8999);
        baseMsgPostMsg(body);
    }
}

uint16_t backGroundQueryMsg::getMsgId()
{
    boost::mutex::scoped_lock lock(getMsgIdMutex);
    return backGroundMsgQueryID++;
}

//req: 8000(0x1770)        query_robot_status_on_node
void backGroundQueryMsg::query_robot_status_on_node(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_on_node==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("backGroundQuerylMsg::query_robot_status_on_node retVal = NULL");
		return;
	}

    int OnNodeId = val["OnNode"].asInt();
    ROBOTSTATUS_SET_ONNODE(OnNodeId);

    int errCode;
    string errMsg;

    if ( val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();


    if (0 == errCode)
    {
        ROS_INFO("8000||backGroundQuerylMsg::query_robot_status_on_node, onNode:%d", OnNodeId);
        return;
    }
    else
    {
        ROS_ERROR("8000||backGroundQuerylMsg::query_robot_status_on_node error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8001(0x1771)        query_robot_status_offset
void backGroundQueryMsg::query_robot_status_offset(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_offset==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("backGroundQuerylMsg::query_robot_status_offset retVal = NULL");
		return;
	}

    int offset = val["offset"].asInt();
    ROBOTSTATUS_SET_OFFSET(offset);

    int errCode;
    string errMsg;

    if ( val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8001||backGroundQuerylMsg::query_robot_status_offset, offset:%d", offset);
        return;
    }
    else
    {
        ROS_ERROR("8001||backGroundQuerylMsg::query_robot_status_offset error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8002(0x1772)        query_robot_status_lift
void backGroundQueryMsg::query_robot_status_lift(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_lift==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("backGroundQuerylMsg::query_robot_status_lift retVal = NULL");
		return;
	}

    int length = val["length"].asInt();
    ROBOTSTATUS_SET_LIFT(length);

    int errCode;
    string errMsg;

    if ( val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8002||backGroundQuerylMsg::query_robot_status_lift, length:%d", length);
        return;
    }
    else
    {
        ROS_ERROR("8002||backGroundQuerylMsg::query_robot_status_lift error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8003(0x1773)        query_robot_status_cam_ptz
void backGroundQueryMsg::query_robot_status_cam_ptz(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_cam_ptz==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("backGroundQuerylMsg::query_robot_status_cam_ptz retVal = NULL");
		return;
	}

    int pan = val["rotate"].asInt();
    ROBOTSTATUS_SET_CAM_PTZ(pan);

    int errCode;
    string errMsg;

    if ( val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8003||backGroundQuerylMsg::query_robot_status_cam_ptz, rotate:%d", pan);
        return;
    }
    else
    {
        ROS_ERROR("8003||backGroundQuerylMsg::query_robot_status_cam_ptz error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8004(0x1774)        query_robot_status_body_rotate
void backGroundQueryMsg::query_robot_status_body_rotate(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_body_rotate==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("backGroundQuerylMsg::query_robot_status_body_rotate retVal = NULL");
		return;
	}

    int rotate = val["rotate"].asInt();
    ROBOTSTATUS_SET_BODY_ROTATE(rotate);


    int errCode;
    string errMsg;

    if ( val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8004||backGroundQuerylMsg::query_robot_status_body_rotate, rotate:%d", rotate);
        return;
    }
    else
    {
        ROS_ERROR("8004||backGroundQuerylMsg::query_robot_status_body_rotate error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8005(0x1775)        query_robot_status_pd
void backGroundQueryMsg::query_robot_status_pd(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_pd==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("backGroundQuerylMsg::query_robot_status_pd retVal = NULL");
		return;
	}

    int dbm = val["dbm"].asInt();
    ROBOTSTATUS_SET_PD_DB(dbm);

    int errCode;
    string errMsg;

    if ( val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8005||backGroundQuerylMsg::query_robot_status_pd, dbm:%f", dbm);
        return;
    }
    else
    {
        ROS_ERROR("8005||backGroundQuerylMsg::query_robot_status_pd error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8006 (0x1F66)        query_robot_status_snoise_status
void backGroundQueryMsg::query_robot_status_snoise_status(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_snoise_status==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundQuerylMsg::query_robot_status_snoise_status retVal = NULL");
        return;
    }

    unsigned char snoise = val["snoise"].asUInt();

    int left = snoise & 0x01;
    int right = (snoise >> 1) & 0x01;
    int bottom1 = (snoise >> 2) & 0x01;
    int bottom2 = (snoise >> 3) & 0x01;


    int bBlock[4] = { left, right, bottom1, bottom2 };


    ROBOTSTATUS.setRobotStatusBeBlocked(bBlock);

    int errCode;
    string errMsg;

    if (val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8006||backGroundQuerylMsg::query_robot_status_snoise_status, status:[%d, %d, %d, %d, ]", bBlock[0], bBlock[1], bBlock[2], bBlock[3]);
        return;
    }
    else
    {
        ROS_ERROR("8006||backGroundQuerylMsg::query_robot_status_snoise_status error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8007 (0x1F67)        query_robot_status_speed
void backGroundQueryMsg::query_robot_status_speed(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_speed==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundQuerylMsg::query_robot_status_speed retVal = NULL");
        return;
    }

    int speed = val["speed"].asInt();
    ROBOTSTATUS.setRobotStatusSpeed(speed);

    int errCode;
    string errMsg;

    if (val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8007||backGroundQuerylMsg::query_robot_status_speed, speed:%d", speed);
        return;
    }
    else
    {
        ROS_ERROR("8007||backGroundQuerylMsg::query_robot_status_speed error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8010 (0x1F6A)        query_robot_status_stop_btn
void backGroundQueryMsg::query_robot_status_stop_btn(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_stop_btn==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundQuerylMsg::query_robot_status_stop_btn retVal = NULL");
        return;
    }

    bool stopBtn = val["stopBtn"].asBool();
    ROBOTSTATUS.setRobotStatusStopBtn(stopBtn);

    int errCode;
    string errMsg;

    if (val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8007||backGroundQuerylMsg::query_robot_status_stop_btn, status:%s", stopBtn ? "pressed" : "released");
        return;
    }
    else
    {
        ROS_ERROR("8007||backGroundQuerylMsg::query_robot_status_stop_btn error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8011 (0x1F6B)        query_robot_status_pd_arm_status
void backGroundQueryMsg::query_robot_status_pd_arm_status(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_status_pd_arm_status==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundQuerylMsg::query_robot_status_pd_arm_status retVal = NULL");
        return;
    }

    RobotBodyPDArmStatus status = (RobotBodyPDArmStatus)(val["status"].asInt());
    ROBOTSTATUS.setRobotStatusPDArm(status, 0);

    int errCode;
    string errMsg;

    if (val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8011||backGroundQuerylMsg::query_robot_status_pd_arm_status, status:%d, angle:%d", (int)status, 0);
        return;
    }
    else
    {
        ROS_ERROR("8011||backGroundQuerylMsg::query_robot_status_pd_arm_status error : code:%d, msg:%s", errCode, errMsg);
    }
}

//req: 8012 (0x1F6C)        query_robot_hardware_status
void backGroundQueryMsg::query_robot_hardware_status(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_hardware_status==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundQuerylMsg::query_robot_hardware_status retVal = NULL");
        return;
    }

    unsigned char rStatus = val["status"].asUInt();

    int type = rStatus >> 4;
    int status = (rStatus & 0x0f);

    ROBOTSTATUS.setHardwareStatus((RobotHardwareType)type, status);

    ROS_INFO("query_robot_hardware_status: recv_status:%x, type:%d, status:%d", rStatus, type, status);

    int errCode;
    string errMsg;

    if (val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        //ROS_INFO("8011||backGroundQuerylMsg::query_robot_hardware_status, status:%d, angle:%d", (int)status, pdArmRotate);
        return;
    }
    else
    {
        //ROS_ERROR("8011||backGroundQuerylMsg::query_robot_hardware_status error : code:%d, msg:%s", errCode, errMsg);
    }
}

void backGroundQueryMsg::query_robot_motor_status_debug(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_motor_status_debug==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundQuerylMsg::query_robot_motor_status_debug retVal = NULL");
        return;
    }

    int hStatus = val["horizontal"].asInt();
    int vStatus = val["vertical"].asInt();
    QString currTime = QDateTime::currentDateTime().toString("yyyy-MM-hh hh:mm:ss");
    signal_motorDebugInfo(currTime + " " + motorStatusString[hStatus], currTime + " " + motorStatusString[vStatus]);
}

void backGroundQueryMsg::query_robot_sick_status_debug(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_sick_status_debug==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundQuerylMsg::query_robot_sick_status_debug retVal = NULL");
        return;
    }

    unsigned hStatus = val["status"].asUInt();

    signal_sickDebugInfo(hStatus);
}

void backGroundQueryMsg::query_robot_version(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_WARN("backGroundQuerylMsg::query_robot_version==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("backGroundQuerylMsg::query_robot_version retVal = NULL");
        return;
    }

    unsigned char status = (val["version"].asUInt());

    ROS_INFO("version:0x%02X", status);

    int errCode;
    string errMsg;

    if (val["errCode"].isNull() || val["errCode"].asInt() == 0)
    {
        ROS_INFO("no error");
        return;
    }

    errCode = val["errCode"].asInt();
    errMsg = val["errMsg"].asString();

    if (0 == errCode)
    {
        ROS_INFO("8013||backGroundQuerylMsg::query_robot_version, status:%d, angle:%d", (int)status, 0);
        return;
    }
    else
    {
        ROS_ERROR("8013||backGroundQuerylMsg::query_robot_version error : code:%d, msg:%s", errCode, errMsg);
    }
}
