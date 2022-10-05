#include "LibDLWheelRobotMsg.h"

static uint16_t wheelRobotCtrlMsgId = 0;

LibDLWheelRobotMsg::LibDLWheelRobotMsg()
{

}

LibDLWheelRobotMsg::~LibDLWheelRobotMsg()
{

}

void LibDLWheelRobotMsg::registerHandles()
{

}

void LibDLWheelRobotMsg::registerMsgHandle(int messageID, JsonMsgCallBack callback)
{
    m_baseSocket->registerMsgHandle(messageID, callback);
}
uint16_t LibDLWheelRobotMsg::getMsgId()
{
    boost::mutex::scoped_lock lock(getMsgIdMutex);
    return wheelRobotCtrlMsgId++;
}
