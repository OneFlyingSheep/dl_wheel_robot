#include "CCoreServer.h"
#include <iostream>
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include <QDir>
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"
#include <windows.h>
#include "ole2.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"
//#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreServer.h"
#include "LibDLCreateExcel/GetExcelObject.hpp"
#include <QUuid>

CCoreConnection::CCoreConnection(boost::asio::io_service& io_service) : m_socket(io_service)
{
    m_needClosed = false;
    m_connect = false;
    m_msgRecvBuffer.resize(MAX_BUFF_LENGTH);
    m_msgSendBuffer.resize(MAX_BUFF_LENGTH);
}

CCoreConnection::~CCoreConnection()
{
    //m_socket.get_io_service().stop();
}

CCoreConnection::pointer CCoreConnection::create(boost::asio::io_service& io_service)
{
    return pointer(new CCoreConnection(io_service));
}

boost::asio::ip::tcp::socket& CCoreConnection::socket()
{
    return m_socket;
}

void CCoreConnection::begin()
{
    ROS_WARN(" connect success begin");
    //connect success
    m_connect = true;
    m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
    boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
        boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
        boost::bind(&CCoreConnection::handle_readMsgHeader, shared_from_this(),
            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void CCoreConnection::postMsg(jsonBody message)
{
    boost::mutex::scoped_lock lock(m_sendMutex);

    if (m_needClosed == true)
    {
//        ROS_ERROR("%s socket closed,so message will not send", m_handleName.c_str());
        return;
    }

    bool needSend = false;
    if (m_msgWriteCacheList.size() == 0)
    {
        needSend = true;
    }

    boost::shared_ptr<roboKitMsg> msg(new roboKitMsg);
    msg->msgBody = message;

    m_msgWriteCacheList.push_back(msg);

    if (needSend && m_connect == true)
    {
        sendOneMessageFromeCacheList();
    }
}

void CCoreConnection::postDirectMsg(boost::shared_ptr<roboKitMsg> message)
{
    boost::mutex::scoped_lock lock(m_sendMutex);

    if (m_connect)
    {
        m_msgSendBuffer.resize((message->msgHeader.getLength() + MSG_HEADER_LENGTH), 0);

        memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());

        uint8_t *point = &(*m_msgSendBuffer.begin());
        //    uint8_t *h = &(*m_msgSendBuffer.begin());
        memcpy(point, (void*)message->msgHeader.convertStructToUint8(), MSG_HEADER_LENGTH);
        point += MSG_HEADER_LENGTH;

        memcpy(point, (void*)message->msgBody.getJsonString().c_str(), message->msgHeader.getLength());
        point += message->msgHeader.getLength();

        boost::asio::async_write(m_socket, boost::asio::buffer(m_msgSendBuffer),
            boost::bind(&CCoreConnection::handleMsgSend, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred, m_msgSendBuffer.size()));
    }
}

void CCoreConnection::registerMsgHandles(std::map<int, JsonMsgCallBack> callBackMap)
{
    m_msgHandleMap = callBackMap;
    m_msgHandleMap[Request_robot_config_2d_map_query] = boost::bind(&CCoreConnection::robot_config_2d_map_query_req, this, _1);
	m_msgHandleMap[Request_robot_config_smap_query] = boost::bind(&CCoreConnection::robot_config_smap_query_req, this, _1);
	m_msgHandleMap[Request_robot_create_report_req] = boost::bind(&CCoreConnection::robot_creatr_report_query_req, this, _1);
	m_msgHandleMap[Request_robot_examine_report_isexist_req] = boost::bind(&CCoreConnection::robot_examine_report_isexist_query_req, this, _1);
	m_msgHandleMap[Request_robot_task_edit_insert_from_map_req] = boost::bind(&CCoreConnection::robot_task_edit_insert_from_map_query_req, this, _1);
    m_msgHandleMap[Request_robot_threshold_set_by_device_uuid_req] = boost::bind(&CCoreConnection::Remote_robot_threshold_by_device, this, _1);
    m_msgHandleMap[Request_robot_threshold_set_by_meter_type_req] = boost::bind(&CCoreConnection::Remote_robot_threshold_by_meter, this, _1);
}

void CCoreConnection::closeSession()
{
    m_socket.close();
}

bool CCoreConnection::getConnectStatus()
{
    return m_connect;
}

userLoginRetVal CCoreConnection::getRole()
{
    userLoginRetVal retVal;

    boost::system::error_code err;

    m_msgRecvBuffer.resize(MSG_HEADER_LENGTH, 0);

    memset(&(*m_msgRecvBuffer.begin()), 0, m_msgRecvBuffer.size());

    m_socket.read_some(boost::asio::buffer(m_msgRecvBuffer), err);

    if (err)
    {
        ROS_ERROR("getRole readMessage failed");
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = WHEEL_LOGIN_READ_FAILED;
        retVal.errMsg = err.message();
        send_Login_retVal(retVal);
        return retVal;
    }

    boost::shared_ptr<roboKitMsg> recvMessage(new roboKitMsg);
    unsigned   char * point1 = &(*m_msgRecvBuffer.begin());

    recvMessage->msgHeader.parseHeader(point1);

    uint32_t leftLength = recvMessage->msgHeader.getLength();

    ROS_INFO("handle_readMsgHeader:Recv msgHeader Length=%u", leftLength);

    char ss[COMMON_WR_BUFF_LENGTH_2048] = { 0 };
    int cur = 0;
    for (int i = 0; i < MSG_HEADER_LENGTH; i++)
    {
        cur += sprintf(ss + cur, "%02X ", m_msgRecvBuffer[i]);
    }
    ROS_INFO("recv socket msg head data:[%s]", ss);

    if (leftLength == 0 || leftLength > COMMON_WR_BUFF_LENGTH_2048)
    {
        QString errmsg;
        errmsg.sprintf("msg length over %d readMessage failed", COMMON_WR_BUFF_LENGTH_2048);
        ROS_ERROR(errmsg.toStdString().c_str());
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = WHEEL_LOGIN_READ_FAILED;
        retVal.errMsg = std::string(errmsg.toLocal8Bit());
        send_Login_retVal(retVal);
        return retVal;
    }

    m_msgRecvBuffer.resize(leftLength);

    memset(&(*m_msgRecvBuffer.begin()), 0, m_msgRecvBuffer.size());

    m_socket.read_some(boost::asio::buffer(m_msgRecvBuffer), err);

    if (err)
    {
        ROS_ERROR("getRole readMessage 2 failed");
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = WHEEL_LOGIN_READ_FAILED;
        retVal.errMsg = err.message();
        send_Login_retVal(retVal);
        return retVal;
    }

    int msgMaxLength = recvMessage->msgHeader.getLength() + 1;
    uint8_t *buff = new uint8_t[msgMaxLength];
    memset(buff, '\0', msgMaxLength);
    memcpy(buff, &(*m_msgRecvBuffer.begin()), recvMessage->msgHeader.getLength());
    //ROS_INFO("msg_type:%d,length:%d", message->msgHeader.getType(), message->msgHeader.getLength());
    recvMessage->msgBody.fromJsonStringToJsonVal(buff, msgMaxLength);
    recvMessage->msgBody.setMsgNumber(recvMessage->msgHeader.getNumber());
    recvMessage->msgBody.setMsgType(recvMessage->msgHeader.getType());

    memset(ss, 0, COMMON_WR_BUFF_LENGTH_2048);
    cur = 0;
    for (int i = 0; i < (msgMaxLength < COMMON_WR_BUFF_LENGTH_2048 ? msgMaxLength : COMMON_WR_BUFF_LENGTH_2048); i++)
        cur += sprintf(ss + cur, "%02X ", buff[i]);
    ROS_INFO("recv socket json body data:[%s], length:%d", ss, msgMaxLength);
    //continue read
    delete[]buff;
    buff = NULL;

    Json::Value val = recvMessage->msgBody.getJsonVal();

    //judge;
    QString userName = QString::fromStdString(val["userName"].asString());
    QString password = QString::fromStdString(val["password"].asString());

    WHEEL_ROBOT_DB.getUserRole(userName, password, retVal);
    
    send_Login_retVal(retVal);

    if (err)
    {
        ROS_ERROR("dologin sendMessage failed");
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = WHEEL_LOGIN_WRITE_FAILED;
        retVal.errMsg = err.message();
        return retVal;
    }
    return retVal;
}

void CCoreConnection::robot_config_2d_map_query_req(boost::shared_ptr<roboKitMsg> msg)
{
    // query current map
    Json::Value mapList;
    jsonBody body;
    body.setMsgType(Request_robot_config_2d_map_query + WHEELROBOT_PointADD);
    
	
    QDir sourceDir(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath+"/2dmap/");
    QStringList fileNames = sourceDir.entryList(QDir::Files);

    for (int i = 0; i < fileNames.size(); i++)
    {
        mapList[i] = std::string(fileNames[i].toLocal8Bit());
    }
    body.jsonAppendElement("map_name", mapList);
    postMsg(body);
}

void CCoreConnection::robot_config_smap_query_req(boost::shared_ptr<roboKitMsg> msg)
{
    // query current smap
    Json::Value mapList;
    jsonBody body;
    body.setMsgType(Request_robot_config_smap_query + WHEELROBOT_PointADD);

    QDir sourceDir(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath+"/smap/");
    QStringList fileNames = sourceDir.entryList(QDir::Files);

    for (int i = 0; i < fileNames.size(); i++)
    {
        mapList[i] = std::string(fileNames[i].toLocal8Bit());
    }
    body.jsonAppendElement("smap_name", mapList);
    postMsg(body);
}

void CCoreConnection::robot_creatr_report_query_req(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
 	QString task_uuid = QString::fromStdString(val["task_uuid"].asString());
 	QString task_name = QString::fromStdString(val["task_name"].asString());
 	QString task_time = QString::fromStdString(val["task_time"].asString());
	task_name = task_name + task_time.remove("-").remove("T").remove(":").remove(" ");

// 	HRESULT r = OleInitialize(0);
// 	if (r != S_OK && r != S_FALSE)
// 	{
// 	}
// 	SearchRecordCreateExcel excel;
// 	bool bl = excel.CreateNewExcelForTask(task_uuid, task_name);
// 	OleUninitialize();
// 	WheelCreateReportWithExcel _cl;
// 	bool bl = _cl.createReportForTask(task_uuid, task_name);

	bool bl = Excel::createExcelReport(Excel::ChooseExcelEnum::USEQT5, task_uuid, task_name);

	jsonBody body;
	body.setMsgType(Request_robot_create_report_req + WHEELROBOT_PointADD);
	body.jsonAppendElement("report_bool", bl);
	body.jsonAppendElement("report_name", task_name.toStdString());
	postMsg(body);
}

void CCoreConnection::robot_examine_report_isexist_query_req(boost::shared_ptr<roboKitMsg> msg)
{
	Json::Value val = msg->msgBody.getJsonVal();
	QString reportName = QString::fromStdString(val["reportName"].asString());
	QString task_uuid = QString::fromStdString(val["task_uuid"].asString());
	QString task_name = QString::fromStdString(val["task_name"].asString());
	QString task_time = QString::fromStdString(val["task_time"].asString());
	
 	QFileInfo info(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + QString("/report/%1.xlsx").arg(reportName.remove(" ")));
 	bool bl = info.isFile();

	jsonBody body;
	body.setMsgType(Request_robot_examine_report_isexist_req + WHEELROBOT_PointADD);
	body.jsonAppendElement("is_exist", bl);
	body.jsonAppendElement("report_name", reportName.toStdString());
	body.jsonAppendElement("task_uuid", task_uuid.toStdString());
	body.jsonAppendElement("task_name", task_name.toStdString());
	body.jsonAppendElement("task_time", task_time.toStdString());
	postMsg(body);
}

void CCoreConnection::robot_task_edit_insert_from_map_query_req(boost::shared_ptr<roboKitMsg> msg)
{
	ROS_INFO("LibDLWheelRobotCoreServer::robot_task_edit_insert_from_map_query_req==Str:%s", msg->msgBody.getJsonString().c_str());
	Json::Value val = msg->msgBody.getJsonVal();

	if (val.isNull())
	{
		ROS_ERROR("LibDLWheelRobotCoreServer::robot_task_edit_insert_from_map_query_req retVal = NULL");
		return;
	}
	WheelTaskEditStruct edit;
	edit.task_edit_uuid = WHEEL_DEVICE_CONFIG.getUUid();
	edit.task_edit_name = QString::fromStdString(val["task_edit_name"].asString());
	edit.task_edit_date = QString::fromStdString(val["task_edit_date"].asString());
	edit.task_edit_type_id = WHEEL_USER_DEFINED_TASK;

	QString retMsg;
	QList<QString> device_uuid;
	Json::Value::iterator devItr = val["device_uuid"].begin();
	for (; devItr != val["device_uuid"].end(); devItr++)
	{
		device_uuid.append(QString::fromStdString(devItr->asString()));
	}
 	bool bRet = WHEEL_ROBOT_DB.insertTaskEditAndInsertTaskDevicesDB(edit, device_uuid, retMsg);


	WheelRobotAssignTask task;

	Json::Value dev;
	for (int i = 0; i < device_uuid.size(); i++)
	{
		dev[i] = device_uuid[i].toStdString();
	}
	jsonBody body;
	body.setMsgType(Request_robot_task_edit_insert_from_map_req + WHEELROBOT_PointADD);
	body.jsonAppendElement("task_uuid", WHEEL_DEVICE_CONFIG.getUUid().toStdString().c_str());
	body.jsonAppendElement("task_name", edit.task_edit_name.toStdString().c_str());
	body.jsonAppendElement("task_edit_uuid", edit.task_edit_uuid.toStdString().c_str());
	body.jsonAppendElement("priority", (int)WHEEL_ROBOT_TASK_1ST);
	body.jsonAppendElement("devices", dev);
	body.jsonAppendElement("dev_optimize", true);
	body.jsonAppendElement("task_end_action", (int)WHEEL_ROBOT_TASK_END_TYPE_STANDBY);
	body.jsonAppendElement("breakTask", true);
	postMsg(body);
}

void CCoreConnection::Remote_robot_threshold_by_device(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_threshold_by_device==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_threshold_by_device = NULL");
        return;
    }
    QVector<THRESHOLD_ELEMENT> alarmNormalList;
    QVector<THRESHOLD_ELEMENT> alarmWarningList;
    QVector<THRESHOLD_ELEMENT> alarmCommonList;
    QVector<THRESHOLD_ELEMENT> alarmSerialList;
    QVector<THRESHOLD_ELEMENT> alarmDangerList;

    Json::Value alarmNormalRoot = val["alarmNormal"];
    for (int i = 0; i < alarmNormalRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmNormalRoot[i]["type"].asInt();
        for (int j = 0; j < alarmNormalRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmNormalRoot[i]["val"][j].asString().c_str()));
        }
        alarmNormalList.push_back(ele);
    }

    Json::Value alarmWarningRoot = val["alarmWarning"];
    for (int i = 0; i < alarmWarningRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmWarningRoot[i]["type"].asInt();
        for (int j = 0; j < alarmWarningRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmWarningRoot[i]["val"][j].asString().c_str()));
        }
        alarmWarningList.push_back(ele);
    }

    Json::Value alarmCommonRoot = val["alarmCommon"];
    for (int i = 0; i < alarmCommonRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmCommonRoot[i]["type"].asInt();
        for (int j = 0; j < alarmCommonRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmCommonRoot[i]["val"][j].asString().c_str()));
        }
        alarmCommonList.push_back(ele);
    }

    Json::Value alarmSerialRoot = val["alarmSerial"];
    for (int i = 0; i < alarmSerialRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmSerialRoot[i]["type"].asInt();
        for (int j = 0; j < alarmSerialRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmSerialRoot[i]["val"][j].asString().c_str()));
        }
        alarmSerialList.push_back(ele);
    }

    Json::Value alarmDangerRoot = val["alarmDanger"];
    for (int i = 0; i < alarmDangerRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmDangerRoot[i]["type"].asInt();
        for (int j = 0; j < alarmDangerRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmDangerRoot[i]["val"][j].asString().c_str()));
        }
        alarmDangerList.push_back(ele);
    }

    QString luaString;
    QString errMsg;
    bool bConvertSucceed = m_convert2Lua.getLuaScript(alarmNormalList, alarmWarningList, alarmCommonList, alarmSerialList, alarmDangerList, luaString, errMsg);

    if (bConvertSucceed)
    {
        QString threshold_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
        QString filePath = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/threshold/";
        QDir dir(filePath);
        if (!dir.exists())
        {
            dir.mkpath(filePath);
        }

        QFile file(filePath + threshold_uuid + ".lua");
        file.open(QIODevice::ReadWrite | QIODevice::Truncate);

        QTextStream f(&file);
        f << luaString;
        file.close();

        Json::FastWriter writer;

        QString jsonAlarmNormal = QString::fromLatin1(writer.write(alarmNormalRoot).c_str());
        QString jsonAlarmWarning = QString::fromLatin1(writer.write(alarmWarningRoot).c_str());
        QString jsonAlarmCommon = QString::fromLatin1(writer.write(alarmCommonRoot).c_str());
        QString jsonAlarmSerial = QString::fromLatin1(writer.write(alarmSerialRoot).c_str());
        QString jsonAlarmDanger = QString::fromLatin1(writer.write(alarmDangerRoot).c_str());

        QVector<QString> device_uuid;

        for (int i = 0; i < val["device_uuid"].size(); i++)
        {
            device_uuid.push_back(QString::fromLatin1(val["device_uuid"][i].asString().c_str()));
        }

        //         QVector<THRESHOLD_ELEMENT> t;
        //         m_convert2Lua.getStructFromJson(jsonAlarmNormal, t);
        //         m_convert2Lua.getStructFromJson(jsonAlarmWarning, t);
        //         m_convert2Lua.getStructFromJson(jsonAlarmCommon, t);
        //         m_convert2Lua.getStructFromJson(jsonAlarmSerial, t);
        //         m_convert2Lua.getStructFromJson(jsonAlarmDanger, t);

        WHEEL_ROBOT_DB.insertThresholdByDeviceUuid(jsonAlarmNormal, jsonAlarmWarning, jsonAlarmCommon, jsonAlarmSerial, jsonAlarmDanger,
            device_uuid, threshold_uuid);
    }

    jsonBody retBody;
    retBody.setMsgType((int)Request_robot_threshold_set_by_device_uuid_req + WHEELROBOT_PointADD);

    retBody.jsonAppendElement("bSucceed", bConvertSucceed);
    retBody.jsonAppendElement("retMsg", std::string(errMsg.toLocal8Bit()));

    postMsg(retBody);
}

void CCoreConnection::Remote_robot_threshold_by_meter(boost::shared_ptr<roboKitMsg> msg)
{
    ROS_INFO("LibDLWheelRobotCoreServer::Remote_robot_threshold_by_meter==Str:%s", msg->msgBody.getJsonString().c_str());
    Json::Value val = msg->msgBody.getJsonVal();

    if (val.isNull())
    {
        ROS_ERROR("LibDLWheelRobotCoreServer::Remote_robot_threshold_by_meter = NULL");
        return;
    }
    QVector<THRESHOLD_ELEMENT> alarmNormalList;
    QVector<THRESHOLD_ELEMENT> alarmWarningList;
    QVector<THRESHOLD_ELEMENT> alarmCommonList;
    QVector<THRESHOLD_ELEMENT> alarmSerialList;
    QVector<THRESHOLD_ELEMENT> alarmDangerList;

    Json::Value alarmNormalRoot = val["alarmNormal"];
    for (int i = 0; i < alarmNormalRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmNormalRoot[i]["type"].asInt();
        for (int j = 0; j < alarmNormalRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmNormalRoot[i]["val"][j].asString().c_str()));
        }
        alarmNormalList.push_back(ele);
    }

    Json::Value alarmWarningRoot = val["alarmWarning"];
    for (int i = 0; i < alarmWarningRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmWarningRoot[i]["type"].asInt();
        for (int j = 0; j < alarmWarningRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmWarningRoot[i]["val"][j].asString().c_str()));
        }
        alarmWarningList.push_back(ele);
    }

    Json::Value alarmCommonRoot = val["alarmCommon"];
    for (int i = 0; i < alarmCommonRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmCommonRoot[i]["type"].asInt();
        for (int j = 0; j < alarmCommonRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmCommonRoot[i]["val"][j].asString().c_str()));
        }
        alarmCommonList.push_back(ele);
    }

    Json::Value alarmSerialRoot = val["alarmSerial"];
    for (int i = 0; i < alarmSerialRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmSerialRoot[i]["type"].asInt();
        for (int j = 0; j < alarmSerialRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmSerialRoot[i]["val"][j].asString().c_str()));
        }
        alarmSerialList.push_back(ele);
    }

    Json::Value alarmDangerRoot = val["alarmDanger"];
    for (int i = 0; i < alarmDangerRoot.size(); i++)
    {
        THRESHOLD_ELEMENT ele;
        ele.type = (THRESHOLD_TYPE)alarmDangerRoot[i]["type"].asInt();
        for (int j = 0; j < alarmDangerRoot[i]["val"].size(); j++)
        {
            ele.val.push_back(QString::fromLatin1(alarmDangerRoot[i]["val"][j].asString().c_str()));
        }
        alarmDangerList.push_back(ele);
    }

    QString luaString;
    QString errMsg;
    bool bConvertSucceed = m_convert2Lua.getLuaScript(alarmNormalList, alarmWarningList, alarmCommonList, alarmSerialList, alarmDangerList, luaString, errMsg);

    if (bConvertSucceed)
    {
        QString threshold_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
        QFile file(WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath + "/threshold/" + threshold_uuid + ".lua");
        file.open(QIODevice::ReadWrite | QIODevice::Truncate);

        QTextStream f(&file);
        f << luaString;
        file.close();

        Json::FastWriter writer;

        QString jsonAlarmNormal = QString::fromStdString(writer.write(alarmNormalRoot));
        QString jsonAlarmWarning = QString::fromStdString(writer.write(alarmWarningRoot));
        QString jsonAlarmCommon = QString::fromStdString(writer.write(alarmCommonRoot));
        QString jsonAlarmSerial = QString::fromStdString(writer.write(alarmSerialRoot));
        QString jsonAlarmDanger = QString::fromStdString(writer.write(alarmDangerRoot));

        WHEEL_ROBOT_DB.insertThresholdByMeterType(jsonAlarmNormal, jsonAlarmWarning, jsonAlarmCommon, jsonAlarmSerial, jsonAlarmDanger,
            (WheelRobotMeterType)val["meter_type"].asInt(), threshold_uuid);
    }

    jsonBody retBody;
    retBody.setMsgType((int)Request_robot_threshold_set_by_meter_type_req + WHEELROBOT_PointADD);

    retBody.jsonAppendElement("bSucceed", bConvertSucceed);
    retBody.jsonAppendElement("retMsg", std::string(errMsg.toLocal8Bit()));

    postMsg(retBody);
}

void CCoreConnection::handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred)
{
    ROS_INFO("read");

    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        if (m_connect)
        {
            checkConnection();
            m_socket.close();
            m_connect = false;
        }
        return;
    }

    if (m_needClosed == true)
    {
        ROS_WARN("hh---close sockets");
        m_connect = false;
        m_socket.close();
        return;
    }
    //m_lastActiveTime = getTickCount();

    if (bytes_transferred != MSG_HEADER_LENGTH)
    {
        ROS_INFO("bytes_transferred != JsonMsg::headerLength Read Message Head Failed");
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
            boost::bind(&CCoreConnection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
    }

    if (m_msgRecvBuffer[0] != 0x5A || m_msgRecvBuffer[1] != 0x01)
    {
        ROS_ERROR("m_msgRecvBuffer not match");
        int count = 0;
        bool bFind = false;
        boost::system::error_code err_;
        while (count < MSG_HEADER_LENGTH)
        {
            if (m_msgRecvBuffer[count] == 0x5A)
            {
                bFind = true;
                break;
            }            
            count++;
        }
        ROS_ERROR("m_msgRecvBuffer count:%d", count);
        if (bFind)
        {
            ROS_ERROR("m_msgRecvBuffer find!!");
            m_msgRecvBuffer.resize(MSG_HEADER_LENGTH - count);
            m_socket.read_some(boost::asio::buffer(m_msgRecvBuffer), err_);

            if (err_)
            {
                ROS_ERROR("m_msgRecvBuffer read_some err");
                ROS_ERROR(" 0x5A sock.read_some(): An error occurred:%s", err.message().c_str());
                if (m_connect)
                {
                    checkConnection();
                    m_socket.close();
                    m_connect = false;
                }
                return;
            }

            if (m_msgRecvBuffer[0] != 0x5A || m_msgRecvBuffer[1] != 0x01)
            {
                ROS_ERROR("m_msgRecvBuffer not match 2");
                //error
                m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
                //printf("head recv  %d \r\n %s",bytes_transferred, preetyDump(&m_msgRecvBuffer[0], bytes_transferred).c_str() );
                boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
                    boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
                    boost::bind(&CCoreConnection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
                return;
            }
        }
        else
        {
            ROS_ERROR("m_msgRecvBuffer not find!!!");
            //error
            m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
            //printf("head recv  %d \r\n %s",bytes_transferred, preetyDump(&m_msgRecvBuffer[0], bytes_transferred).c_str() );
            boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
                boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
                boost::bind(&CCoreConnection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            return;
        }
    }

    boost::shared_ptr<roboKitMsg> recvMessage(new roboKitMsg);
    unsigned   char * point = &(*m_msgRecvBuffer.begin());

    recvMessage->msgHeader.parseHeader(point);

    int leftLength = recvMessage->msgHeader.getLength();

    ROS_INFO("Recv jsonBody Length=%d, number=%d, type=%d", leftLength, recvMessage->msgHeader.getNumber(), recvMessage->msgHeader.getType());


    if (leftLength < 0 || leftLength > MAX_BUFF_LENGTH - 1)
    {
        //error
        m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
        //printf("head recv  %d \r\n %s",bytes_transferred, preetyDump(&m_msgRecvBuffer[0], bytes_transferred).c_str() );
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
            boost::bind(&CCoreConnection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
    }
    else
    {
        //normal
        m_msgRecvBuffer.resize(leftLength);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(leftLength),
            boost::bind(&CCoreConnection::handle_readJsonBody, shared_from_this(), boost::asio::placeholders::error, recvMessage, boost::asio::placeholders::bytes_transferred)
        );
    }
}

void CCoreConnection::handle_readJsonBody(const boost::system::error_code& err, boost::shared_ptr<roboKitMsg> message, std::size_t bytes_transferred)
{
    ROS_INFO("tcp_connection::handle_readJsonBody");
    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        m_socket.close();
        //m_socket.async_connect(m_endpoint, boost::bind(&CSocket::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
        return;
    }

    //m_lastActiveTime = getTickCount();
	if (message->msgHeader.getLength() > 0)
	{
        int msgMaxLength = message->msgHeader.getLength() + 1;
        uint8_t *buff = new uint8_t[msgMaxLength];
        memset(buff, '\0', msgMaxLength);
        memcpy(buff, &(*m_msgRecvBuffer.begin()), message->msgHeader.getLength());
        //ROS_INFO("msg_type:%d,length:%d", message->msgHeader.getType(), message->msgHeader.getLength());
        message->msgBody.fromJsonStringToJsonVal(buff, msgMaxLength);
        message->msgBody.setMsgNumber(message->msgHeader.getNumber());
        message->msgBody.setMsgType(message->msgHeader.getType());
        //printf("body recv %d -------------------B \r\n %s", bytes_transferred, preetyDump(&m_msgRecvBuffer[0], bytes_transferred).c_str());
        //continue read
        delete[]buff;
        buff = NULL;
	}
    m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
    boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
        boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
        boost::bind(&CCoreConnection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    dispathRoboKitMessage(message);
}

void CCoreConnection::generateMsgFromJsonVal(boost::shared_ptr<roboKitMsg> msg)
{
    msg->msgHeader.setNumber(msg->msgBody.getMsgNumber());
    if (msg->msgBody.getJsonVal().isNull())
    {
        msg->msgHeader.setLength(0);
    }
    else
    {
        ROS_INFO("getJsonString:%s", msg->msgBody.getJsonString());
        msg->msgHeader.setLength(msg->msgBody.getJsonString().length());
    }
    msg->msgHeader.setType(msg->msgBody.getMsgType());

    m_msgSendBuffer.resize((msg->msgHeader.getLength() + MSG_HEADER_LENGTH), 0);

    memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());

    uint8_t *point = &(*m_msgSendBuffer.begin());
    memcpy(point, (void*)msg->msgHeader.convertStructToUint8(), MSG_HEADER_LENGTH);
    point += MSG_HEADER_LENGTH;

    memcpy(point, (void*)msg->msgBody.getJsonString().c_str(), msg->msgHeader.getLength());
    point += msg->msgHeader.getLength();

    ROS_INFO("generateMsgFromJsonVal");
}

void CCoreConnection::handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred, std::size_t sendBuffSize)
{
    ROS_INFO("handleMsgSend");
    boost::mutex::scoped_lock lock(m_sendMutex);

    if (!m_connect)
    {
        return;
    }

    if (error)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", error.message().c_str());
        m_connect = false;
        checkConnection();
        return;
    }
    else
    {
        if (bytes_transferred != sendBuffSize)
        {
            ROS_ERROR("Not all data has been right!!!need write %d ,but only write:%d", (int)sendBuffSize, (int)bytes_transferred);
            //ROS_ERROR("%s", preetyDump(&m_msgSendBuffer[0], bytes_transferred).c_str());
            //std::cout<<"Not all data has been right!!!need write "<<m_msgSendBuffer.size()<<"  but only write "<<bytes_transferred<<"\r\n";
        }
        else
        {
        }
        if (m_msgWriteCacheList.empty())
        {
        }
        else
        {
            //if less then 25,means keep alive message
//            if (m_msgWriteCacheList.front()->msgHeader.getLength() < 25)
//            {
//                ROS_WARN("Send to IP:%s \r\n%s", m_socket.local_endpoint().address().to_string().c_str(), m_msgWriteCacheList.front()->msgBody.getJsonString().c_str());
//                m_msgWriteCacheList.erase(m_msgWriteCacheList.begin());
//            }
//            else
//            {
//            ROS_INFO("Send to IP:%s \r\n%s", m_socket.local_endpoint().address().to_string().c_str(), m_msgWriteCacheList.front()->msgBody.getJsonString().c_str());
            m_msgWriteCacheList.erase(m_msgWriteCacheList.begin());
//            }
        }
        if (m_msgWriteCacheList.empty())
        {
        }
        else
        {
            sendOneMessageFromeCacheList();
        }
    }
}

void CCoreConnection::sendOneMessageFromeCacheList()
{
    ROS_INFO("sendOneMessageFromeCacheList : %d", m_msgWriteCacheList.size());
    if (m_msgWriteCacheList.size() > 0)
    {
        boost::shared_ptr<roboKitMsg> message = m_msgWriteCacheList.front();
        generateMsgFromJsonVal(message);
        ROS_INFO("sendOneMessageFromeCacheList message: %d", m_msgWriteCacheList.size());
        boost::asio::async_write(m_socket, boost::asio::buffer(m_msgSendBuffer),
            boost::bind(&CCoreConnection::handleMsgSend, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred,
                m_msgSendBuffer.size()));
    }
}

void CCoreConnection::dispathRoboKitMessage(boost::shared_ptr<roboKitMsg> message)
{
    ROS_INFO("dispathRoboKitMessage");
    m_socket.get_io_service().post(boost::bind(&CCoreConnection::dispathRoboKitMessageIMPL, shared_from_this(), message));
}

void CCoreConnection::dispathRoboKitMessageIMPL(boost::shared_ptr<roboKitMsg> message)
{
    ROS_INFO("1111recv message  %s, type :%d, id:%d", message->msgBody.getJsonString().c_str(), message->msgHeader.getType(), message->msgHeader.getNumber());

    int msgType = message->msgHeader.getType();
    if (m_msgHandleMap.find(msgType) != m_msgHandleMap.end())
    {

        try
        {
            m_msgHandleMap[msgType](message);
        }
        catch (...)
        {
            ROS_ERROR("dispathRoboKitMessageIMPL err, exception occured");
        }
    }
    else
    {
        ROS_ERROR("No msgHandleMap. type:%d", msgType);
    }
}

void CCoreConnection::send_Login_retVal(userLoginRetVal retVal)
{
    roboKitMsg msg;
    boost::system::error_code err;
    msg.msgBody.setMsgNumber(0);
    msg.msgBody.setMsgType(0x4268);

    msg.msgBody.jsonAppendElement("role", retVal.role);
    msg.msgBody.jsonAppendElement("errCode", retVal.retCode);
    msg.msgBody.jsonAppendElement("errMsg", retVal.errMsg);

    if (retVal.role != WHEEL_USER_NONE)
    {
        msg.msgBody.jsonAppendElement("coreServerIp", std::string(WHEEL_ROBOT_CORE_CONFIG.getCfg().coreServerIp.toLocal8Bit()));
        msg.msgBody.jsonAppendElement("coreServerPort", WHEEL_ROBOT_CORE_CONFIG.getCfg().coreServerPort);
        msg.msgBody.jsonAppendElement("rcfServerPort", WHEEL_ROBOT_CORE_CONFIG.getCfg().rcfServerPort);
        msg.msgBody.jsonAppendElement("databaseRemoteIp", std::string(WHEEL_ROBOT_CORE_CONFIG.getCfg().databaseRemoteIp.toLocal8Bit()));
        msg.msgBody.jsonAppendElement("databasePort", WHEEL_ROBOT_CORE_CONFIG.getCfg().databasePort);
        msg.msgBody.jsonAppendElement("databaseUsername", std::string(WHEEL_ROBOT_CORE_CONFIG.getCfg().databaseUsername.toLocal8Bit()));
        msg.msgBody.jsonAppendElement("databasePassword", std::string(WHEEL_ROBOT_CORE_CONFIG.getCfg().databasePassword.toLocal8Bit()));
        msg.msgBody.jsonAppendElement("databaseName", std::string(WHEEL_ROBOT_CORE_CONFIG.getCfg().databaseName.toLocal8Bit()));
        msg.msgBody.jsonAppendElement("infraredManufacturer", WHEEL_ROBOT_CORE_CONFIG.getCfg().infraredManufacturer);

        
        Json::Value robotListRoot;
        QList<WheelRobotCoreRobotConfig> tmpList = WHEEL_ROBOT_CORE_CONFIG.getAllRobotCfg();
        for (int i = 0; i < tmpList.size(); i++)
        {
            robotListRoot[i] = std::string(tmpList[i].robotName.toLocal8Bit());
        }
        msg.msgBody.jsonAppendElement("robotList", robotListRoot);

        msg.msgBody.jsonAppendElement("robotIp", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().robotIp.toLocal8Bit()));
        msg.msgBody.jsonAppendElement("robotName", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().robotName.toLocal8Bit()));
        msg.msgBody.jsonAppendElement("hcCtrlPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcCtrlPort);
        msg.msgBody.jsonAppendElement("hcRtspPort", WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcRtspPort);
        msg.msgBody.jsonAppendElement("hcUserName", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcUserName.toLocal8Bit()));
        msg.msgBody.jsonAppendElement("hcPassword", std::string(WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcPassword.toLocal8Bit()));
    }

    msg.msgHeader.setNumber(msg.msgBody.getMsgNumber());
    msg.msgHeader.setLength(msg.msgBody.getJsonString().length());

    msg.msgHeader.setType(msg.msgBody.getMsgType());

    m_msgSendBuffer.resize((msg.msgHeader.getLength() + MSG_HEADER_LENGTH), 0);

    memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());

    uint8_t *point = &(*m_msgSendBuffer.begin());
    //    uint8_t *h = &(*m_msgSendBuffer.begin());
    memcpy(point, (void*)msg.msgHeader.convertStructToUint8(), MSG_HEADER_LENGTH);
    point += MSG_HEADER_LENGTH;

    memcpy(point, (void*)msg.msgBody.getJsonString().c_str(), msg.msgHeader.getLength());
    point += msg.msgHeader.getLength();

    m_socket.write_some(boost::asio::buffer(m_msgSendBuffer), err);
}

CCoreServer::CCoreServer(boost::asio::ip::tcp::endpoint &endpoint, boost::asio::io_service &io_service) : m_acceptor(io_service, endpoint), m_maxSessionAllowed(0)
{
    ROS_INFO("CCoreServer::CCoreServer");
    m_endpoint = endpoint;
}

CCoreServer::~CCoreServer()
{
    ROS_INFO("Server closed");
    closeSocket();
}

void CCoreServer::initSocket()
{
    ROS_INFO("CCoreServer::initSocket");
    CCoreConnection::pointer new_connection = CCoreConnection::create(m_acceptor.get_io_service());

    m_acceptor.async_accept(new_connection->socket(),
        boost::bind(&CCoreServer::handle_accept, shared_from_this(), new_connection,
            boost::asio::placeholders::error));

    if (m_timer == NULL)
    {
        m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_acceptor.get_io_service(), boost::posix_time::seconds(10)));
    }

    m_timer->async_wait(boost::bind(&CCoreServer::connectAliveCallback, shared_from_this(), boost::asio::placeholders::error));
}

void CCoreServer::postMsg2AllConnection(jsonBody message)
{
    postMsg2Engineer(message);
    postMsg2UserConnection(message);
    postMsg2ManagerConnection(message);
    postMsg2SuperManagerConnection(message);
}

void CCoreServer::postMsg2Engineer(jsonBody message)
{
    boost::mutex::scoped_lock lock(m_EngineerMutex);
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator itr = m_EngineerSocketVec.begin();
    for (itr; itr != m_EngineerSocketVec.end(); itr++)
    {
        (*itr)->postMsg(message);
    }
}

void CCoreServer::postMsg2UserConnection(jsonBody message)
{
    boost::mutex::scoped_lock lock(m_UserMutex);
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator itr = m_UserSocketVec.begin();
    for (itr; itr != m_UserSocketVec.end(); itr++)
    {
        (*itr)->postMsg(message);
    }
}

void CCoreServer::postMsg2ManagerConnection(jsonBody message)
{
    boost::mutex::scoped_lock lock(m_ManagerMutex);
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator itr = m_ManagerSocketVec.begin();
    for (itr; itr != m_ManagerSocketVec.end(); itr++)
    {
        (*itr)->postMsg(message);
    }
}

void CCoreServer::postMsg2SuperManagerConnection(jsonBody message)
{
    boost::mutex::scoped_lock lock(m_SuperManagerMutex);
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator itr = m_SuperManagerSocketVec.begin();
    for (itr; itr != m_SuperManagerSocketVec.end(); itr++)
    {
        (*itr)->postMsg(message);
    }
}

void CCoreServer::postDirectMsg2AllConnection(boost::shared_ptr<roboKitMsg> message)
{
    postDirectMsg2Engineer(message);
    postDirectMsg2UserConnection(message);
    postDirectMsg2ManagerConnection(message);
    postDirectMsg2SuperManagerConnection(message);
}

void CCoreServer::postDirectMsg2Engineer(boost::shared_ptr<roboKitMsg> message)
{
    boost::mutex::scoped_lock lock(m_EngineerMutex);
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator itr = m_EngineerSocketVec.begin();
    for (itr; itr != m_EngineerSocketVec.end(); itr++)
    {
        (*itr)->postDirectMsg(message);
    }
}

void CCoreServer::postDirectMsg2UserConnection(boost::shared_ptr<roboKitMsg> message)
{
    boost::mutex::scoped_lock lock(m_UserMutex);
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator itr = m_UserSocketVec.begin();
    for (itr; itr != m_UserSocketVec.end(); itr++)
    {
        (*itr)->postDirectMsg(message);
    }
}

void CCoreServer::postDirectMsg2ManagerConnection(boost::shared_ptr<roboKitMsg> message)
{
    boost::mutex::scoped_lock lock(m_ManagerMutex);
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator itr = m_ManagerSocketVec.begin();
    for (itr; itr != m_ManagerSocketVec.end(); itr++)
    {
        (*itr)->postDirectMsg(message);
    }
}

void CCoreServer::postDirectMsg2SuperManagerConnection(boost::shared_ptr<roboKitMsg> message)
{
    boost::mutex::scoped_lock lock(m_SuperManagerMutex);
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator itr = m_SuperManagerSocketVec.begin();
    for (itr; itr != m_SuperManagerSocketVec.end(); itr++)
    {
        (*itr)->postDirectMsg(message);
    }
}

void CCoreServer::registerMsgHandle(int messageID, JsonMsgCallBack callback)
{
    m_msgHandleMap[messageID] = callback;
}

void CCoreServer::closeSocket()
{
    m_SuperManagerMutex.lock();
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator superManagerItr = m_SuperManagerSocketVec.begin();
    for (superManagerItr; superManagerItr != m_SuperManagerSocketVec.end(); superManagerItr++)
    {
        (*superManagerItr)->closeSession();
    }
    m_SuperManagerMutex.unlock();

    m_ManagerMutex.lock();
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator managerItr = m_ManagerSocketVec.begin();
    for (managerItr; managerItr != m_ManagerSocketVec.end(); managerItr++)
    {
        (*managerItr)->closeSession();
    }
    m_ManagerMutex.unlock();

    m_UserMutex.lock();
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator userItr = m_UserSocketVec.begin();
    for (userItr; userItr != m_UserSocketVec.end(); userItr++)
    {
        (*userItr)->closeSession();
    }
    m_UserMutex.unlock();

    m_EngineerMutex.lock();
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator engineerItr = m_EngineerSocketVec.begin();
    for (engineerItr; engineerItr != m_EngineerSocketVec.end(); engineerItr++)
    {
        (*engineerItr)->closeSession();
    }
    m_EngineerMutex.unlock();
}

void CCoreServer::setMaxSession(int max)
{
    m_maxSessionAllowed = max;
}

void CCoreServer::connectAliveCallback(boost::system::error_code err)
{
    ROS_INFO("connectAliveCallback");
    checkConnection();
    if (m_timer == NULL)
    {
        m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_acceptor.get_io_service(), boost::posix_time::seconds(10)));
    }
    m_timer->expires_from_now(boost::posix_time::seconds(10));
    m_timer->async_wait(boost::bind(&CCoreServer::connectAliveCallback, shared_from_this(), boost::asio::placeholders::error));
}

void CCoreServer::handle_accept(CCoreConnection::pointer new_connection, const boost::system::error_code& error)
{
    ROS_INFO("handle_accept");
    //ROS_WARN("max:%d, size:%d, err:%d", m_maxSessionAllowed, (int)m_socketVec.size(), error.value());
    if (error)
    {  
        ROS_WARN(" accept failed");
        ROS_ERROR(" accept failed,continue try address is %s:%d ", m_endpoint.address().to_string().c_str(), m_endpoint.port());
    }
    //else if (m_socketVec.size() >= m_maxSessionAllowed && m_maxSessionAllowed > 0)
    //{
    //    ROS_WARN(" accept failed, max session reached");
    //}
    else
    {
        int32_t timeout = 30 * 1000;
        setsockopt(new_connection->socket().native(), SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
        setsockopt(new_connection->socket().native(), SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));

        userLoginRetVal ret = new_connection->getRole();

        //new_connection->begin();
        //new_connection->registerMsgHandles(m_msgHandleMap);
        //
        //m_socketVec.push_back(new_connection);
        
        switch (ret.role)
        {
        case WHEEL_USER_NONE:
            new_connection->socket().close();
            break;
        case WHEEL_USER_NORMAL:
            new_connection->begin();
            m_UserMutex.lock();
            m_UserSocketVec.push_back(new_connection);
            new_connection->registerMsgHandles(m_msgHandleMap);
            new_connection->checkConnection = boost::bind(&CCoreServer::checkConnection, shared_from_this());
            m_UserMutex.unlock();
            break;
        case WHEEL_USER_MANAGER:
            new_connection->begin();
            m_ManagerMutex.lock();
            m_ManagerSocketVec.push_back(new_connection);
            new_connection->registerMsgHandles(m_msgHandleMap);
            new_connection->checkConnection = boost::bind(&CCoreServer::checkConnection, shared_from_this());
            m_ManagerMutex.unlock();
            break;
        case WHEEL_USER_SUPER_MANAGER:
            new_connection->begin();
            m_SuperManagerMutex.lock();
            m_SuperManagerSocketVec.push_back(new_connection);
            new_connection->registerMsgHandles(m_msgHandleMap);
            new_connection->checkConnection = boost::bind(&CCoreServer::checkConnection, shared_from_this());
            m_SuperManagerMutex.unlock();
            break;
        case WHEEL_USER_ENGINEER:
            if (m_EngineerSocketVec.size() < 1)
            {
                new_connection->begin();
                m_EngineerMutex.lock();
                m_EngineerSocketVec.push_back(new_connection);
                new_connection->registerMsgHandles(m_msgHandleMap);
                new_connection->checkConnection = boost::bind(&CCoreServer::checkConnection, shared_from_this());
                m_EngineerMutex.unlock();
            }
            else
            {
                new_connection->socket().close();
            }
            break;
        default:
            new_connection->socket().close();
            break;
        }
    }

    boost::xtime xt;
    boost::xtime_get(&xt, boost::TIME_UTC_);
    xt.sec += 5;
    boost::thread::sleep(xt);
    CCoreConnection::pointer connection = CCoreConnection::create(m_acceptor.get_io_service());

    m_acceptor.async_accept(connection->socket(),
        boost::bind(&CCoreServer::handle_accept, shared_from_this(), connection,
            boost::asio::placeholders::error));
}

void CCoreServer::checkConnection()
{
    m_SuperManagerMutex.lock();
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator superManagerItr = m_SuperManagerSocketVec.begin();
    for (superManagerItr; superManagerItr != m_SuperManagerSocketVec.end();)
    {
        if (!(*superManagerItr)->getConnectStatus())
        {
            (*superManagerItr)->closeSession();
            superManagerItr = m_SuperManagerSocketVec.erase(superManagerItr);
            ROS_WARN("killed one session, current m_SuperManagerSocketVec size:%d", (int)m_SuperManagerSocketVec.size());
        }
        else
        {
            superManagerItr++;
        }
    }
    m_SuperManagerMutex.unlock();

    m_ManagerMutex.lock();
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator managerItr = m_ManagerSocketVec.begin();
    for (managerItr; managerItr != m_ManagerSocketVec.end();)
    {
        if (!(*managerItr)->getConnectStatus())
        {
            (*managerItr)->closeSession();
            managerItr = m_ManagerSocketVec.erase(managerItr);
            ROS_WARN("killed one session, current m_ManagerSocketVec size:%d", (int)m_ManagerSocketVec.size());
        }
        else
        {
            managerItr++;
        }
    }
    m_ManagerMutex.unlock();

    m_UserMutex.lock();
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator userItr = m_UserSocketVec.begin();
    for (userItr; userItr != m_UserSocketVec.end();)
    {
        if (!(*userItr)->getConnectStatus())
        {
            (*userItr)->closeSession();
            userItr = m_UserSocketVec.erase(userItr);
            ROS_WARN("killed one session, current m_UserSocketVec size:%d", (int)m_UserSocketVec.size());
        }
        else
        {
            userItr++;
        }
    }
    m_UserMutex.unlock();

    m_EngineerMutex.lock();
    std::vector<boost::shared_ptr<CCoreConnection> >::iterator engineerItr = m_EngineerSocketVec.begin();
    for (engineerItr; engineerItr != m_EngineerSocketVec.end();)
    {
        (*engineerItr)->closeSession();
        if (!(*engineerItr)->getConnectStatus())
        {
            engineerItr = m_EngineerSocketVec.erase(engineerItr);
            ROS_WARN("killed one session, current m_EngineerSocketVec size:%d", (int)m_EngineerSocketVec.size());
        }
        else
        {
            engineerItr++;
        }
    }
    m_EngineerMutex.unlock();
}
