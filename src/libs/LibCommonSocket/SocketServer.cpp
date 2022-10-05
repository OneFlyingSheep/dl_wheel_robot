#include "SocketServer.h"
#include <vector>
#include <iostream>
#include "LibDLHangRailRobotDBOperation/LibDLHangRailRobotDBOperation.h"
#include "LibDLHangRailConfigData/DLHangRailCoreConfigStruct.h"

socket_connection::socket_connection(boost::asio::io_service& io_service)
    : m_socket(io_service)
{
    m_needClosed = false;
    m_connect = false;
    m_userType = USER_NONE;
    m_msgRecvBuffer.resize(MAX_BUFF_LENGTH);
    m_msgSendBuffer.resize(MAX_BUFF_LENGTH);
}

socket_connection::~socket_connection()
{
    //m_socket.get_io_service().stop();
}

socket_connection::pointer socket_connection::create(boost::asio::io_service& io_service)
{
    return pointer(new socket_connection(io_service));
}

boost::asio::ip::tcp::socket& socket_connection::socket()
{
    return m_socket;
}

void socket_connection::begin()
{
    ROS_WARN(" connect success again");
    //connect success
    m_connect = true;
    m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
    boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
        boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
        boost::bind(&socket_connection::handle_readMsgHeader, shared_from_this(),
            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void socket_connection::postMsg(boost::shared_ptr<commonMsg> message)
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

    m_msgWriteCacheList.push_back(message);

    if (needSend && m_connect == true)
    {
        sendOneMessageFromeCacheList();
    }
}

void socket_connection::registerMsgHandles(std::map<int, commonMsgCallBack> callBackMap)
{
    m_msgHandleMap = callBackMap;
}

void socket_connection::closeSession()
{
    m_socket.close();
}

bool socket_connection::getConnectStatus()
{
    return m_connect;
}

hangRobotUserLoginRetVal socket_connection::getRole()
{
    hangRobotUserLoginRetVal retVal;

    boost::system::error_code err;

    m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH, 0);

    memset(&(*m_msgRecvBuffer.begin()), 0, m_msgRecvBuffer.size());

    m_socket.read_some(boost::asio::buffer(m_msgRecvBuffer), err);

    if (err)
    {
        ROS_ERROR("getRole readMessage failed");
        retVal.role = USER_NONE;
        retVal.retCode = HANG_RAIL_LOGIN_READ_FAILED;
        retVal.errMsg = err.message();
        send_Login_retVal(retVal);
        return retVal;
    }

    boost::shared_ptr<commonMsg> recvMessage(new commonMsg);
    unsigned   char * point = &(*m_msgRecvBuffer.begin());

    recvMessage->parseProtocolHeader(point);

    int leftLength = recvMessage->getMsgLeft2ReadLength();

    ROS_INFO("handle_readMsgHeader:Recv msgHeader Length=%u", leftLength);

    if (leftLength == 0 || leftLength > MAX_BUFF_LENGTH)
    {
        QString errmsg;
        errmsg.sprintf("msg length over %d readMessage failed", MAX_BUFF_LENGTH);
        ROS_ERROR(errmsg.toStdString().c_str());
        retVal.role = USER_NONE;
        retVal.retCode = HANG_RAIL_LOGIN_READ_FAILED;
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
        retVal.role = USER_NONE;
        retVal.retCode = HANG_RAIL_LOGIN_READ_FAILED;
        retVal.errMsg = err.message();
        send_Login_retVal(retVal);
        return retVal;
    }

    int msgHeaderLength = recvMessage->getProtocolHeader().msgHeaderLength + 1;
    uint8_t *buffHeader = new uint8_t[msgHeaderLength];
    memset(buffHeader, '\0', msgHeaderLength);
    memcpy(buffHeader, &(*m_msgRecvBuffer.begin()), recvMessage->getProtocolHeader().msgHeaderLength);
    recvMessage->fromUint8ToValHeader(buffHeader, msgHeaderLength);

    int msgBodyLength = recvMessage->getProtocolHeader().msgBodyLength + 1;
    uint8_t *buffBody = new uint8_t[msgBodyLength];
    memset(buffBody, '\0', msgBodyLength);
    memcpy(buffBody, &m_msgRecvBuffer.at(msgHeaderLength - 1), recvMessage->getProtocolHeader().msgBodyLength);
    recvMessage->fromUint8ToValBody(buffBody, msgBodyLength);

    delete[]buffHeader;
    delete[]buffBody;
    buffHeader = NULL;
    buffBody = NULL;

    Json::Value val = recvMessage->getBodyJsonVal();

    //judge;
    QString userName = QString::fromLocal8Bit(val["userName"].asString().c_str());
    QString password = QByteArray::fromBase64(val["password"].asString().c_str());
	ROS_ERROR("server login:%s,%s!", userName.toLocal8Bit(), password.toLocal8Bit());
    //HANG_RAIL_ROBOT_DB.getUserRole(userName, password, retVal);
    ROBOTDB_DB.getUserRole(userName, password, retVal);

    send_Login_retVal(retVal);

    return retVal;
}

void socket_connection::send_Login_retVal(hangRobotUserLoginRetVal retVal)
{
    boost::system::error_code err;
    boost::shared_ptr<commonMsg> msg(new commonMsg);

    msg->setMsgTypeId(MSG_TYPE_COMMUNICATION_LOGIN);
    msg->setMsgId(QUuid::createUuid().toString().remove('-').remove('{').remove('}').toStdString());
    m_ssid = QUuid::createUuid().toString().remove('-').remove('{').remove('}').toStdString();
    msg->setMsgSsid(m_ssid);

    msg->AppendMsgBodyElement("role", retVal.role);
    msg->AppendMsgBodyElement("retCode", retVal.retCode);
    msg->AppendMsgBodyElement("errMsg", std::string(QString::fromLocal8Bit(retVal.errMsg.c_str()).toLocal8Bit()));

    if (retVal.role != USER_NONE)
    {
        

        msg->AppendMsgBodyElement("databaseLocalhost", std::string(HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().databaseLocalhost.toLocal8Bit()));
        msg->AppendMsgBodyElement("databaseName", std::string(HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().databaseName.toLocal8Bit()));
        msg->AppendMsgBodyElement("databasePassword", std::string(HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().databasePassword.toLocal8Bit()));
        msg->AppendMsgBodyElement("databasePort", HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().databasePort);
        msg->AppendMsgBodyElement("databaseUsername", std::string(HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().databaseUsername.toLocal8Bit()));
        msg->AppendMsgBodyElement("hcnetPassword", std::string(HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().hcnetPassword.toLocal8Bit()));
        msg->AppendMsgBodyElement("hcnetPort", HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().hcnetPort);
        msg->AppendMsgBodyElement("hcnetRTSPPort", HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().hcnetRTSPPort);
        msg->AppendMsgBodyElement("hcnetUserName", std::string(HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().hcnetUserName.toLocal8Bit()));
        msg->AppendMsgBodyElement("RcfServerPort", HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().RcfServerPort);
        msg->AppendMsgBodyElement("routerIp", std::string(HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().routerIp.toLocal8Bit()));
        msg->AppendMsgBodyElement("routerIp", std::string(HANGRAIL_ROBOT_CORE_CONFIG.getCoreBackCfg().routerIp.toLocal8Bit()));
    }
    msg->generateMsg();
    m_msgSendBuffer.clear();
    m_msgSendBuffer.resize((msg->getMsgTotalLength() + MSG_HEADER_TOTAL_LENGTH), 0);

    memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());

    uint8_t *point = &(*m_msgSendBuffer.begin());

    memcpy(point, (void*)msg->getProtocolHeaderOrigin(), MSG_HEADER_TOTAL_LENGTH);
    point += MSG_HEADER_TOTAL_LENGTH;

    memcpy(point, (void*)msg->getHeaderJsonString().c_str(), msg->getProtocolHeader().msgHeaderLength);
    point += msg->getProtocolHeader().msgHeaderLength;

    memcpy(point, (void*)msg->getBodyJsonString().c_str(), msg->getProtocolHeader().msgBodyLength);

    m_socket.write_some(boost::asio::buffer(m_msgSendBuffer), err);
}

std::string socket_connection::getSsid()
{
    return m_ssid;
}

UserType socket_connection::getUserType()
{
    return m_userType;
}

void socket_connection::handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred)
{
    ROS_INFO("read");
    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        if (m_connect)
        {
            m_socket.close();
            m_connect = false;
            checkConnection();
        }
        return;
    }

    if (m_needClosed == true)
    {
        ROS_WARN("hh---close sockets");
        m_connect = false;
        m_socket.close();
        checkConnection();
        return;
    }
    //m_lastActiveTime = getTickCount();

    if (bytes_transferred != MSG_HEADER_TOTAL_LENGTH)
    {
        ROS_INFO("bytes_transferred != commonMsg::headerLength Read Message Head Failed");
        m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
            boost::bind(&socket_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
    }

    if (m_msgRecvBuffer[0] != MSG_HEADER_BEGIN_CHAR_1ST || m_msgRecvBuffer[1] != MSG_HEADER_BEGIN_CHAR_2ND
        || m_msgRecvBuffer[MSG_HEADER_TOTAL_LENGTH - 2] != MSG_HEADER_END_CHAR_1ST || m_msgRecvBuffer[MSG_HEADER_TOTAL_LENGTH - 1] != MSG_HEADER_END_CHAR_2ND)
    {
        ROS_ERROR("m_msgRecvBuffer not match");
        int count = 0;
        bool bFind = false;
        boost::system::error_code err_;
        while (count < MSG_HEADER_TOTAL_LENGTH)
        {
            if (m_msgRecvBuffer[count] == MSG_HEADER_BEGIN_CHAR_1ST)
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
            std::vector<unsigned char> tempBuffer;
            tempBuffer.resize(MSG_HEADER_TOTAL_LENGTH - count);
            m_socket.read_some(boost::asio::buffer(tempBuffer), err_);
            if (err_)
            {
                ROS_ERROR("m_msgRecvBuffer read_some err");
                ROS_ERROR("m_msgRecvBuffer sock.read_some(): An error occurred:%s", err.message().c_str());
                if (m_connect)
                {
                    m_socket.close();
                    m_connect = false;
                    checkConnection();
                }
                return;
            }

            if (tempBuffer[MSG_HEADER_TOTAL_LENGTH - count - 2] != MSG_HEADER_END_CHAR_1ST
                || tempBuffer[MSG_HEADER_TOTAL_LENGTH - count - 1] != MSG_HEADER_END_CHAR_2ND)
            {
                ROS_ERROR("m_msgRecvBuffer not match 2");
                //error
                m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
                //printf("head recv  %d \r\n %s",bytes_transferred, preetyDump(&m_msgRecvBuffer[0], bytes_transferred).c_str() );
                boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
                    boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
                    boost::bind(&socket_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
                return;
            }
            else
            {
                std::vector<unsigned char> totalBuff;
                totalBuff.resize(MSG_HEADER_TOTAL_LENGTH);
                for (int i = 0; i < MSG_HEADER_TOTAL_LENGTH; i++)
                {
                    if (i < MSG_HEADER_TOTAL_LENGTH - count)
                    {
                        totalBuff.push_back(m_msgRecvBuffer[count + i]);
                    }
                    else
                    {
                        totalBuff.push_back(tempBuffer[i - (MSG_HEADER_TOTAL_LENGTH - count)]);
                    }
                }
                m_msgRecvBuffer.clear();
                m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
                m_msgRecvBuffer = totalBuff;
            }
        }
        else
        {
            ROS_ERROR("m_msgRecvBuffer not find!!!");
            //error
            m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
            //printf("head recv  %d \r\n %s",bytes_transferred, preetyDump(&m_msgRecvBuffer[0], bytes_transferred).c_str() );
            boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
                boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
                boost::bind(&socket_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            return;
        }
    }

    boost::shared_ptr<commonMsg> recvMessage(new commonMsg);
    unsigned   char * point = &(*m_msgRecvBuffer.begin());

    recvMessage->parseProtocolHeader(point);

    int leftLength = recvMessage->getMsgLeft2ReadLength();

    ROS_INFO("handle_readMsgHeader:Recv msg Total Length=%u", leftLength);

    //     try
    //     {
    //         char ss[COMMON_WR_BUFF_LENGTH_1024] = { 0 };
    //         int cur = 0;
    //         for (int i = 0; i < MSG_HEADER_TOTAL_LENGTH; i++)
    //         {
    //             cur += sprintf(ss + cur, "%02X ", m_msgRecvBuffer[i]);
    //         }
    //         ROS_INFO("recv socket msg head data:[%s]", ss);
    //     }
    //     catch (...)
    //     {
    //         ROS_ERROR("exception!!");
    //     }


    try
    {
        if (leftLength < 0 || leftLength > MAX_BUFF_LENGTH - 1)
        {
            //error
            m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
            //printf("head recv  %d \r\n %s",bytes_transferred, preetyDump(&m_msgRecvBuffer[0], bytes_transferred).c_str() );
            ROS_ERROR("recv error body length:%u", leftLength);
            boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
                boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
                boost::bind(&socket_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
            );
        }
        else
        {
            //normal
            m_msgRecvBuffer.resize(leftLength);
            boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
                boost::asio::transfer_exactly(leftLength),
                boost::bind(&socket_connection::handle_readCommonMsgBody, shared_from_this(), boost::asio::placeholders::error, recvMessage, boost::asio::placeholders::bytes_transferred)
            );
        }
    }
    catch (...)
    {
        ROS_ERROR("exception!!length:%d", leftLength);
    }
}

void socket_connection::handle_readCommonMsgBody(const boost::system::error_code& err, boost::shared_ptr<commonMsg> message, std::size_t bytes_transferred)
{
    ROS_INFO("tcp_connection::handle_readJsonBody");
    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        m_socket.close();
        //m_socket.async_connect(m_endpoint, boost::bind(&CSocket::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
        return;
    }

    if (message->getMsgLeft2ReadLength() > 0)
    {
        int msgHeaderLength = message->getProtocolHeader().msgHeaderLength + 1;
        uint8_t *buffHeader = new uint8_t[msgHeaderLength];
        memset(buffHeader, '\0', msgHeaderLength);
        memcpy(buffHeader, &(*m_msgRecvBuffer.begin()), message->getProtocolHeader().msgHeaderLength);
        message->fromUint8ToValHeader(buffHeader, msgHeaderLength);

        int msgBodyLength = message->getProtocolHeader().msgBodyLength + 1;
        uint8_t *buffBody = new uint8_t[msgBodyLength];
        memset(buffBody, '\0', msgBodyLength);
        memcpy(buffBody, &m_msgRecvBuffer.at(msgHeaderLength - 1), message->getProtocolHeader().msgBodyLength);
        message->fromUint8ToValBody(buffBody, msgBodyLength);

        //char ss[COMMON_WR_BUFF_LENGTH_1024] = { 0 };        
        //int cur = 0;
        //for (int i = 0; i < (msgMaxLength < COMMON_WR_BUFF_LENGTH_1024 ? msgMaxLength : COMMON_WR_BUFF_LENGTH_1024); i++)
        //    cur += sprintf(ss + cur, "%02X ", buff[i]);
        //ROS_INFO("recv socket json body data:[%s], length:%d", ss, msgMaxLength);
        //continue read
        delete[]buffHeader;
        delete[]buffBody;
        buffHeader = NULL;
        buffBody = NULL;
    }
    m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
    boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
        boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
        boost::bind(&socket_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    dispathRoboKitMessage(message);
}

void socket_connection::generateMsgFromCommonVal(boost::shared_ptr<commonMsg> msg)
{
    msg->setMsgId(QUuid::createUuid().toString().remove('-').remove('{').remove('}').toStdString());
    msg->setMsgSsid(m_ssid);
    msg->generateMsg();

    m_msgSendBuffer.resize((msg->getMsgTotalLength() + MSG_HEADER_TOTAL_LENGTH), 0);

    memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());

    uint8_t *point = &(*m_msgSendBuffer.begin());
    memcpy(point, (void*)msg->getProtocolHeaderOrigin(), MSG_HEADER_TOTAL_LENGTH);
    point += MSG_HEADER_TOTAL_LENGTH;

    memcpy(point, (void*)msg->getHeaderJsonString().c_str(), msg->getProtocolHeader().msgHeaderLength);
    point += msg->getProtocolHeader().msgHeaderLength;

    memcpy(point, (void*)msg->getBodyJsonString().c_str(), msg->getProtocolHeader().msgBodyLength);
}

void socket_connection::handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    ROS_INFO("handleMsgSend");
    boost::mutex::scoped_lock lock(m_sendMutex);
    if (error)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", error.message().c_str());
        m_connect = false;
        m_socket.close();
        checkConnection();
        return;
    }
    else
    {
        if (bytes_transferred != m_msgSendBuffer.size())
        {
            ROS_ERROR("Not all data has been right!!!need write %d ,but only write:%d", (int)m_msgSendBuffer.size(), (int)bytes_transferred);
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

void socket_connection::sendOneMessageFromeCacheList()
{
    ROS_INFO("sendOneMessageFromeCacheList : %d", m_msgWriteCacheList.size());
    if (m_msgWriteCacheList.size() > 0)
    {
        boost::shared_ptr<commonMsg> message = m_msgWriteCacheList.front();
        generateMsgFromCommonVal(message);
        ROS_INFO("sendOneMessageFromeCacheList message: %d", m_msgWriteCacheList.size());
        boost::asio::async_write(m_socket, boost::asio::buffer(m_msgSendBuffer),
            boost::bind(&socket_connection::handleMsgSend, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }
}

void socket_connection::dispathRoboKitMessage(boost::shared_ptr<commonMsg> message)
{
    ROS_INFO("dispathRoboKitMessage");
    m_socket.get_io_service().post(boost::bind(&socket_connection::dispathRoboKitMessageIMPL, shared_from_this(), message));
}

void socket_connection::dispathRoboKitMessageIMPL(boost::shared_ptr<commonMsg> message)
{
//    jsonBody tmpBody;
//         currentbaseMessage = message;
//         printf("recv message  %s\r\n", currentbaseMessage->msgBody.getJsonString().c_str());
//     
//         try
//         {
//             /*tmpBody = */m_msgHandleMap[currentbaseMessage->msgHeader.getType()](currentbaseMessage);
//            postMsg(tmpBody);
//         }
//         catch (...)
//         {
//             ROS_ERROR("dispathRoboKitMessageIMPL err, maybe haven't register function handle, Connection closed");
//             m_connect = false;
//             m_socket.close();
//         }
//     
//         currentbaseMessage = boost::shared_ptr<commonMsg>(new commonMsg);

    ROS_INFO("dispathRoboKitMessageIMPL server recv message,  ssid:%s, msgId:%s, typeId:%d",
        message->getMsgHeader().ssid.c_str(), message->getMsgHeader().msgid.c_str(), message->getMsgHeader().msgtypeid);

    int msgType = message->getMsgHeader().msgtypeid;

    if (m_msgHandleMap.find(msgType) != m_msgHandleMap.end())
    {
        try
        {
            m_msgHandleMap[msgType](message);
        }
        catch (...)
        {
            ROS_ERROR("dispathRoboKitMessageIMPL [exception], maybe haven't register function handle.");
        }
    }
    else
    {
        ROS_ERROR("No msgHandleMap. type:%d", msgType);
    }
}

SocketServer::SocketServer(boost::asio::ip::tcp::endpoint &endpoint, boost::asio::io_service &io_service) :
    m_acceptor(io_service, endpoint)
{
    ROS_INFO("CSocketServer::CSocketServer");
    m_maxSessionAllowed = 100;
    m_endpoint = endpoint;
    m_socketVec.clear();
    m_timer = NULL;
}

SocketServer::~SocketServer()
{
    ROS_INFO("Server closed");
    closeSocket();
}

boost::shared_ptr<socket_connection> SocketServer::getConnectionBySsid(std::string ssid)
{
    boost::mutex::scoped_lock lock(m_socketVecLock);
    std::vector<boost::shared_ptr<socket_connection> >::iterator itr = m_socketVec.begin();
    for (itr; itr != m_socketVec.end();)
    {
        if ((*itr)->getSsid() == ssid)
        {
            return *itr;
        }
    }

    return NULL;
}

void SocketServer::initSocket()
{
    ROS_INFO("CSocketServer::initSocket");
    socket_connection::pointer new_connection = socket_connection::create(m_acceptor.get_io_service());

    m_acceptor.async_accept(new_connection->socket(),
        boost::bind(&SocketServer::handle_accept, shared_from_this(), new_connection,
            boost::asio::placeholders::error));

    if (m_timer == NULL)
    {
        m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_acceptor.get_io_service(), boost::posix_time::seconds(10)));
    }

    m_timer->async_wait(boost::bind(&SocketServer::connectAliveCallback, shared_from_this(), boost::asio::placeholders::error));

}

void SocketServer::postMsg(boost::shared_ptr<commonMsg> message)
{
    boost::mutex::scoped_lock lock(m_socketVecLock);
    if (m_socketVec.size() <= 0)
    {
        ROS_INFO("no connection");
        return ;
    }
    std::vector<boost::shared_ptr<socket_connection> >::iterator itr = m_socketVec.begin();
    for (itr; itr != m_socketVec.end(); itr++)
    {
        (*itr)->postMsg(message);
    }
}

void SocketServer::postMsgByUserType(boost::shared_ptr<commonMsg> message, UserType type)
{
    boost::mutex::scoped_lock lock(m_socketVecLock);
    if (m_socketVec.size() <= 0)
    {
        ROS_INFO("no connection");
        return;
    }
    std::vector<boost::shared_ptr<socket_connection> >::iterator itr = m_socketVec.begin();
    for (itr; itr != m_socketVec.end(); itr++)
    {
        if ((*itr)->getUserType() == type)
        {
            (*itr)->postMsg(message);
        }
    }
}

void SocketServer::postMsgBySsid(boost::shared_ptr<commonMsg> message, std::string ssid)
{
    boost::mutex::scoped_lock lock(m_socketVecLock);
    if (m_socketVec.size() <= 0)
    {
        ROS_INFO("no connection");
        return;
    }
    std::vector<boost::shared_ptr<socket_connection> >::iterator itr = m_socketVec.begin();
    for (itr; itr != m_socketVec.end(); itr++)
    {
        if ((*itr)->getSsid() == ssid)
        {
            (*itr)->postMsg(message);
        }
    }
}

void SocketServer::registerMsgHandle(int messageID, commonMsgCallBack callback)
{
    m_acceptor.get_io_service().post(boost::bind(&SocketServer::registerMsgHandleIMPL, shared_from_this(), messageID, callback));
}

void SocketServer::registerMsgHandleIMPL(int messageID, commonMsgCallBack callback)
{
    m_msgHandleMap[messageID] = callback;
}

void SocketServer::closeSocket()
{
    boost::mutex::scoped_lock lock(m_socketVecLock);
    std::vector<boost::shared_ptr<socket_connection> >::iterator itr = m_socketVec.begin();
    for (itr; itr != m_socketVec.end(); itr++)
    {
        (*itr)->closeSession();
    }
}

void SocketServer::setMaxSession(int max)
{
    m_maxSessionAllowed = max;
}

void SocketServer::connectAliveCallback(boost::system::error_code err)
{
    boost::mutex::scoped_lock lock(m_socketVecLock);
    if (m_socketVec.size() > 0)
    {
        std::vector<boost::shared_ptr<socket_connection> >::iterator itr = m_socketVec.begin();
        for (itr; itr != m_socketVec.end();)
        {
            if (!(*itr)->getConnectStatus())
            {
                (*itr)->closeSession();
                itr = m_socketVec.erase(itr);
                ROS_INFO("killed one session, current size:%d", (int)m_socketVec.size());
            }
            else
            {
                 itr++;
            }
        }
    }

    ROS_INFO("connectAliveCallback");

    if (m_socketVec.size() > 0 )
    {
        if (m_timer == NULL)
        {
            m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_acceptor.get_io_service(), boost::posix_time::seconds(10)));
        }
        m_timer->expires_from_now(boost::posix_time::seconds(10));
        m_timer->async_wait(boost::bind(&SocketServer::connectAliveCallback, shared_from_this(), boost::asio::placeholders::error));
    }

}

boost::asio::ip::tcp::endpoint SocketServer::getEndpoint()
{
    return m_endpoint;
}

void SocketServer::checkConnection()
{
    boost::mutex::scoped_lock lock(m_socketVecLock);
    std::vector<boost::shared_ptr<socket_connection> >::iterator vectorItr = m_socketVec.begin();
    for (vectorItr; vectorItr != m_socketVec.end();)
    {
        if (!(*vectorItr)->getConnectStatus())
        {
            (*vectorItr)->closeSession();
            vectorItr = m_socketVec.erase(vectorItr);
            ROS_WARN("killed one session, current m_socketVec size:%d", (int)m_socketVec.size());
        }
        else
        {
            vectorItr++;
        }
    }
}

void SocketServer::handle_accept(socket_connection::pointer new_connection, const boost::system::error_code& error)
{
    ROS_INFO("handle_accept");
    ROS_WARN("max:%d, size:%d, err:%d", m_maxSessionAllowed, (int)m_socketVec.size(), error.value());
    if (error)
    {  
        ROS_WARN(" accept failed");
        ROS_ERROR(" accept failed,continue try address is %s:%d ", m_endpoint.address().to_string().c_str(), m_endpoint.port());
    }
    else if (m_socketVec.size() >= m_maxSessionAllowed)
    {
        ROS_WARN(" accept failed, max session reached");
		new_connection->closeSession();
    }
    else
    {
        hangRobotUserLoginRetVal ret = new_connection->getRole();

        if (ret.role != USER_NONE)
        {
            m_socketVecLock.lock();
            new_connection->begin();
            new_connection->registerMsgHandles(m_msgHandleMap);
            m_socketVec.push_back(new_connection);
            new_connection->checkConnection = boost::bind(&SocketServer::checkConnection, shared_from_this());
            m_socketVecLock.unlock();
        }
        else
        {
            new_connection->closeSession();
        }        

//         if (m_timer == NULL)
//         {
//             m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_acceptor.get_io_service(), boost::posix_time::seconds(10)));
//             m_timer->async_wait(boost::bind(&SocketServer::connectAliveCallback, shared_from_this(), boost::asio::placeholders::error));
//         }
    }

    boost::xtime xt;
    boost::xtime_get(&xt, boost::TIME_UTC_);
    xt.sec += 5;
    boost::thread::sleep(xt);
    socket_connection::pointer connection = socket_connection::create(m_acceptor.get_io_service());

    m_acceptor.async_accept(connection->socket(),
        boost::bind(&SocketServer::handle_accept, shared_from_this(), connection,
            boost::asio::placeholders::error));
}
