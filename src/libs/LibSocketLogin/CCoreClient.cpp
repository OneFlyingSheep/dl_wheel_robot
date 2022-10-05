#include "CCoreClient.h"

CCoreClient::CCoreClient(boost::asio::ip::tcp::endpoint &end, boost::asio::io_service &io_service, std::string username, std::string password) :
    m_resolver(io_service), m_socket(io_service), bLoginMode(true), m_userName(username), m_password(password)
{
    m_server = end.address().to_string();
    int32_t timeout = 30 * 1000;
    setsockopt(m_socket.native(), SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout));
    setsockopt(m_socket.native(), SOL_SOCKET, SO_SNDTIMEO, (const char*)&timeout, sizeof(timeout));
    m_msgRecvBuffer.resize(MAX_BUFF_LENGTH);
    m_msgSendBuffer.resize(MAX_BUFF_LENGTH);
    char tempPort[128];
    sprintf(tempPort, "%d", end.port());
    m_port = std::string(tempPort);
    m_clientConnect = false;
    m_needClosed = false;
    //m_handleName = handleName;
}

CCoreClient::~CCoreClient()
{
    m_needClosed = true;
    closeSocket();
}

userLoginRetVal CCoreClient::doLogin(bool bReconnect)
{
    userLoginRetVal retVal;
    if (!bLoginMode)
    {
        ROS_ERROR("error connect mode");
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = WHEEL_LOGIN_CONNECT_FAILED;
        retVal.errMsg = "error connect mode";
        return retVal;
    }
    boost::asio::ip::tcp::resolver::query query(m_server, m_port);
    boost::asio::ip::tcp::endpoint endpoint(*m_resolver.resolve(query));
    boost::system::error_code err;

    if (!bReconnect)
    {
        m_socket.connect(endpoint, err);
        if (err)
        {
            ROS_ERROR("dologin connect failed");
            retVal.role = WHEEL_USER_NONE;
            retVal.retCode = WHEEL_LOGIN_CONNECT_FAILED;
            retVal.errMsg = err.message();
            return retVal;
        }
    }

    roboKitMsg msg;
    msg.msgBody.setMsgNumber(0);
    msg.msgBody.setMsgType(0x1B58);

    msg.msgBody.jsonAppendElement("userName", m_userName.c_str());
    msg.msgBody.jsonAppendElement("password", m_password.c_str());

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

    if (err)
    {
        ROS_ERROR("dologin sendMessage failed");
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = WHEEL_LOGIN_WRITE_FAILED;
        retVal.errMsg = err.message();
        return retVal;
    }

    m_msgRecvBuffer.resize(MSG_HEADER_LENGTH, 0);

    memset(&(*m_msgRecvBuffer.begin()), 0, m_msgRecvBuffer.size());

    m_socket.read_some(boost::asio::buffer(m_msgRecvBuffer), err);

    if (err)
    {
        ROS_ERROR("dologin readMessage failed");
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = WHEEL_LOGIN_READ_FAILED;
        retVal.errMsg = err.message();
        return retVal;
    }

    boost::shared_ptr<roboKitMsg> recvMessage(new roboKitMsg);
    unsigned   char * point1 = &(*m_msgRecvBuffer.begin());

    recvMessage->msgHeader.parseHeader(point1);

    uint32_t leftLength = recvMessage->msgHeader.getLength();

    ROS_INFO("handle_readMsgHeader:Recv msgHeader Length=%u", leftLength);

//     char ss[COMMON_WR_BUFF_LENGTH_1024] = { 0 };
//     int cur = 0;
//     for (int i = 0; i < MSG_HEADER_LENGTH; i++)
//     {
//         cur += sprintf(ss + cur, "%02X ", m_msgRecvBuffer[i]);
//     }
//     ROS_INFO("recv socket msg head data:[%s]", ss);

    m_msgRecvBuffer.resize(leftLength);

    memset(&(*m_msgRecvBuffer.begin()), 0, m_msgRecvBuffer.size());

    m_socket.read_some(boost::asio::buffer(m_msgRecvBuffer), err);

    if (err)
    {
        ROS_ERROR("dologin readMessage 2 failed");
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = WHEEL_LOGIN_READ_FAILED;
        retVal.errMsg = err.message();
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

//     memset(ss, 0, COMMON_WR_BUFF_LENGTH_1024);
//     cur = 0;
//     for (int i = 0; i < (msgMaxLength < COMMON_WR_BUFF_LENGTH_1024 ? msgMaxLength : COMMON_WR_BUFF_LENGTH_1024); i++)
//         cur += sprintf(ss + cur, "%02X ", buff[i]);
//     ROS_INFO("recv socket json body data:[%s], length:%d", ss, msgMaxLength);
    //continue read
    delete[]buff;
    buff = NULL;

    Json::Value val = recvMessage->msgBody.getJsonVal();

    retVal.role = (WheelUserType)val["role"].asInt();
    retVal.retCode = (WheelLoginRetCode)val["retCode"].asInt();
    retVal.errMsg = val["errMsg"].asString();
    retVal.coreCfg.coreServerIp = QString::fromLocal8Bit(val["coreServerIp"].asString().c_str());
    retVal.coreCfg.coreServerPort = val["coreServerPort"].asInt();
    retVal.coreCfg.rcfServerPort = val["rcfServerPort"].asInt();
    retVal.coreCfg.databaseRemoteIp = QString::fromLocal8Bit(val["databaseRemoteIp"].asString().c_str());
    retVal.coreCfg.databasePort = val["databasePort"].asInt();
    retVal.coreCfg.databaseUsername = QString::fromLocal8Bit(val["databaseUsername"].asString().c_str());
    retVal.coreCfg.databasePassword = QString::fromLocal8Bit(val["databasePassword"].asString().c_str());
    retVal.coreCfg.databaseName = QString::fromLocal8Bit(val["databaseName"].asString().c_str());
    retVal.coreCfg.infraredManufacturer = val["infraredManufacturer"].asInt();

    Json::Value robotListRoot = val["robotList"];
    QStringList robotList;
    for (int i = 0; i < robotListRoot.size(); i++)
    {
        robotList.push_back(QString::fromLocal8Bit(robotListRoot[i].asString().c_str()));
    }

    retVal.coreCfg.robotList.clear();
    retVal.coreCfg.robotList = robotList;

    retVal.robotCfg.robotIp = QString::fromLocal8Bit(val["robotIp"].asString().c_str());
    retVal.robotCfg.robotName = QString::fromLocal8Bit(val["robotName"].asString().c_str());
    retVal.robotCfg.hcCtrlPort = val["hcCtrlPort"].asInt();
    retVal.robotCfg.hcRtspPort = val["hcRtspPort"].asInt();
    retVal.robotCfg.hcUserName = QString::fromLocal8Bit(val["hcUserName"].asString().c_str());
    retVal.robotCfg.hcPassword = QString::fromLocal8Bit(val["hcPassword"].asString().c_str());

    if (WHEEL_USER_NONE == retVal.role)
    {
        retVal.role = WHEEL_USER_NONE;
        retVal.retCode = (WheelLoginRetCode)val["errCode"].asInt();
        retVal.errMsg = val["errMsg"].asString();
        return retVal;
    }

    m_clientConnect = true;
    ROS_WARN(" connect success");
    m_endpoint = endpoint;
    //connect success

    if (!bReconnect)
    {
        m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
            boost::bind(&CCoreClient::handle_readMsgHeader, shared_from_this(),
                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        if (m_timer == NULL)
        {
            m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_socket.get_io_service(), boost::posix_time::seconds(5)));
        }

        m_timer->async_wait(boost::bind(&CCoreClient::sendKeepAliveMessage, shared_from_this(), boost::asio::placeholders::error));

        if (m_msgWriteCacheList.size() != 0)
        {
            sendOneMessageFromCacheList();
        }
    }

    return retVal;
}

void CCoreClient::postMsg(jsonBody message)
{
    boost::mutex::scoped_lock lock(m_sendMutex);

    if (m_needClosed == true || m_clientConnect == false)
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
//    ROS_INFO("str:%s, size:%d, type:%d", message.getJsonString().c_str(), message.getJsonString().length(), message.getMsgType());

    m_msgWriteCacheList.push_back(msg);

//    sendOneMessageFromeCacheList();
    if (needSend && m_clientConnect == true)
    {
        sendOneMessageFromCacheList();
    }
}

void CCoreClient::registerMsgHandle(int messageID, JsonMsgCallBack callback)
{
    //m_socket.get_io_service().post(boost::bind(&CSocketClient::registerMsgHandleIMPL, shared_from_this(), messageID, callback));
    m_msgHandleMap[messageID] = callback;
}

void CCoreClient::registerMsgHandleIMPL(int messageID, JsonMsgCallBack callback)
{
    //m_msgHandleMap[messageID] = callback;
}

void CCoreClient::closeSocket()
{
    ROS_INFO("socket closed");
    m_needClosed = true;
    if (m_clientConnect)
    {
        m_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    }

    m_socket.close();
}

bool CCoreClient::IsConnected()
{
    return m_clientConnect;
}

void CCoreClient::setMaxSession(int max)
{
    return ;
}

void CCoreClient::reconnect(boost::asio::ip::tcp::endpoint &end)
{
    m_server = end.address().to_string();
    char tempPort[128];
    sprintf(tempPort, "%d", end.port());
    m_port = std::string(tempPort);
    m_socket.close();
    Sleep(100);
}

void CCoreClient::generateMsgFromJsonVal(boost::shared_ptr<roboKitMsg> msg)
{
    msg->msgHeader.setNumber(msg->msgBody.getMsgNumber());
    if (msg->msgBody.getJsonVal() == NULL || msg->msgBody.getJsonVal().isNull())
    {
        msg->msgHeader.setLength(0);
    }
    else
    {
        msg->msgHeader.setLength(msg->msgBody.getJsonString().length());
    }
    msg->msgHeader.setType(msg->msgBody.getMsgType());

    m_msgSendBuffer.resize((msg->msgHeader.getLength() + MSG_HEADER_LENGTH), 0);

    memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());

    uint8_t *point = &(*m_msgSendBuffer.begin());
//    uint8_t *h = &(*m_msgSendBuffer.begin());
    memcpy(point, (void*)msg->msgHeader.convertStructToUint8(), MSG_HEADER_LENGTH);
    point += MSG_HEADER_LENGTH;

    memcpy(point, (void*)msg->msgBody.getJsonString().c_str(), msg->msgHeader.getLength());
    point += msg->msgHeader.getLength();

//    char ss[128];
//    int cur___ = 0;
//    for (int i = 0; i < MSG_HEADER_LENGTH + msg->msgHeader.getLength(); i++)
//        cur___ += sprintf(ss + cur___, "%02X ", h[i]);
//    printf("msg-convert111111:%s\n", ss);
}

void CCoreClient::handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    boost::mutex::scoped_lock lock(m_sendMutex);
    if (m_needClosed)
    {
        m_socket.close();
        m_clientConnect = false;
        return;
    }
    if (error && m_clientConnect)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", error.message().c_str());
        m_clientConnect = false;
        m_socket.close();
        m_socket.async_connect(m_endpoint, boost::bind(&CCoreClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
        return;
    }
    else
    {
        if (bytes_transferred != m_msgSendBuffer.size())
        {
            ROS_ERROR("Not all data has been write!!!need write %d ,but only write:%d", (int)m_msgSendBuffer.size(), (int)bytes_transferred);
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
//             if (m_msgWriteCacheList.front()->msgHeader.getLength() < 25)
//             {
//                 ROS_WARN("Send to CBS %s", m_msgWriteCacheList.front()->msgBody.getJsonString().c_str());
//                 m_msgWriteCacheList.erase(m_msgWriteCacheList.begin());
//             }
//             else
//             {
                //ROS_WARN("Send to CBS %s", m_msgWriteCacheList.front()->msgBody.getJsonString().c_str());
                m_msgWriteCacheList.erase(m_msgWriteCacheList.begin());
            //}
        }

        if (m_msgWriteCacheList.empty())
        {
        }
        else
        {
            sendOneMessageFromCacheList();
        }
    }
}

void CCoreClient::handle_resolve(const boost::system::error_code& err, boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
{
    ROS_INFO("CSocketClient::handle_resolve");
    if (!err)
    {
        // Attempt a connection to each endpoint in the list until we
        // successfully establish a connection.
        std::stringstream ss;
        ss << (endpoint_iterator->endpoint()) << std::endl;
        m_fullAddr = ss.str();
        ROS_INFO("%s  host resolve success,ip is %s", m_handleName.c_str(), ss.str().c_str());
        boost::asio::async_connect(m_socket, endpoint_iterator,
            boost::bind(&CCoreClient::handle_connect, shared_from_this(),
                endpoint_iterator, boost::asio::placeholders::error));
    }
    else
    {
        ROS_ERROR(" resolve failed,continue try address is %s:%s ", m_server.c_str(), m_port.c_str());
        boost::xtime xt;
        boost::xtime_get(&xt, boost::TIME_UTC_);
        xt.sec += 5;
        boost::thread::sleep(xt);
        boost::asio::ip::tcp::resolver::query query(m_server, m_port);
        m_resolver.async_resolve(query,
            boost::bind(&CCoreClient::handle_resolve, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::iterator));
    }
}

void CCoreClient::handle_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iterator, const boost::system::error_code& err)
{
    ROS_INFO("CSocketClient::handle_connect");
    boost::mutex::scoped_lock lock(m_connectMutex);
    if (!err)
    {
        m_clientConnect = true;
        ROS_WARN(" connect success");
        m_endpoint = *endpoint_iterator;
        //connect success
        m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
            boost::bind(&CCoreClient::handle_readMsgHeader, shared_from_this(),
                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        if (m_timer == NULL)
        {
            m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_socket.get_io_service(), boost::posix_time::seconds(5)));
        }

        m_timer->async_wait(boost::bind(&CCoreClient::sendKeepAliveMessage, shared_from_this(), boost::asio::placeholders::error));

        if (m_msgWriteCacheList.size() != 0)
        {
            sendOneMessageFromCacheList();
        }
    }
    else
    {
        ROS_WARN(" connect lost");
        m_clientConnect = false;

        if (m_needClosed == false)
        {
            ROS_ERROR(" %s connect to %s failed , (%s) Retry after 5 seconds", m_handleName.c_str(), m_fullAddr.c_str(), err.message().c_str());
            m_socket.close();
            boost::xtime xt;
            boost::xtime_get(&xt, boost::TIME_UTC_);
            xt.sec += 5;
            boost::thread::sleep(xt);
            boost::asio::async_connect(m_socket, endpoint_iterator,
                boost::bind(&CCoreClient::handle_connect, shared_from_this(),
                    endpoint_iterator, boost::asio::placeholders::error));
        }
    }
}

void CCoreClient::handle_reconnect(boost::asio::ip::tcp::endpoint ep, const boost::system::error_code& err)
{
    boost::mutex::scoped_lock lock(m_connectMutex);

    if (!err)
    {
        m_clientConnect = true;
        userLoginRetVal ret = doLogin(true);

        if (ret.role == WHEEL_USER_NONE)
        {
            m_clientConnect = false;

            if (m_needClosed == false)
            {
                ROS_ERROR(" %s connect to %s failed , (%s) Retry after 5 seconds", m_handleName.c_str(), m_fullAddr.c_str(), err.message().c_str());
                m_socket.close();
                boost::xtime xt;
                boost::xtime_get(&xt, boost::TIME_UTC_);
                xt.sec += 5;
                boost::thread::sleep(xt);
                m_socket.async_connect(ep, boost::bind(&CCoreClient::handle_reconnect, shared_from_this(), ep, boost::asio::placeholders::error));
            }
        }

        ROS_WARN(" reconnect success");
        //connect success
        m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
            boost::bind(&CCoreClient::handle_readMsgHeader, shared_from_this(),
                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        if (m_timer == NULL)
        {
            m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_socket.get_io_service(), boost::posix_time::seconds(10)));
        }

        m_timer->async_wait(boost::bind(&CCoreClient::sendKeepAliveMessage, shared_from_this(), boost::asio::placeholders::error));

        if (m_msgWriteCacheList.size() != 0)
        {
		    sendOneMessageFromCacheList();
        }

    }
    else
    {
        ROS_WARN(" connect lost");
        m_clientConnect = false;

        if (m_needClosed == false)
        {
            ROS_ERROR(" %s connect to %s failed , (%s) Retry after 5 seconds", m_handleName.c_str(), m_fullAddr.c_str(), err.message().c_str());
            m_socket.close();
            boost::xtime xt;
            boost::xtime_get(&xt, boost::TIME_UTC_);
            xt.sec += 5;
            boost::thread::sleep(xt);
            m_socket.async_connect(ep, boost::bind(&CCoreClient::handle_reconnect, shared_from_this(), ep, boost::asio::placeholders::error));
        }
    }
}

void CCoreClient::handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred)
{
    ROS_INFO("read");
    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        m_clientConnect = false;
        m_socket.close();
        if (!m_needClosed)
        {
            ROS_ERROR(" %s connect to %s failed , (%s) Retry after 5 seconds", m_handleName.c_str(), m_fullAddr.c_str(), err.message().c_str());
            boost::xtime xt;
            boost::xtime_get(&xt, boost::TIME_UTC_);
            xt.sec += 5;
            boost::thread::sleep(xt);
            m_socket.async_connect(m_endpoint, boost::bind(&CCoreClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
        }
        return;
    }

    if (m_needClosed == true)
    {
        ROS_WARN("%s---close sockets", m_handleName.c_str());
        m_clientConnect = false;
        m_socket.close();
        return;
    }

//    m_lastActiveTime = getTickCount();

    if (bytes_transferred != MSG_HEADER_LENGTH)
    {
        ROS_INFO("bytes_transferred != JsonMsg::headerLength Read Message Head Failed");
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
            boost::bind(&CCoreClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
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
                if (m_clientConnect)
                {
                    m_socket.close();
                    m_clientConnect = false;
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
                    boost::bind(&CCoreClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
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
                boost::bind(&CCoreClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
            return;
        }
    }

    boost::shared_ptr<roboKitMsg> recvMessage(new roboKitMsg);
    unsigned   char * point = &(*m_msgRecvBuffer.begin());

    recvMessage->msgHeader.parseHeader(point);

    uint32_t leftLength = recvMessage->msgHeader.getLength();

    ROS_INFO("handle_readMsgHeader:Recv msgHeader Length=%u", leftLength);

    char ss[COMMON_WR_BUFF_LENGTH_1024] = { 0 };
    int cur = 0;
    for (int i = 0; i < MSG_HEADER_LENGTH; i++)
    {
        cur += sprintf(ss + cur, "%02X ", m_msgRecvBuffer[i]);
    }
    ROS_INFO("recv socket msg head data:[%s]", ss);

    if (leftLength < 0 || leftLength > MAX_BUFF_LENGTH - 1)
    {
        //error
        m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
        //printf("head recv  %d \r\n %s",bytes_transferred, preetyDump(&m_msgRecvBuffer[0], bytes_transferred).c_str() );
        ROS_ERROR("recv error body length:%u", leftLength);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
            boost::bind(&CCoreClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
    }
    else
    {
        //normal
        m_msgRecvBuffer.resize(leftLength);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(leftLength),
            boost::bind(&CCoreClient::handle_readJsonBody, shared_from_this(), boost::asio::placeholders::error, recvMessage, boost::asio::placeholders::bytes_transferred)
        );
    }
}

void CCoreClient::handle_readJsonBody(const boost::system::error_code& err, boost::shared_ptr<roboKitMsg> message, std::size_t bytes_transferred)
{
    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        m_clientConnect = false;
        m_socket.close();
        m_socket.async_connect(m_endpoint, boost::bind(&CCoreClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
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

        //char ss[COMMON_WR_BUFF_LENGTH_1024] = { 0 };        
        //int cur = 0;
        //for (int i = 0; i < (msgMaxLength < COMMON_WR_BUFF_LENGTH_1024 ? msgMaxLength : COMMON_WR_BUFF_LENGTH_1024); i++)
        //    cur += sprintf(ss + cur, "%02X ", buff[i]);
        //ROS_INFO("recv socket json body data:[%s], length:%d", ss, msgMaxLength);
        //continue read
        delete[]buff;
        buff = NULL;
	}
    m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
    boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
        boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
        boost::bind(&CCoreClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    dispathRoboKitMessage(message);
}

void CCoreClient::socket_io_service_init()
{
//    boost::recursive_mutex::scoped_lock  scoped_lock(socket_mutex);
//    if (globalSocketClientInitMark == false)
//    {
//        globalSocketClientInitMark = true;
//        boost::thread * t1 = new boost::thread(boost::bind(&boost::asio::io_service::run, &socketNetClient_service));
//        boost::thread * t2 = new boost::thread(boost::bind(&boost::asio::io_service::run, &socketworkClient_service));
//    }
}

void CCoreClient::sendOneMessageFromCacheList()
{
    if (m_msgWriteCacheList.size() > 0)
    {
        boost::shared_ptr<roboKitMsg> message = m_msgWriteCacheList.front();
        generateMsgFromJsonVal(message);
        boost::asio::async_write(m_socket, boost::asio::buffer(m_msgSendBuffer),
            boost::bind(&CCoreClient::handleMsgSend, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }
}

void CCoreClient::sendKeepAliveMessage(boost::system::error_code err)
{
    jsonBody body;
    body.setMsgType(9999);

    postMsg(body);
}

void CCoreClient::dispathRoboKitMessage(boost::shared_ptr<roboKitMsg> message)
{
    m_socket.get_io_service().post(boost::bind(&CCoreClient::dispathRoboKitMessageIMPL, shared_from_this(), message));
}

void CCoreClient::dispathRoboKitMessageIMPL(boost::shared_ptr<roboKitMsg> message)
{    
    //ROS_INFO("1111recv message  %s, type :%d, id:%d", message->msgBody.getJsonString().c_str(), message->msgHeader.getType(), message->msgHeader.getNumber());

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
