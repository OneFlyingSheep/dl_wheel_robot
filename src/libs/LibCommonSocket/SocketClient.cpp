#include "SocketClient.h"

SocketClient::SocketClient(boost::asio::ip::tcp::endpoint &end, boost::asio::io_service &io_service) :
    m_resolver(io_service), m_socket(io_service), bLoginMode(true)
{
//    socket_io_service_init();
    m_server = end.address().to_string();
    m_msgRecvBuffer.resize(MAX_BUFF_LENGTH);
    m_msgSendBuffer.resize(MAX_BUFF_LENGTH);
    char tempPort[128];
    sprintf(tempPort, "%d", end.port());
    m_port = std::string(tempPort);
    m_clientConnect = false;
    m_needClosed = false;
    m_ssid.clear();
    //m_ssid = "niubi";
    //m_handleName = handleName;
    //currentbaseMessage = boost::shared_ptr<commonMsg>(new commonMsg);
}

SocketClient::~SocketClient()
{
    m_needClosed = true;
    //closeSocket();
}

hangRobotUserLoginRetVal SocketClient::doLogin(std::string username, std::string password, bool bReconnect)
{
	ROS_ERROR("SocketClient:start doLogin!");
    hangRobotUserLoginRetVal retVal;
    m_userName = username;
    m_password = password;

    if (!bLoginMode)
    {
        ROS_ERROR("error connect mode");
        retVal.role = USER_NONE;
        retVal.retCode = HANG_RAIL_LOGIN_CONNECT_FAILED;
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
            retVal.role = USER_NONE;
            retVal.retCode = HANG_RAIL_LOGIN_CONNECT_FAILED;
            retVal.errMsg = err.message();
            return retVal;
        }
    }

    //boost::shared_ptr<commonMsg> msg(new commonMsg);

    //msg->setMsgTypeId(MSG_TYPE_COMMUNICATION_LOGIN);
    //msg->setMsgId(QUuid::createUuid().toString().remove('-').remove('{').remove('}').toStdString());
    //msg->setMsgSsid(m_ssid);

    //msg->AppendMsgBodyElement("userName", m_userName.c_str());
    //msg->AppendMsgBodyElement("password", m_password.c_str());
    //   
    //msg->generateMsg();
    //m_msgSendBuffer.clear();
    //m_msgSendBuffer.resize((msg->getMsgTotalLength() + MSG_HEADER_TOTAL_LENGTH), 0);

    //memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());

    //uint8_t *point = &(*m_msgSendBuffer.begin());

    //memcpy(point, (void*)msg->getProtocolHeaderOrigin(), MSG_HEADER_TOTAL_LENGTH);
    //point += MSG_HEADER_TOTAL_LENGTH;

    //memcpy(point, (void*)msg->getHeaderJsonString().c_str(), msg->getProtocolHeader().msgHeaderLength);
    //point += msg->getProtocolHeader().msgHeaderLength;

    //memcpy(point, (void*)msg->getBodyJsonString().c_str(), msg->getProtocolHeader().msgBodyLength);

    //m_socket.write_some(boost::asio::buffer(m_msgSendBuffer), err);

    //if (err)
    //{
    //    ROS_ERROR("dologin sendMessage failed");
    //    retVal.role = USER_NONE;
    //    retVal.retCode = HANG_RAIL_LOGIN_WRITE_FAILED;
    //    retVal.errMsg = err.message();
    //    return retVal;
    //}

    //m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH, 0);

    //memset(&(*m_msgRecvBuffer.begin()), 0, m_msgRecvBuffer.size());

    //m_socket.read_some(boost::asio::buffer(m_msgRecvBuffer), err);

    //if (err)
    //{
    //    ROS_ERROR("dologin readMessage failed");
    //    retVal.role = USER_NONE;
    //    retVal.retCode = HANG_RAIL_LOGIN_READ_FAILED;
    //    retVal.errMsg = err.message();
    //    return retVal;
    //}

    //boost::shared_ptr<commonMsg> recvMessage(new commonMsg);
    //point = &(*m_msgRecvBuffer.begin());

    //recvMessage->parseProtocolHeader(point);

    //int leftLength = recvMessage->getMsgLeft2ReadLength();

    //ROS_INFO("handle_readMsgHeader:Recv msg Total Length=%u", leftLength);

    ////     char ss[COMMON_WR_BUFF_LENGTH_1024] = { 0 };
    ////     int cur = 0;
    ////     for (int i = 0; i < MSG_HEADER_LENGTH; i++)
    ////     {
    ////         cur += sprintf(ss + cur, "%02X ", m_msgRecvBuffer[i]);
    ////     }
    ////     ROS_INFO("recv socket msg head data:[%s]", ss);

    //m_msgRecvBuffer.resize(leftLength);

    //memset(&(*m_msgRecvBuffer.begin()), 0, m_msgRecvBuffer.size());

    //m_socket.read_some(boost::asio::buffer(m_msgRecvBuffer), err);

    //if (err)
    //{
    //    ROS_ERROR("dologin readMessage 2 failed");
    //    retVal.role = USER_NONE;
    //    retVal.retCode = HANG_RAIL_LOGIN_READ_FAILED;
    //    retVal.errMsg = err.message();
    //    return retVal;
    //}

    //int msgHeaderLength = recvMessage->getProtocolHeader().msgHeaderLength + 1;
    //uint8_t *buffHeader = new uint8_t[msgHeaderLength];
    //memset(buffHeader, '\0', msgHeaderLength);
    //memcpy(buffHeader, &(*m_msgRecvBuffer.begin()), recvMessage->getProtocolHeader().msgHeaderLength);
    //recvMessage->fromUint8ToValHeader(buffHeader, msgHeaderLength);

    //int msgBodyLength = recvMessage->getProtocolHeader().msgBodyLength + 1;
    //uint8_t *buffBody = new uint8_t[msgBodyLength];
    //memset(buffBody, '\0', msgBodyLength);
    //memcpy(buffBody, &m_msgRecvBuffer.at(msgHeaderLength - 1), recvMessage->getProtocolHeader().msgBodyLength);
    //recvMessage->fromUint8ToValBody(buffBody, msgBodyLength);

    ////     memset(ss, 0, COMMON_WR_BUFF_LENGTH_1024);
    ////     cur = 0;
    ////     for (int i = 0; i < (msgMaxLength < COMMON_WR_BUFF_LENGTH_1024 ? msgMaxLength : COMMON_WR_BUFF_LENGTH_1024); i++)
    ////         cur += sprintf(ss + cur, "%02X ", buff[i]);
    ////     ROS_INFO("recv socket json body data:[%s], length:%d", ss, msgMaxLength);
    //    //continue read
    //delete[]buffHeader;
    //delete[]buffBody;
    //buffHeader = NULL;
    //buffBody = NULL;

    //Json::Value val = recvMessage->getBodyJsonVal();

    //retVal.role = (UserType)val["role"].asInt();
    //retVal.retCode = (HangRailLoginRetCode)val["retCode"].asInt();
    //retVal.errMsg = val["errMsg"].asString();

    //if (USER_NONE == retVal.role)
    //{
    //    return retVal;
    //}

    //m_ssid = recvMessage->getMsgHeader().ssid;

    //retVal.coreCfg.databaseLocalhost = QString::fromStdString(val["databaseLocalhost"].asString());
    //retVal.coreCfg.databaseName = QString::fromStdString(val["databaseName"].asString());
    //retVal.coreCfg.databasePassword = QString::fromStdString(val["databasePassword"].asString());
    //retVal.coreCfg.databasePort = val["databasePort"].asInt();
    //retVal.coreCfg.databaseUsername = QString::fromStdString(val["databaseUsername"].asString());
    //retVal.coreCfg.hcnetPassword = QString::fromStdString(val["hcnetPassword"].asString());
    //retVal.coreCfg.hcnetPort = val["hcnetPort"].asInt();
    //retVal.coreCfg.hcnetRTSPPort = val["hcnetRTSPPort"].asInt();
    //retVal.coreCfg.hcnetUserName = QString::fromStdString(val["hcnetUserName"].asString());
    //retVal.coreCfg.RcfServerPort = val["RcfServerPort"].asInt();
    //retVal.coreCfg.routerIp = QString::fromStdString(val["routerIp"].asString());

    //m_clientConnect = true;
    //ROS_WARN(" connect success");
    //m_endpoint = endpoint;
    //connect success

    if (!bReconnect)
    {
        m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
            boost::bind(&SocketClient::handle_readMsgHeader, shared_from_this(),
                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        if (m_timer == NULL)
        {
            m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_socket.get_io_service(), boost::posix_time::seconds(5)));
        }

        m_timer->async_wait(boost::bind(&SocketClient::sendKeepAliveMessage, shared_from_this(), boost::asio::placeholders::error));

        if (m_msgWriteCacheList.size() != 0)
        {
            sendOneMessageFromCacheList();
        }
    }

    return retVal;
}

void SocketClient::initSocket()
{
    boost::asio::ip::tcp::resolver::query query(m_server, m_port);
    m_resolver.async_resolve(query,
        boost::bind(&SocketClient::handle_resolve, this,
            boost::asio::placeholders::error,
            boost::asio::placeholders::iterator));
}

void SocketClient::postMsg(boost::shared_ptr<commonMsg> message)
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

    m_msgWriteCacheList.push_back(message);

//    sendOneMessageFromeCacheList();
    if (needSend && m_clientConnect == true)
    {
        sendOneMessageFromCacheList();
    }
}

void SocketClient::postClientMsg(boost::shared_ptr<commonMsg> message)
{
//     boost::mutex::scoped_lock lock(m_sendMutex);
// 
//     m_msgSendBuffer.resize((message->msgHeader.getLength() + MSG_HEADER_TOTAL_LENGTH), 0);
// 
//     memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());
// 
//     uint8_t *point = &(*m_msgSendBuffer.begin());
//     //    uint8_t *h = &(*m_msgSendBuffer.begin());
//     memcpy(point, (void*)message->msgHeader.convertStructToUint8(), MSG_HEADER_TOTAL_LENGTH);
//     point += MSG_HEADER_TOTAL_LENGTH;
// 
//     memcpy(point, (void*)message->msgBody.getJsonString().c_str(), message->msgHeader.getLength());
//     point += message->msgHeader.getLength();
// 
//     boost::asio::async_write(m_socket, boost::asio::buffer(m_msgSendBuffer),
//         boost::bind(&SocketClient::handleMsgSend, shared_from_this(),
//             boost::asio::placeholders::error,
//             boost::asio::placeholders::bytes_transferred));
}

void SocketClient::registerMsgHandle(int messageID, commonMsgCallBack callback)
{
    //m_socket.get_io_service().post(boost::bind(&SocketClient::registerMsgHandleIMPL, shared_from_this(), messageID, callback));
    m_msgHandleMap[messageID] = callback;
}

void SocketClient::closeSocket()
{
    ROS_INFO("socket closed");
    m_needClosed = true;

    if (m_clientConnect)
    {
        m_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    }

    m_socket.close();
}

bool SocketClient::IsConnected()
{
    return m_clientConnect;
}

void SocketClient::setMaxSession(int max)
{
    return ;
}

void SocketClient::reconnect(boost::asio::ip::tcp::endpoint &end)
{
    m_server = end.address().to_string();
    char tempPort[128];
    sprintf(tempPort, "%d", end.port());
    m_port = std::string(tempPort);
    m_socket.close();
    Sleep(100);
    initSocket();
}

void SocketClient::generateMsgFromJsonVal(boost::shared_ptr<commonMsg> msg)
{
//     msg->msgHeader.setNumber(msg->msgBody.getMsgNumber());
//     if (msg->msgBody.getJsonVal() == NULL || msg->msgBody.getJsonVal().isNull())
//     {
//         msg->msgHeader.setLength(0);
//     }
//     else
//     {
//         msg->msgHeader.setLength(msg->msgBody.getJsonString().length());
//     }
//     msg->msgHeader.setType(msg->msgBody.getMsgType());

    msg->setMsgId(QUuid::createUuid().toString().remove('-').remove('{').remove('}').toStdString());
    msg->setMsgSsid(m_ssid);
    msg->generateMsg();

    m_msgSendBuffer.resize((msg->getMsgTotalLength() + MSG_HEADER_TOTAL_LENGTH), 0);

    memset(&(*m_msgSendBuffer.begin()), 0, m_msgSendBuffer.size());

    uint8_t *point = &(*m_msgSendBuffer.begin());
    uint8_t *h = &(*m_msgSendBuffer.begin());
    memcpy(point, (void*)msg->getProtocolHeaderOrigin(), MSG_HEADER_TOTAL_LENGTH);
    point += MSG_HEADER_TOTAL_LENGTH;

    memcpy(point, (void*)msg->getHeaderJsonString().c_str(), msg->getProtocolHeader().msgHeaderLength);
    point += msg->getProtocolHeader().msgHeaderLength;

    memcpy(point, (void*)msg->getBodyJsonString().c_str(), msg->getProtocolHeader().msgBodyLength);

//    char ss[65535];
//    int cur___ = 0;
//    for (int i = 0; i < msg->getMsgTotalLength() + MSG_HEADER_TOTAL_LENGTH; i++)
//        cur___ += sprintf(ss + cur___, "%02X ", h[i]);
//    ROS_WARN("msg-convert111111:%s\n", ss);
}

void SocketClient::handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred)
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
        m_socket.async_connect(m_endpoint, boost::bind(&SocketClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
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
//                 ROS_INFO("Send to CBS %s", m_msgWriteCacheList.front()->msgBody.getJsonString().c_str());
//                 m_msgWriteCacheList.erase(m_msgWriteCacheList.begin());
//             }
//             else
//             {
                //ROS_INFO("Send to CBS type:%d, jsonString: %s", m_msgWriteCacheList.front()->msgBody.getMsgType(), m_msgWriteCacheList.front()->msgBody.getJsonString().c_str());
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

void SocketClient::handle_resolve(const boost::system::error_code& err, boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
{
    ROS_INFO("SocketClient::handle_resolve");
    if (!err)
    {
        // Attempt a connection to each endpoint in the list until we
        // successfully establish a connection.
        std::stringstream ss;
        ss << (endpoint_iterator->endpoint()) << std::endl;
        m_fullAddr = ss.str();
        ROS_INFO("%s  host resolve success,ip is %s", m_handleName.c_str(), ss.str().c_str());
        boost::asio::async_connect(m_socket, endpoint_iterator,
            boost::bind(&SocketClient::handle_connect, shared_from_this(),
                endpoint_iterator, boost::asio::placeholders::error));
    }
    else
    {
        ROS_ERROR(" resolve failed,continue try address is %s:%s ", m_server.c_str(), m_port.c_str());
        boost::xtime xt;
        boost::xtime_get(&xt, boost::TIME_UTC_);
        xt.sec += 3;
        boost::thread::sleep(xt);
        boost::asio::ip::tcp::resolver::query query(m_server, m_port);
        m_resolver.async_resolve(query,
            boost::bind(&SocketClient::handle_resolve, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::iterator));
    }
}

void SocketClient::handle_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iterator, const boost::system::error_code& err)
{
    ROS_INFO("SocketClient::handle_connect");
    boost::mutex::scoped_lock lock(m_connectMutex);
    if (!err)
    {
        m_clientConnect = true;
        ROS_WARN(" connect success");
        m_endpoint = *endpoint_iterator;
        //connect success
        m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
            boost::bind(&SocketClient::handle_readMsgHeader, shared_from_this(),
                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        if (m_timer == NULL)
        {
            m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_socket.get_io_service(), boost::posix_time::seconds(5)));
        }

        m_timer->async_wait(boost::bind(&SocketClient::sendKeepAliveMessage, shared_from_this(), boost::asio::placeholders::error));

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
                boost::bind(&SocketClient::handle_connect, shared_from_this(),
                    endpoint_iterator, boost::asio::placeholders::error));
        }
    }
}

void SocketClient::handle_reconnect(boost::asio::ip::tcp::endpoint ep, const boost::system::error_code& err)
{
    boost::mutex::scoped_lock lock(m_connectMutex);

    if (!err)
    {
        m_clientConnect = true;
        hangRobotUserLoginRetVal ret = doLogin(m_userName, m_password, true);

        if (ret.role == USER_NONE)
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
                m_socket.async_connect(ep, boost::bind(&SocketClient::handle_reconnect, shared_from_this(), ep, boost::asio::placeholders::error));
            }
        }

        ROS_WARN(" connect success again");
        //connect success
        m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
            boost::bind(&SocketClient::handle_readMsgHeader, shared_from_this(),
                boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        if (m_timer == NULL)
        {
            m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_socket.get_io_service(), boost::posix_time::seconds(10)));
        }

        m_timer->async_wait(boost::bind(&SocketClient::sendKeepAliveMessage, shared_from_this(), boost::asio::placeholders::error));

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
            m_socket.close();
            ROS_ERROR(" %s connect to %s failed , (%s) Retry after 5 seconds", m_handleName.c_str(), m_fullAddr.c_str(), err.message().c_str());
            boost::xtime xt;
            boost::xtime_get(&xt, boost::TIME_UTC_);
            xt.sec += 5;
            boost::thread::sleep(xt);            
            m_socket.async_connect(ep, boost::bind(&SocketClient::handle_reconnect, shared_from_this(), ep, boost::asio::placeholders::error));
        }
    }
}

void SocketClient::handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred)
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
            m_socket.async_connect(m_endpoint, boost::bind(&SocketClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
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

    if (bytes_transferred != MSG_HEADER_TOTAL_LENGTH)
    {
        ROS_INFO("bytes_transferred != commonMsg::headerLength Read Message Head Failed");
        m_msgRecvBuffer.resize(MSG_HEADER_TOTAL_LENGTH);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(MSG_HEADER_TOTAL_LENGTH),
            boost::bind(&SocketClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
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
        if (bFind && (count < 11))
        {
            ROS_ERROR("m_msgRecvBuffer find!!");
            std::vector<unsigned char> tempBuffer;
            tempBuffer.resize(MSG_HEADER_TOTAL_LENGTH - count);
            m_socket.read_some(boost::asio::buffer(tempBuffer), err_);
            if (err_)
            {
                ROS_ERROR("m_msgRecvBuffer read_some err");
                ROS_ERROR("m_msgRecvBuffer sock.read_some(): An error occurred:%s", err.message().c_str());
                if (m_clientConnect)
                {
                    m_socket.close();
                    m_clientConnect = false;
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
                    boost::bind(&SocketClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
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
                boost::bind(&SocketClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
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
                boost::bind(&SocketClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
            );
        }
        else
        {
            //normal
            m_msgRecvBuffer.resize(leftLength);
            boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
                boost::asio::transfer_exactly(leftLength),
                boost::bind(&SocketClient::handle_readCommonMsg, shared_from_this(), boost::asio::placeholders::error, recvMessage, boost::asio::placeholders::bytes_transferred)
            );
        }
    }
    catch (...)
    {
        ROS_ERROR("exception!!length:%d", leftLength);
    }
}

void SocketClient::handle_readCommonMsg(const boost::system::error_code& err, boost::shared_ptr<commonMsg> message, std::size_t bytes_transferred)
{
    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        m_clientConnect = false;
        m_socket.close();
        if (!m_needClosed)
        {
            m_socket.async_connect(m_endpoint, boost::bind(&SocketClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
        }
        return;
    }

    //m_lastActiveTime = getTickCount();
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
        boost::bind(&SocketClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    dispathRoboKitMessage(message);
}

void SocketClient::socket_io_service_init()
{
//    boost::recursive_mutex::scoped_lock  scoped_lock(socket_mutex);
//    if (globalSocketClientInitMark == false)
//    {
//        globalSocketClientInitMark = true;
//        boost::thread * t1 = new boost::thread(boost::bind(&boost::asio::io_service::run, &socketNetClient_service));
//        boost::thread * t2 = new boost::thread(boost::bind(&boost::asio::io_service::run, &socketworkClient_service));
//    }
}

void SocketClient::sendOneMessageFromCacheList()
{
    if (m_msgWriteCacheList.size() > 0)
    {
        boost::shared_ptr<commonMsg> message = m_msgWriteCacheList.front();

        generateMsgFromJsonVal(message);
        boost::asio::async_write(m_socket, boost::asio::buffer(m_msgSendBuffer),
            boost::bind(&SocketClient::handleMsgSend, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }
}

void SocketClient::sendKeepAliveMessage(boost::system::error_code err)
{
    return;
}

void SocketClient::dispathRoboKitMessage(boost::shared_ptr<commonMsg> message)
{
    m_socket.get_io_service().post(boost::bind(&SocketClient::dispathRoboKitMessageIMPL, shared_from_this(), message));
}

void SocketClient::dispathRoboKitMessageIMPL(boost::shared_ptr<commonMsg> message)
{
    //currentbaseMessage->msgHeader.parseHeader(message->msgHeader.convertStructToUint8());
    //currentbaseMessage->msgBody.fromJsonStringToJsonVal((uint8_t*)message->msgBody.getJsonString().c_str(), message->msgBody.getJsonString().length());
    //memcpy((void*)&currentbaseMessage->msgHeader, (void*)&message->msgHeader, sizeof(header));
    //memcpy((void*)&currentbaseMessage->msgBody, (void*)&message->msgBody, sizeof(commonMsg));
    
    ROS_INFO("dispathRoboKitMessageIMPL recv message,  ssid:%s, msgId:%s, typeId:%d", 
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

    //message = boost::shared_ptr<commonMsg>(new commonMsg);
}
