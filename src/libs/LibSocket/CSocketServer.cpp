#include "CSocketServer.h"
#include <iostream>

tcp_connection::tcp_connection(boost::asio::io_service& io_service)
    : m_socket(io_service)
{
    m_needClosed = false;
    m_connect = false;
    m_msgRecvBuffer.resize(MAX_BUFF_LENGTH);
    m_msgSendBuffer.resize(MAX_BUFF_LENGTH);
}

tcp_connection::~tcp_connection()
{
    //m_socket.get_io_service().stop();
}

tcp_connection::pointer tcp_connection::create(boost::asio::io_service& io_service)
{
    return pointer(new tcp_connection(io_service));
}

boost::asio::ip::tcp::socket& tcp_connection::socket()
{
    return m_socket;
}

void tcp_connection::begin()
{
    ROS_WARN(" connect success again");
    //connect success
    m_connect = true;
    m_msgRecvBuffer.resize(MSG_HEADER_LENGTH);
    boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
        boost::asio::transfer_exactly(MSG_HEADER_LENGTH),
        boost::bind(&tcp_connection::handle_readMsgHeader, shared_from_this(),
            boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void tcp_connection::postMsg(jsonBody message)
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

void tcp_connection::registerMsgHandles(std::map<int, JsonMsgCallBack> callBackMap)
{
    m_msgHandleMap = callBackMap;
}

void tcp_connection::closeSession()
{
    m_socket.close();
}

bool tcp_connection::getConnectStatus()
{
    return m_connect;
}

void tcp_connection::handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred)
{
    ROS_INFO("read");
    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        if (m_connect)
        {
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
            boost::bind(&tcp_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
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
                    boost::bind(&tcp_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
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
                boost::bind(&tcp_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
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
            boost::bind(&tcp_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)
        );
    }
    else
    {
        //normal
        m_msgRecvBuffer.resize(leftLength);
        boost::asio::async_read(m_socket, boost::asio::buffer(m_msgRecvBuffer),
            boost::asio::transfer_exactly(leftLength),
            boost::bind(&tcp_connection::handle_readJsonBody, shared_from_this(), boost::asio::placeholders::error, recvMessage, boost::asio::placeholders::bytes_transferred)
        );
    }
}

void tcp_connection::handle_readJsonBody(const boost::system::error_code& err, boost::shared_ptr<roboKitMsg> message, std::size_t bytes_transferred)
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
        boost::bind(&tcp_connection::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    dispathRoboKitMessage(message);
}

void tcp_connection::generateMsgFromJsonVal(boost::shared_ptr<roboKitMsg> msg)
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

void tcp_connection::handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    ROS_INFO("handleMsgSend");
    boost::mutex::scoped_lock lock(m_sendMutex);
    if (error)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", error.message().c_str());
        m_connect = false;
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
        ROS_INFO("7");
        if (m_msgWriteCacheList.empty())
        {
        }
        else
        {
            sendOneMessageFromeCacheList();
        }
    }
}

void tcp_connection::sendOneMessageFromeCacheList()
{
    ROS_INFO("sendOneMessageFromeCacheList : %d", m_msgWriteCacheList.size());
    if (m_msgWriteCacheList.size() > 0)
    {
        boost::shared_ptr<roboKitMsg> message = m_msgWriteCacheList.front();
        generateMsgFromJsonVal(message);
        ROS_INFO("sendOneMessageFromeCacheList message: %d", m_msgWriteCacheList.size());
        boost::asio::async_write(m_socket, boost::asio::buffer(m_msgSendBuffer),
            boost::bind(&tcp_connection::handleMsgSend, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }
}

void tcp_connection::dispathRoboKitMessage(boost::shared_ptr<roboKitMsg> message)
{
    ROS_INFO("dispathRoboKitMessage");
    m_socket.get_io_service().post(boost::bind(&tcp_connection::dispathRoboKitMessageIMPL, shared_from_this(), message));
}

void tcp_connection::dispathRoboKitMessageIMPL(boost::shared_ptr<roboKitMsg> message)
{
//    jsonBody tmpBody;
    currentbaseMessage = message;
    printf("recv message  %s\r\n", currentbaseMessage->msgBody.getJsonString().c_str());

    try
    {
        /*tmpBody = */m_msgHandleMap[currentbaseMessage->msgHeader.getType()](currentbaseMessage);
//        postMsg(tmpBody);
    }
    catch (...)
    {
        ROS_ERROR("dispathRoboKitMessageIMPL err, maybe haven't register function handle, Connection closed");
        m_connect = false;
        m_socket.close();
    }

    currentbaseMessage = boost::shared_ptr<roboKitMsg>(new roboKitMsg);
}

CSocketServer::CSocketServer(boost::asio::ip::tcp::endpoint &endpoint, boost::asio::io_service &io_service) :
    m_acceptor(io_service, endpoint)
{
    ROS_INFO("CSocketServer::CSocketServer");
    m_maxSessionAllowed = 0;
    m_endpoint = endpoint;
    m_socketVec.clear();
}

CSocketServer::~CSocketServer()
{
    ROS_INFO("Server closed");
    closeSocket();
}

void CSocketServer::initSocket()
{
    ROS_INFO("CSocketServer::initSocket");
    tcp_connection::pointer new_connection = tcp_connection::create(m_acceptor.get_io_service());

    m_acceptor.async_accept(new_connection->socket(),
        boost::bind(&CSocketServer::handle_accept, shared_from_this(), new_connection,
            boost::asio::placeholders::error));
}

void CSocketServer::postMsg(jsonBody message)
{
    if (m_socketVec.size() <= 0)
    {
        ROS_INFO("no connection");
        return ;
    }
    std::vector<boost::shared_ptr<tcp_connection> >::iterator itr = m_socketVec.begin();
    for (itr; itr != m_socketVec.end(); itr++)
    {
        (*itr)->postMsg(message);
    }
}

void CSocketServer::postClientMsg(boost::shared_ptr<roboKitMsg> message)
{
    ROS_ERROR("CSocketServer::postClientMsg err");
}

void CSocketServer::registerMsgHandle(int messageID, JsonMsgCallBack callback)
{
    m_acceptor.get_io_service().post(boost::bind(&CSocketServer::registerMsgHandleIMPL, shared_from_this(), messageID, callback));
}

void CSocketServer::registerMsgHandleIMPL(int messageID, JsonMsgCallBack callback)
{
    m_msgHandleMap[messageID] = callback;
}

void CSocketServer::closeSocket()
{

}

bool CSocketServer::IsConnected()
{
    if (m_socketVec.empty())
    {
        return false;
    }
    else
    {
        return true;
    }
}

void CSocketServer::setMaxSession(int max)
{
    m_maxSessionAllowed = max;
}

void CSocketServer::connectAliveCallback(boost::system::error_code err)
{
    if (m_socketVec.size() > 0)
    {
//        ROS_INFO("check CONNECTION");
        std::vector<boost::shared_ptr<tcp_connection> >::iterator itr = m_socketVec.begin();
        jsonBody keepalive;
        for (itr; itr != m_socketVec.end();)
        {
            //(*itr)->postMsg(keepalive);
            if (!(*itr)->getConnectStatus())
            {
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
        m_timer->async_wait(boost::bind(&CSocketServer::connectAliveCallback, shared_from_this(), boost::asio::placeholders::error));
    }

}

void CSocketServer::reconnect(boost::asio::ip::tcp::endpoint &end)
{
    //m_acceptor.close();
    //m_acceptor(io_service, end);
}

void CSocketServer::handle_accept(tcp_connection::pointer new_connection, const boost::system::error_code& error)
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
    }
    else
    {
        new_connection->begin();
        new_connection->registerMsgHandles(m_msgHandleMap);

        m_socketVec.push_back(new_connection);

        if (m_timer == NULL)
        {
            m_timer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(m_acceptor.get_io_service(), boost::posix_time::seconds(10)));
        }

        m_timer->async_wait(boost::bind(&CSocketServer::connectAliveCallback, shared_from_this(), boost::asio::placeholders::error));
    }

    boost::xtime xt;
    boost::xtime_get(&xt, boost::TIME_UTC_);
    xt.sec += 5;
    boost::thread::sleep(xt);
    tcp_connection::pointer connection = tcp_connection::create(m_acceptor.get_io_service());

    m_acceptor.async_accept(connection->socket(),
        boost::bind(&CSocketServer::handle_accept, shared_from_this(), connection,
            boost::asio::placeholders::error));
}
