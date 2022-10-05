#include "ProtocolSocketClient.h"
#include <stdlib.h>

ProtocolSocketClient::ProtocolSocketClient(boost::asio::ip::tcp::endpoint &end, boost::asio::io_service &io_service) :
    m_resolver(io_service), m_socket(io_service)
{
    m_msgRegex = new boost::regex(".*\xEB\x90.*\xEB\x90");
    m_server = end.address().to_string();
    //memcpy(m_msgRecvBuff, 0, PROTOCOL_MAX_BUFF_LENGTH);
    m_msgRecvBuffWritePoint = 0;
    m_msgRecvBuffReadPoint = 0;
    m_msgSendBuffer.resize(PROTOCOL_MAX_BUFF_LENGTH);
    m_msgRecvBuff.resize(PROTOCOL_MAX_BUFF_LENGTH);
    char tmpBuff[1024] = { 0 };
    itoa(end.port(), tmpBuff, 10);
    m_port = std::string(tmpBuff);
    m_clientConnect = false;
    m_needClosed = false;
}

ProtocolSocketClient::~ProtocolSocketClient()
{
    if (m_msgRegex)
    {
        delete m_msgRegex;
        m_msgRegex = NULL;
    }
    m_needClosed = true;
    closeSocket();
}

void ProtocolSocketClient::initSocket()
{
    boost::asio::ip::tcp::resolver::query query(m_server, m_port);
    m_resolver.async_resolve(query,
        boost::bind(&ProtocolSocketClient::handle_resolve, shared_from_this(),
            boost::asio::placeholders::error,
            boost::asio::placeholders::iterator));
}

void ProtocolSocketClient::registerMsgHandle(protocolHandleFunc func)
{
    m_handleFunc = func;
}

void ProtocolSocketClient::closeSocket()
{
    printf("close socket\n");
    m_needClosed = true;

    if (m_clientConnect)
    {
        m_socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
    }

    m_socket.close();
}

bool ProtocolSocketClient::IsConnected()
{
    return m_clientConnect;
}

void ProtocolSocketClient::postMsg(boost::shared_ptr<XmlProtocolMsg> message)
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

    //    ROS_INFO("str:%s, size:%d, type:%d", message.getJsonString().c_str(), message.getJsonString().length(), message.getMsgType());

    m_msgWriteCacheList.push_back(message);

    //    sendOneMessageFromeCacheList();
    if (needSend && m_clientConnect == true)
    {
        sendOneMessageFromCacheList();
    }
}

void ProtocolSocketClient::handle_resolve(const boost::system::error_code& err, boost::asio::ip::tcp::resolver::iterator endpoint_iterator)
{
    ROS_INFO("SocketClient::handle_resolve");
    if (!err)
    {
        // Attempt a connection to each endpoint in the list until we
        // successfully establish a connection.
        std::stringstream ss;
        ss << (endpoint_iterator->endpoint()) << std::endl;
        m_fullAddr = ss.str();
        ROS_INFO("host resolve success,ip is %s", ss.str().c_str());
        boost::asio::async_connect(m_socket, endpoint_iterator,
            boost::bind(&ProtocolSocketClient::handle_connect, shared_from_this(),
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
            boost::bind(&ProtocolSocketClient::handle_resolve, shared_from_this(),
                boost::asio::placeholders::error,
                boost::asio::placeholders::iterator));
    }
}

void ProtocolSocketClient::handle_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iterator, const boost::system::error_code& err)
{
    ROS_INFO("SocketClient::handle_connect");
    boost::mutex::scoped_lock lock(m_connectMutex);
    if (!err)
    {
        m_clientConnect = true;
        ROS_WARN(" connect success");
        m_endpoint = *endpoint_iterator;

        if (m_msgWriteCacheList.size() != 0)
        {
            sendOneMessageFromCacheList();
        }

        m_msgRecvStreamBuff.consume(m_msgRecvStreamBuff.size());
        boost::asio::async_read_until(m_socket, m_msgRecvStreamBuff, *m_msgRegex,
            boost::bind(&ProtocolSocketClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        ROS_WARN(" connect lost");
        m_clientConnect = false;

        if (m_needClosed == false)
        {
            ROS_ERROR("connect to %s failed , (%s) Retry after 5 seconds", m_fullAddr.c_str(), err.message().c_str());
            m_socket.close();
            boost::xtime xt;
            boost::xtime_get(&xt, boost::TIME_UTC_);
            xt.sec += 5;
            boost::thread::sleep(xt);
            boost::asio::async_connect(m_socket, endpoint_iterator,
                boost::bind(&ProtocolSocketClient::handle_connect, shared_from_this(),
                    endpoint_iterator, boost::asio::placeholders::error));
        }
    }
}

void ProtocolSocketClient::handle_reconnect(boost::asio::ip::tcp::endpoint epp, const boost::system::error_code& err)
{
    ROS_INFO("SocketClient::handle_connect");
    boost::mutex::scoped_lock lock(m_connectMutex);
    if (!err)
    {
        m_clientConnect = true;
        ROS_WARN(" connect success");

        if (m_msgWriteCacheList.size() != 0)
        {
            sendOneMessageFromCacheList();
        }

        m_msgRecvStreamBuff.consume(m_msgRecvStreamBuff.size());
        boost::asio::async_read_until(m_socket, m_msgRecvStreamBuff, *m_msgRegex,
            boost::bind(&ProtocolSocketClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
    }
    else
    {
        ROS_WARN(" connect lost");
        m_clientConnect = false;

        if (m_needClosed == false)
        {
            ROS_ERROR("connect to %s failed , (%s) Retry after 5 seconds", m_fullAddr.c_str(), err.message().c_str());
            m_socket.close();
            boost::xtime xt;
            boost::xtime_get(&xt, boost::TIME_UTC_);
            xt.sec += 5;
            boost::thread::sleep(xt);
            m_socket.async_connect(epp, boost::bind(&ProtocolSocketClient::handle_reconnect, shared_from_this(), epp, boost::asio::placeholders::error));

        }
    }
}

void ProtocolSocketClient::handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred)
{
    if (err)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", err.message().c_str());
        m_clientConnect = false;
        m_socket.close();

        boost::xtime xt;
        boost::xtime_get(&xt, boost::TIME_UTC_);
        xt.sec += 5;
        boost::thread::sleep(xt);
        m_socket.async_connect(m_endpoint, boost::bind(&ProtocolSocketClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
        return;
    }

    if (m_needClosed == true)
    {
        ROS_WARN("%s---close sockets");
        m_clientConnect = false;
        m_socket.close();

        boost::xtime xt;
        boost::xtime_get(&xt, boost::TIME_UTC_);
        xt.sec += 5;
        boost::thread::sleep(xt);
        m_socket.async_connect(m_endpoint, boost::bind(&ProtocolSocketClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
        return;
    }

    m_msgRecvBuff.resize(m_msgRecvStreamBuff.size());
    unsigned char* output = &*m_msgRecvBuff.begin();
    memcpy(output, boost::asio::buffer_cast<const void*>(m_msgRecvStreamBuff.data()), m_msgRecvStreamBuff.size());

    if (msgCheck(m_msgRecvBuff))
    {
        boost::shared_ptr<XmlProtocolMsg> msg(new XmlProtocolMsg);
        msg->setOriginalMsg(m_msgRecvBuff);
        dispathRoboKitMessage(msg);
    }

    m_msgRecvStreamBuff.consume(m_msgRecvStreamBuff.size());
    boost::asio::async_read_until(m_socket, m_msgRecvStreamBuff, *m_msgRegex,
        boost::bind(&ProtocolSocketClient::handle_readMsgHeader, shared_from_this(), boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
}

void ProtocolSocketClient::handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred)
{
    boost::mutex::scoped_lock lock(m_sendMutex);
    if (m_needClosed)
    {
        m_socket.close();
        m_clientConnect = false;

        boost::xtime xt;
        boost::xtime_get(&xt, boost::TIME_UTC_);
        xt.sec += 5;
        boost::thread::sleep(xt);
        m_socket.async_connect(m_endpoint, boost::bind(&ProtocolSocketClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
        return;
    }
    if (error && m_clientConnect)
    {
        ROS_ERROR("sock.read_some(): An error occurred:%s", error.message().c_str());
        m_clientConnect = false;

        m_socket.close();
        boost::xtime xt;
        boost::xtime_get(&xt, boost::TIME_UTC_);
        xt.sec += 5;
        boost::thread::sleep(xt);
        m_socket.async_connect(m_endpoint, boost::bind(&ProtocolSocketClient::handle_reconnect, shared_from_this(), m_endpoint, boost::asio::placeholders::error));
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
            m_msgWriteCacheList.erase(m_msgWriteCacheList.begin());
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

bool ProtocolSocketClient::generateMsg(boost::shared_ptr<XmlProtocolMsg> msg)
{
    std::string str;
    OriginalBaseMsg *ori_msg = msg->getOriginalMsg();
    int x = sizeof(OriginalBaseMsg);
    if (ori_msg != NULL)
    {
        int leng = 16 + ori_msg->xml_length_;

        m_msgSendBuffer.resize(leng, 0);

        uint8_t *p = &(*m_msgSendBuffer.begin());

        memcpy(p, ori_msg->start_flag_, sizeof(ori_msg->start_flag_));
        p += sizeof(ori_msg->start_flag_);

        memcpy(p, &ori_msg->sequence_id_, sizeof(ori_msg->sequence_id_));
        p += sizeof(ori_msg->sequence_id_);

        memcpy(p, &ori_msg->xml_length_, sizeof(ori_msg->xml_length_));
        p += sizeof(ori_msg->xml_length_);

        memcpy(p, ori_msg->xml_string_.c_str(), ori_msg->xml_length_);
        p += ori_msg->xml_length_;

        memcpy(p, &ori_msg->end_flag_, sizeof(ori_msg->end_flag_));
        p += sizeof(ori_msg->end_flag_);

        return msgCheck(m_msgSendBuffer);
    }
    else
    {
        return false;
    }

}

void ProtocolSocketClient::dispathRoboKitMessage(boost::shared_ptr<XmlProtocolMsg> message)
{
    m_socket.get_io_service().post(boost::bind(&ProtocolSocketClient::dispathRoboKitMessageImpl, shared_from_this(), message));
}

void ProtocolSocketClient::dispathRoboKitMessageImpl(boost::shared_ptr<XmlProtocolMsg> message)
{
    // msg handle
    if (message->decode())
    {
        m_handleFunc(message);
    }
    else
    {
        ROS_ERROR("decode failed");
    }

}

void ProtocolSocketClient::sendOneMessageFromCacheList()
{
    if (m_msgWriteCacheList.size() > 0)
    {
        boost::shared_ptr<XmlProtocolMsg> message = m_msgWriteCacheList.front();
        if (generateMsg(message))
        {
            boost::asio::async_write(m_socket, boost::asio::buffer(m_msgSendBuffer),
                boost::bind(&ProtocolSocketClient::handleMsgSend, shared_from_this(),
                    boost::asio::placeholders::error,
                    boost::asio::placeholders::bytes_transferred));
        }
        else
        {
            m_msgWriteCacheList.erase(m_msgWriteCacheList.begin());
        }
    }
}

bool ProtocolSocketClient::msgCheck(std::vector<unsigned char> buff)
{
    int leng = buff.size();
    if (buff[0] == 0xEB && buff[1] == 0x90 && buff[leng - 2] == 0xEB && buff[leng - 1] == 0x90)
    {
        return true;
    }
    else
    {
        ROS_ERROR("msg check failed!!");
        return false;
    }
}

// void ProtocolSocketClient::parse_buff_func()
// {
//     while (bThreadRunning)
//     {
//         Sleep(10);
//         boost::mutex::scoped_lock lock(m_msgRecvBuffLock);
//         if (m_msgRecvBuffWritePoint != m_msgRecvBuffReadPoint)
//         {
//             std::vector<unsigned char> parseBuff;
//             //std::vector<unsigned char>::iterator ptr_ = parseBuff.begin();
//             bool bFind = false;
//             int i = m_msgRecvBuffReadPoint;
//             parseBuff.push_back(m_msgRecvBuff[i]);
//             parseBuff.push_back(m_msgRecvBuff[(i + 1) % PROTOCOL_MAX_BUFF_LENGTH]);
//             i = (m_msgRecvBuffReadPoint + 2) % PROTOCOL_MAX_BUFF_LENGTH;
// 
//             for (i; i != m_msgRecvBuffWritePoint; i = (i + 1) % PROTOCOL_MAX_BUFF_LENGTH)
//             {
//                 if (m_msgRecvBuff[i] == 0xEB && m_msgRecvBuff[(i + 1) % PROTOCOL_MAX_BUFF_LENGTH] == 0x90)
//                 {
//                     parseBuff.push_back(m_msgRecvBuff[i]);
//                     parseBuff.push_back(m_msgRecvBuff[(i + 1) % PROTOCOL_MAX_BUFF_LENGTH]);
//                     bFind = true;
//                 }
//                 else
//                 {
//                     parseBuff.push_back(m_msgRecvBuff[i]);
//                 }
//             }
// 
//             if (bFind)
//             {
//                 m_msgRecvBuffReadPoint = (i + 2) % PROTOCOL_MAX_BUFF_LENGTH;
//                 // deal msg;
//                 //dispathRoboKitMessage();
//             }
//         }
//     }
// }
