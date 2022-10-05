#pragma once
#ifndef __CSOCKET_SERVER_H__
#define __CSOCKET_SERVER_H__

#include "baseSocket.h"

class tcp_connection
    : public boost::enable_shared_from_this<tcp_connection>
{
public:
    tcp_connection(boost::asio::io_service& io_service);
    ~tcp_connection();

    typedef boost::shared_ptr<tcp_connection> pointer;

    static pointer create(boost::asio::io_service& io_service);

    boost::asio::ip::tcp::socket& socket();

    void begin();

    void postMsg(jsonBody message);
    void registerMsgHandles(std::map<int, JsonMsgCallBack> callBackMap);
    void closeSession();

    bool getConnectStatus();

private:


    void handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred);
    void handle_readJsonBody(const boost::system::error_code& err, boost::shared_ptr<roboKitMsg> message, std::size_t bytes_transferred);
    void generateMsgFromJsonVal(boost::shared_ptr<roboKitMsg> msg);
    void handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred);
    void sendOneMessageFromeCacheList();

    void dispathRoboKitMessage(boost::shared_ptr<roboKitMsg> message);
    void dispathRoboKitMessageIMPL(boost::shared_ptr<roboKitMsg> message);

    boost::asio::ip::tcp::socket                    m_socket;
    boost::mutex                                    m_sendMutex;
    boost::recursive_mutex                          socket_mutex;

    std::vector<unsigned char>                      m_msgSendBuffer;
    std::vector<unsigned char>                      m_msgRecvBuffer;

    boost::shared_ptr<roboKitMsg>                   currentbaseMessage;
    std::list<boost::shared_ptr<roboKitMsg > >      m_msgWriteCacheList;
    bool                                            m_needClosed;
    std::map<int, JsonMsgCallBack>                  m_msgHandleMap;
    boost::atomic_bool                              m_connect;
};

class CSocketServer :
    public boost::enable_shared_from_this <CSocketServer>, public baseSocket
{
public:
    CSocketServer(boost::asio::ip::tcp::endpoint &endpoint, boost::asio::io_service &io_service);
    ~CSocketServer();

public:
    void initSocket();
    void postMsg(jsonBody message);
    void postClientMsg(boost::shared_ptr<roboKitMsg> message);
    void registerMsgHandle(int messageID, JsonMsgCallBack callback);
    void registerMsgHandleIMPL(int messageID, JsonMsgCallBack callback);
    void closeSocket();
    bool IsConnected();
    void setMaxSession(int max);
    void connectAliveCallback(boost::system::error_code err);

    void reconnect(boost::asio::ip::tcp::endpoint &end);

private:
    void handle_accept(tcp_connection::pointer new_connection, const boost::system::error_code& error);

    std::vector<boost::shared_ptr<tcp_connection> >                 m_socketVec;
    boost::asio::ip::tcp::acceptor                                  m_acceptor;
    boost::asio::ip::tcp::endpoint                                  m_endpoint;
    boost::shared_ptr<boost::asio::deadline_timer>                  m_timer;
};

#endif
