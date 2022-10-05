#pragma once
#ifndef __CSOCKET_CLIENT_H__
#define __CSOCKET_CLIENT_H__

#include "baseSocket.h"

class CSocketClient :
    public boost::enable_shared_from_this <CSocketClient>, public baseSocket
{
public:
    CSocketClient(boost::asio::ip::tcp::endpoint &end, boost::asio::io_service &io_service);
    virtual ~CSocketClient();

public:
    void initSocket();

    void postMsg(jsonBody message);
    void postClientMsg(boost::shared_ptr<roboKitMsg> message);
    void registerMsgHandle(int messageID, JsonMsgCallBack callback);
    void registerMsgHandleIMPL(int messageID, JsonMsgCallBack callback);
    void closeSocket();
    bool IsConnected();
    void setMaxSession(int max);

    void reconnect(boost::asio::ip::tcp::endpoint &end);

private:
    void handle_resolve(const boost::system::error_code& err, boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
    void handle_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iterator, const boost::system::error_code& err);
    void handle_reconnect(boost::asio::ip::tcp::endpoint ep, const boost::system::error_code& err);
    void handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred);
    void handle_readJsonBody(const boost::system::error_code& err, boost::shared_ptr<roboKitMsg> message, std::size_t bytes_transferred);
    void socket_io_service_init();
    void generateMsgFromJsonVal(boost::shared_ptr<roboKitMsg> msg);
    void handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred);
    void sendOneMessageFromCacheList();
    void sendKeepAliveMessage(boost::system::error_code err);
    
    void dispathRoboKitMessage(boost::shared_ptr<roboKitMsg> message);
    void dispathRoboKitMessageIMPL(boost::shared_ptr<roboKitMsg> message);


private:
    boost::mutex                                    m_sendMutex;
    boost::mutex                                    m_connectMutex;
    boost::asio::ip::tcp::resolver                  m_resolver;


    //boost::uint64_t                                 m_lastActiveTime;
    boost::atomic_bool                              m_clientConnect;
    boost::atomic_bool                              m_needClosed;
    boost::shared_ptr<boost::asio::deadline_timer>  m_timer;

    boost::asio::ip::tcp::socket                    m_socket;
    boost::asio::ip::tcp::endpoint                  m_endpoint;

    std::string                                     m_server;
    //std::string                                     m_path;
    //std::string                                     m_param;
    std::string                                     m_port;
    std::string                                     m_fullAddr;
    std::string                                     m_handleName;
    
    std::map<int, JsonMsgCallBack>                  m_msgHandleMap;
    boost::recursive_mutex                          socket_mutex;

    std::vector<unsigned char>                      m_msgSendBuffer;
    std::vector<unsigned char>                      m_msgRecvBuffer;

    //boost::shared_ptr<roboKitMsg>                   currentbaseMessage;
    std::list<boost::shared_ptr<roboKitMsg > >      m_msgWriteCacheList;
};

#endif
