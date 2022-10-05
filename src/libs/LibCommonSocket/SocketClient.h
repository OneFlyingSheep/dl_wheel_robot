#pragma once
#ifndef __SOCKET_CLIENT_H__
#define __SOCKET_CLIENT_H__

#include "common/DLHangRailRobotGlobalDef.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>
#include <LibCommonMsg/commonMsg.h>

class SocketClient :
    public boost::enable_shared_from_this <SocketClient>
{
public:
    SocketClient(boost::asio::ip::tcp::endpoint &end, boost::asio::io_service &io_service);
    virtual ~SocketClient();

public:
    hangRobotUserLoginRetVal doLogin(std::string username, std::string password, bool bReconnect = false);

    void initSocket();

    void postMsg(boost::shared_ptr<commonMsg> message);
    void postClientMsg(boost::shared_ptr<commonMsg> message);
    void registerMsgHandle(int messageID, commonMsgCallBack callback);
    void closeSocket();
    bool IsConnected();
    void setMaxSession(int max);

    void reconnect(boost::asio::ip::tcp::endpoint &end);

private:
    void handle_resolve(const boost::system::error_code& err, boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
    void handle_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iterator, const boost::system::error_code& err);
    void handle_reconnect(boost::asio::ip::tcp::endpoint ep, const boost::system::error_code& err);
    void handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred);
    void handle_readCommonMsg(const boost::system::error_code& err, boost::shared_ptr<commonMsg> message, std::size_t bytes_transferred);
    void socket_io_service_init();
    void generateMsgFromJsonVal(boost::shared_ptr<commonMsg> msg);
    void handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred);
    void sendOneMessageFromCacheList();
    void sendKeepAliveMessage(boost::system::error_code err);
    
    void dispathRoboKitMessage(boost::shared_ptr<commonMsg> message);
    void dispathRoboKitMessageIMPL(boost::shared_ptr<commonMsg> message);


private:
    boost::mutex                                            m_sendMutex;
    boost::mutex                                            m_connectMutex;
    boost::asio::ip::tcp::resolver                          m_resolver;

    boost::atomic_bool                                      m_clientConnect;
    boost::atomic_bool                                      m_needClosed;
    boost::shared_ptr<boost::asio::deadline_timer>          m_timer;

    boost::asio::ip::tcp::socket                            m_socket;
    boost::asio::ip::tcp::endpoint                          m_endpoint;

    std::string                                             m_server;
    std::string                                             m_port;
    std::string                                             m_fullAddr;
    std::string                                             m_handleName;
    
    std::map<int, commonMsgCallBack>                        m_msgHandleMap;
    boost::recursive_mutex                                  socket_mutex;

    std::vector<unsigned char>                              m_msgSendBuffer;
    std::vector<unsigned char>                              m_msgRecvBuffer;

    std::list<boost::shared_ptr<commonMsg>>                 m_msgWriteCacheList;

    std::string                                             m_ssid;

    bool                                            bLoginMode;

    std::string                                     m_userName;
    std::string                                     m_password;
};

#endif
