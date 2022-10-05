#pragma once
#ifndef __SOCKET_SERVER_H__
#define __SOCKET_SERVER_H__

#include "common/DLHangRailRobotGlobalDef.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>
#include <LibCommonMsg/commonMsg.h>
#include <LibSocket/generaldef.h>
#include <boost/signals2/signal.hpp>
#include <boost/ref.hpp>
#include <boost/msm/back/state_machine.hpp>

class socket_connection
    : public boost::enable_shared_from_this<socket_connection>
{
public:
    socket_connection(boost::asio::io_service& io_service);
    ~socket_connection();

    typedef boost::shared_ptr<socket_connection> pointer;

    static pointer create(boost::asio::io_service& io_service);

    boost::asio::ip::tcp::socket& socket();

    boost::function<void()> checkConnection;

    void begin();

    void postMsg(boost::shared_ptr<commonMsg> message);
    void registerMsgHandles(std::map<int, commonMsgCallBack> callBackMap);
    void closeSession();

    bool getConnectStatus();

    hangRobotUserLoginRetVal getRole();
    void send_Login_retVal(hangRobotUserLoginRetVal retVal);

    std::string getSsid();

    UserType getUserType();


private:
    void handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred);
    void handle_readCommonMsgBody(const boost::system::error_code& err, boost::shared_ptr<commonMsg> message, std::size_t bytes_transferred);
    void generateMsgFromCommonVal(boost::shared_ptr<commonMsg> msg);
    void handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred);
    void sendOneMessageFromeCacheList();

    void dispathRoboKitMessage(boost::shared_ptr<commonMsg> message);
    void dispathRoboKitMessageIMPL(boost::shared_ptr<commonMsg> message);

    boost::asio::ip::tcp::socket                    m_socket;
    boost::mutex                                    m_sendMutex;
    boost::recursive_mutex                          socket_mutex;

    std::vector<unsigned char>                      m_msgSendBuffer;
    std::vector<unsigned char>                      m_msgRecvBuffer;

    boost::shared_ptr<commonMsg>                   currentbaseMessage;
    std::list<boost::shared_ptr<commonMsg > >      m_msgWriteCacheList;
    bool                                            m_needClosed;
    std::map<int, commonMsgCallBack>                  m_msgHandleMap;
    boost::atomic_bool                              m_connect;
    std::string                                     m_ssid;

    UserType                                        m_userType;
};

class SocketServer :
    public boost::enable_shared_from_this <SocketServer>
{
public:
    SocketServer(boost::asio::ip::tcp::endpoint &endpoint, boost::asio::io_service &io_service);
    ~SocketServer();

public:
    boost::signals2::signal<void()> signal_updateClientInitStatus;

public:
    boost::shared_ptr<socket_connection> getConnectionBySsid(std::string ssid);
    void initSocket();
    void postMsg(boost::shared_ptr<commonMsg> message);

    void postMsgByUserType(boost::shared_ptr<commonMsg> message, UserType type);
    void postMsgBySsid(boost::shared_ptr<commonMsg> message, std::string ssid);

    void registerMsgHandle(int messageID, commonMsgCallBack callback);
    void registerMsgHandleIMPL(int messageID, commonMsgCallBack callback);
    void closeSocket();
    void setMaxSession(int max);
    void connectAliveCallback(boost::system::error_code err);

    boost::asio::ip::tcp::endpoint getEndpoint();

    void checkConnection();

private:
    void handle_accept(socket_connection::pointer new_connection, const boost::system::error_code& error);

    boost::mutex                                                    m_socketVecLock;
    std::vector<boost::shared_ptr<socket_connection> >              m_socketVec;
    boost::asio::ip::tcp::acceptor                                  m_acceptor;
    boost::asio::ip::tcp::endpoint                                  m_endpoint;
    boost::shared_ptr<boost::asio::deadline_timer>                  m_timer;
    std::map<int, commonMsgCallBack>                                m_msgHandleMap;
    int                                                             m_maxSessionAllowed;
};

#endif
