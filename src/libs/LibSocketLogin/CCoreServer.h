#pragma once
#ifndef __CORE_SOCKET_SERVER_H__
#define __CORE_SOCKET_SERVER_H__

#include <common/DLWheelRobotGlobalDef.hpp>
#include "LibSocket/jsonBody.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include <LibConvertSymbolsToLua/LibConvertSymbolsToLua.h>
// #include <boost/signals2.hpp>
// #include <boost/bind.hpp>

class CCoreConnection
    : public boost::enable_shared_from_this<CCoreConnection>
{
public:
    CCoreConnection(boost::asio::io_service& io_service);
    ~CCoreConnection();

//	boost::signals2::signal<void(boost::shared_ptr<roboKitMsg>)> wheelcon; 

    boost::function<void()> checkConnection;

    typedef boost::shared_ptr<CCoreConnection> pointer;

    static pointer create(boost::asio::io_service& io_service);

    boost::asio::ip::tcp::socket& socket();

    void begin();

    void postMsg(jsonBody message);
    void postDirectMsg(boost::shared_ptr<roboKitMsg> message);
    void registerMsgHandles(std::map<int, JsonMsgCallBack> callBackMap);
    void closeSession();

    bool getConnectStatus();

    userLoginRetVal getRole();

    void robot_config_2d_map_query_req(boost::shared_ptr<roboKitMsg> msg);
    void robot_config_smap_query_req(boost::shared_ptr<roboKitMsg> msg);  
	void robot_creatr_report_query_req(boost::shared_ptr<roboKitMsg> msg);
	void robot_examine_report_isexist_query_req(boost::shared_ptr<roboKitMsg> msg);
	void robot_task_edit_insert_from_map_query_req(boost::shared_ptr<roboKitMsg> msg);

    void Remote_robot_threshold_by_device(boost::shared_ptr<roboKitMsg> msg);
    void Remote_robot_threshold_by_meter(boost::shared_ptr<roboKitMsg> msg);

private:
    void handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred);
    void handle_readJsonBody(const boost::system::error_code& err, boost::shared_ptr<roboKitMsg> message, std::size_t bytes_transferred);
    void generateMsgFromJsonVal(boost::shared_ptr<roboKitMsg> msg);
    void handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred, std::size_t sendBuffSize);
    void sendOneMessageFromeCacheList();

    void dispathRoboKitMessage(boost::shared_ptr<roboKitMsg> message);
    void dispathRoboKitMessageIMPL(boost::shared_ptr<roboKitMsg> message);

    void send_Login_retVal(userLoginRetVal retVal);

    boost::asio::ip::tcp::socket                    m_socket;
    boost::mutex                                    m_sendMutex;
    boost::recursive_mutex                          socket_mutex;

    std::vector<unsigned char>                      m_msgSendBuffer;
    std::vector<unsigned char>                      m_msgRecvBuffer;

    std::list<boost::shared_ptr<roboKitMsg > >      m_msgWriteCacheList;
    bool                                            m_needClosed;
    std::map<int, JsonMsgCallBack>                  m_msgHandleMap;
    boost::atomic_bool                              m_connect;

    conventSymbols2Lua                              m_convert2Lua;
};

class CCoreServer :
    public boost::enable_shared_from_this <CCoreServer>
{
public:
    CCoreServer(boost::asio::ip::tcp::endpoint &endpoint, boost::asio::io_service &io_service);
    ~CCoreServer();

public:
    void initSocket();
    void postMsg2AllConnection(jsonBody message);
    void postMsg2Engineer(jsonBody message);
    void postMsg2UserConnection(jsonBody message);
    void postMsg2ManagerConnection(jsonBody message);
    void postMsg2SuperManagerConnection(jsonBody message);
    void postDirectMsg2AllConnection(boost::shared_ptr<roboKitMsg> message);
    void postDirectMsg2Engineer(boost::shared_ptr<roboKitMsg> message);
    void postDirectMsg2UserConnection(boost::shared_ptr<roboKitMsg> message);
    void postDirectMsg2ManagerConnection(boost::shared_ptr<roboKitMsg> message);
    void postDirectMsg2SuperManagerConnection(boost::shared_ptr<roboKitMsg> message);
    void registerMsgHandle(int messageID, JsonMsgCallBack callback);
    void closeSocket();
    void setMaxSession(int max);

    void connectAliveCallback(boost::system::error_code err);

private:
    void handle_accept(CCoreConnection::pointer new_connection, const boost::system::error_code& error);
    void checkConnection();
    std::vector<boost::shared_ptr<CCoreConnection> >                 m_SuperManagerSocketVec;
    std::vector<boost::shared_ptr<CCoreConnection> >                 m_ManagerSocketVec;
    std::vector<boost::shared_ptr<CCoreConnection> >                 m_UserSocketVec;
    std::vector<boost::shared_ptr<CCoreConnection> >                 m_EngineerSocketVec;
    boost::asio::ip::tcp::acceptor                                  m_acceptor;
    boost::asio::ip::tcp::endpoint                                  m_endpoint;
    boost::shared_ptr<boost::asio::deadline_timer>                  m_timer;
    boost::mutex                                                    m_SuperManagerMutex;
    boost::mutex                                                    m_ManagerMutex;
    boost::mutex                                                    m_UserMutex;
    boost::mutex                                                    m_EngineerMutex;
    int                                             m_maxSessionAllowed;
    std::map<int, JsonMsgCallBack>                  m_msgHandleMap;

    std::list<boost::shared_ptr<roboKitMsg > >      m_msgWriteCacheList;
};

#endif
