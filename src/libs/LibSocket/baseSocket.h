#pragma once
#include "generaldef.h"
#include <common/DLRobotCommonDef.h>
#include "jsonBody.h"
#include <boost/signals2.hpp>

class baseSocket
{
public:
    baseSocket();
    virtual ~baseSocket();

public:
    virtual void initSocket() = 0;

    virtual void postMsg(jsonBody message) = 0;
    virtual void postClientMsg(boost::shared_ptr<roboKitMsg> message) = 0;
    virtual void registerMsgHandle(int messageID, JsonMsgCallBack callback) = 0;
    virtual void registerMsgHandleIMPL(int messageID, JsonMsgCallBack callback) = 0;
    virtual void closeSocket() = 0;
    virtual bool IsConnected() = 0;
    virtual void setMaxSession(int max) = 0;
    virtual void reconnect(boost::asio::ip::tcp::endpoint &end) = 0;
    boost::signals2::signal<void()>                 signal_connected;

protected:
    int                                             m_maxSessionAllowed;
    std::map<int, JsonMsgCallBack>                  m_msgHandleMap;

    boost::shared_ptr<roboKitMsg>                   currentbaseMessage;
    std::list<boost::shared_ptr<roboKitMsg > >      m_msgWriteCacheList;
    

};















