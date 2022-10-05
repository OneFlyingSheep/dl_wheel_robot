#pragma once
#include "CSocketClient.h"
#include "CSocketServer.h"
#include "mutex"
#include <boost/signals2.hpp>
using namespace std;
class baseMsgOperation
{
public:
    baseMsgOperation();
    virtual ~baseMsgOperation();

public:
    bool isConnected();
    void runMessageLoop(std::string host, int  port, E_SOCKET_TYPE type, int maxSession = OTHER_SESSION_ALLOWED);
    void runMessageLoop(boost::asio::ip::tcp host, int  port, E_SOCKET_TYPE type, int maxSession = OTHER_SESSION_ALLOWED);
    void baseMsgPostMsg(jsonBody &msg);
    void postClientMsg(boost::shared_ptr<roboKitMsg> message);
    void reconnect(std::string host, int  port);
    void renewConnect(std::string host, int port);

    void registConnectedSignal(boost::function<void()> func);
    
    bool get_is_connect_robot();
protected:
    virtual void registerHandles();
    bool updateSendMessageStatus(int id, int type);
    void addToCheckSendMsgList(jsonBody body);
    void eraseCheckSendMsgList();
    void startCheckDataTimer();

private:
    void checkSocketResp(boost::system::error_code err);

protected:
    boost::shared_ptr<baseSocket> m_baseSocket;
    vector<STRU_SOCK_CHECK_DATA> respCheckDataList;
    mutex checkDataMutex;
    bool bMoreThanTenSec(time_t t);

    boost::thread *m_thread;

    bool bTimerRunning;
    boost::shared_ptr<boost::asio::deadline_timer> m_CheckRespTimer;



};
