#include "baseMsgOperation.h"

static boost::asio::io_service io_service;
static boost::asio::io_service::work io_work(io_service);

baseMsgOperation::baseMsgOperation()
{
    bTimerRunning = false;
    m_thread = NULL;
    m_CheckRespTimer = NULL;
}

baseMsgOperation::~baseMsgOperation()
{
    if (bTimerRunning)
    {
        bTimerRunning = false;
        m_CheckRespTimer->cancel();
    }

    m_baseSocket->closeSocket();
    io_service.stop();

    if (m_thread)
    {
        m_thread->join();
    }
}

bool baseMsgOperation::isConnected()
{
    if (m_baseSocket == NULL)
    {
        return false;
    }
    return m_baseSocket->IsConnected();
}

void baseMsgOperation::baseMsgPostMsg(jsonBody &msg)
{
    m_baseSocket->postMsg(msg);
}

void baseMsgOperation::postClientMsg(boost::shared_ptr<roboKitMsg> message)
{
    m_baseSocket->postClientMsg(message);
}

void baseMsgOperation::reconnect(std::string host, int  port)
{
    m_baseSocket->closeSocket();
}

void baseMsgOperation::renewConnect(std::string host, int port)
{
    m_baseSocket->closeSocket();
    ROS_INFO("renewConnect");
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string(host), port);
    m_baseSocket->reconnect(endpoint);
}

void baseMsgOperation::registConnectedSignal(boost::function<void()> func)
{
    m_baseSocket->signal_connected.connect(func);
}

bool baseMsgOperation::get_is_connect_robot()
{
    return m_baseSocket->IsConnected();
}

void baseMsgOperation::runMessageLoop(std::string host, int port, E_SOCKET_TYPE type, int maxSession /*= OTHER_SESSION_ALLOWED*/)
{
    ROS_INFO("runMessageLoop");
    boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address_v4::from_string(host), port);
    if (SOCK_TYPE_SERVER == type)
    {
        m_baseSocket = boost::shared_ptr<CSocketServer>(new CSocketServer(endpoint, io_service));
        registerHandles();
        m_baseSocket->setMaxSession(maxSession);
        m_baseSocket->initSocket();
    }
    else if (SOCK_TYPE_CLIENT == type)
    {
        m_baseSocket = boost::shared_ptr<CSocketClient>(new CSocketClient(endpoint, io_service));
        registerHandles();
        m_baseSocket->initSocket();
    }
    else
    {
        ROS_ERROR("Err socket type");
        return;
    }
    m_thread = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
}

void baseMsgOperation::runMessageLoop(boost::asio::ip::tcp host, int port, E_SOCKET_TYPE type, int maxSession)
{
    ROS_INFO("runMessageLoop1");
    boost::asio::ip::tcp::endpoint endpoint(host, port);
    if (SOCK_TYPE_SERVER == type)
    {
        m_baseSocket = boost::shared_ptr<CSocketServer>(new CSocketServer(endpoint, io_service));
        registerHandles();
        m_baseSocket->setMaxSession(maxSession);
        m_baseSocket->initSocket();

    }
    else if (SOCK_TYPE_CLIENT == type)
    {
        m_baseSocket = boost::shared_ptr<CSocketClient>(new CSocketClient(endpoint, io_service));
        registerHandles();
        m_baseSocket->initSocket();
    }
    else
    {
        ROS_ERROR("Err socket type");
        return;
    }
    m_thread = new boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
}

void baseMsgOperation::registerHandles()
{
    ROS_ERROR("virtual!!! baseMsgOperation::registerHandles");
}

bool baseMsgOperation::updateSendMessageStatus(int id, int type)
{
    checkDataMutex.lock();
    vector<STRU_SOCK_CHECK_DATA>::iterator itr = respCheckDataList.begin();
    for (itr; itr != respCheckDataList.end(); itr++)
    {
        if (itr->id == id)
        {
            if (itr->type == type)
            {
                itr->bRet = true;
                checkDataMutex.unlock();
                return true;
            }
            else
            {
                itr->bRet = false;
                ROS_ERROR("msg Id has wrong msg type");
                checkDataMutex.unlock();
                return false;
            }
        }
    }
    checkDataMutex.unlock();
    return false;
}

void baseMsgOperation::addToCheckSendMsgList(jsonBody body)
{
    return;
    if (!m_baseSocket->IsConnected())
        return ;
    ROS_INFO("addToCheckSendMsgList:%d",body.getMsgNumber());
    STRU_SOCK_CHECK_DATA cdata;
    cdata.id = body.getMsgNumber();
    cdata.type == body.getMsgType();
    cdata.time = time(NULL);
    cdata.bRet = false;
    checkDataMutex.lock();
    respCheckDataList.push_back(cdata);
    checkDataMutex.unlock();
}

void baseMsgOperation::eraseCheckSendMsgList()
{
    if (respCheckDataList.size() > 0)
    {
        STRU_SOCK_CHECK_DATA data = respCheckDataList.front();

        if (!data.bRet)
        {
            if (bMoreThanTenSec(data.time))
            {
                ROS_ERROR("backGroundCtrlMsg msgId:%d, msgType:%d, overtime", data.id, data.type);
                checkDataMutex.lock();
                respCheckDataList.erase(respCheckDataList.begin());
                checkDataMutex.unlock();
            }
        }
        else
        {
            checkDataMutex.lock();
            respCheckDataList.erase(respCheckDataList.begin());
            checkDataMutex.unlock();
        }
    }
}

void baseMsgOperation::startCheckDataTimer()
{
    bTimerRunning = true;
    if (NULL == m_CheckRespTimer)
    {
        m_CheckRespTimer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(1)));
    }
    m_CheckRespTimer->async_wait(boost::bind(&baseMsgOperation::checkSocketResp, this, boost::asio::placeholders::error));
}

void baseMsgOperation::checkSocketResp(boost::system::error_code err)
{
    eraseCheckSendMsgList();
    try
    {
        if (bTimerRunning && m_CheckRespTimer == NULL)
        {
            m_CheckRespTimer = boost::shared_ptr<boost::asio::deadline_timer>(new boost::asio::deadline_timer(io_service, boost::posix_time::seconds(1)));
        }
        if (bTimerRunning)
        {
            m_CheckRespTimer->expires_from_now(boost::posix_time::seconds(1));
            m_CheckRespTimer->async_wait(boost::bind(&baseMsgOperation::checkSocketResp, this, boost::asio::placeholders::error));
        }
    }
    catch (const std::exception&)
    {
        ROS_ERROR("timer:checkSocketResp exception");
    }

}

bool baseMsgOperation::bMoreThanTenSec(time_t t)
{
    time_t currTime = time(NULL);
    ROS_INFO("currTime:%d, t:%d", (int)currTime, (int)t);
    if (currTime - t > 10)
    {
        return true;
    }
    else
    {
        return false;
    }
}

