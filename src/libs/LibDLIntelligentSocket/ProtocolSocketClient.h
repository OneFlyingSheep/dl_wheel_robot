#include <common/DLRobotCommonDef.h>
#include <boost/shared_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/atomic.hpp>
#include <boost/regex.hpp>
#include "XmlProtocolMsg.h"
#define PROTOCOL_MAX_BUFF_LENGTH (2 * 64 * 1024 - 1)

typedef boost::function<void (boost::shared_ptr<XmlProtocolMsg>) >        protocolHandleFunc;

class ProtocolSocketClient :
    public boost::enable_shared_from_this <ProtocolSocketClient>
{
public:
    ProtocolSocketClient(boost::asio::ip::tcp::endpoint &end, boost::asio::io_service &io_service);
    ~ProtocolSocketClient();

public:    
    void initSocket();

    void registerMsgHandle(protocolHandleFunc func);
    void closeSocket();
    bool IsConnected();
    void postMsg(boost::shared_ptr<XmlProtocolMsg> message);

private:
    void handle_resolve(const boost::system::error_code& err, boost::asio::ip::tcp::resolver::iterator endpoint_iterator);
    void handle_connect(boost::asio::ip::tcp::resolver::iterator endpoint_iterator, const boost::system::error_code& err);
    void handle_reconnect(boost::asio::ip::tcp::endpoint epp, const boost::system::error_code& err);

    void handle_readMsgHeader(const boost::system::error_code& err, std::size_t bytes_transferred);
    void handleMsgSend(const boost::system::error_code& error, std::size_t bytes_transferred);
    bool generateMsg(boost::shared_ptr<XmlProtocolMsg> msg);
    void dispathRoboKitMessage(boost::shared_ptr<XmlProtocolMsg> message);
    void dispathRoboKitMessageImpl(boost::shared_ptr<XmlProtocolMsg> message);
//    void parse_buff_func();
    void sendOneMessageFromCacheList();


private:
    bool msgCheck(std::vector<unsigned char> buff);

    boost::mutex                                            m_sendMutex;
    boost::mutex                                            m_connectMutex;
    boost::asio::ip::tcp::resolver                          m_resolver;

    boost::atomic_bool                                      m_clientConnect;
    boost::atomic_bool                                      m_needClosed;

    boost::asio::ip::tcp::socket                            m_socket;
    boost::asio::ip::tcp::endpoint                          m_endpoint;

    std::string                                             m_server;
    std::string                                             m_port;
    std::string                                             m_fullAddr;

    protocolHandleFunc                                      m_handleFunc;

    std::vector<unsigned char>                              m_msgSendBuffer;
    std::vector<unsigned char>                              m_msgRecvBuff;
    boost::asio::streambuf                                  m_msgRecvStreamBuff;

    //unsigned char                                           m_msgRecvBuff[PROTOCOL_MAX_BUFF_LENGTH];
    int                                                     m_msgRecvBuffWritePoint;
    int                                                     m_msgRecvBuffReadPoint;
    boost::mutex                                            m_msgRecvBuffLock;

    
    boost::thread                                           m_parseBuffThread;
    bool                                                    bThreadRunning;

    std::list<boost::shared_ptr<XmlProtocolMsg>>            m_msgWriteCacheList;

    boost::regex                                            *m_msgRegex;

};