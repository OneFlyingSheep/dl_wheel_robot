#include <common/DLRobotCommonDef.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread/mutex.hpp>

#include "audioPacket.h"

using boost::asio::ip::udp;

class LibUdpClient
{
public:
    LibUdpClient(boost::asio::ip::udp::endpoint ep);
    ~LibUdpClient();

    bool send_audio_packet_to_server(std::vector<unsigned char> msg);
protected:
private:
    boost::asio::io_service io_service_;
    boost::asio::io_service::work io_work_;
    boost::asio::ip::udp::socket udp_socket_;
    boost::asio::ip::udp::endpoint receiver_udp_ep_;

    boost::mutex send_mutex_;
};