#include <LibUdpClient.h>

LibUdpClient::LibUdpClient(boost::asio::ip::udp::endpoint ep) : io_work_(io_service_), udp_socket_(io_service_)
{
    receiver_udp_ep_ = ep;

    udp_socket_.open(udp::v4());
}

LibUdpClient::~LibUdpClient()
{
    udp_socket_.shutdown(boost::asio::socket_base::shutdown_both);
    udp_socket_.close();
}

bool LibUdpClient::send_audio_packet_to_server(std::vector<unsigned char> msg)
{
    boost::mutex::scoped_lock lock(send_mutex_);
    try
    {
        size_t sent_len = udp_socket_.send_to(boost::asio::buffer(msg), receiver_udp_ep_);

        return sent_len == msg.size();
    }
    catch (...)
    {
        ROS_ERROR("send_to_server error");
        return false;
    }

}
