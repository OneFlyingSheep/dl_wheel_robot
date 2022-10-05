#include <LibAudioUdpClient.h>

LibAudioUdpClient::LibAudioUdpClient(std::string ip_address, int port) : endpoint_(boost::asio::ip::address_v4::from_string(ip_address), port), udp_(NULL)
{
}

LibAudioUdpClient::~LibAudioUdpClient()
{
    if (udp_)
    {
        delete udp_;
    }
}

bool LibAudioUdpClient::init()
{
    if (udp_ != NULL)
    {
        udp_ = new LibUdpClient(endpoint_);
    }
    return true;
}

bool LibAudioUdpClient::sent_data(uint8_t *data, uint32_t len)
{
    audioPacket p(1, len, data);

    std::vector<unsigned char> tmpBuff;

    tmpBuff.resize(p.getFormatLength(), 0);

    memset(&(*tmpBuff.begin()), 0, tmpBuff.size());

    memcpy(&(*tmpBuff.begin()), p.getFormatData(), p.getFormatLength());

    bool bSent = udp_->send_audio_packet_to_server(tmpBuff);

    if (bSent)
    {
        ROS_INFO("send_audio_packet_to_server succeed!");
    }
    else
    {
        ROS_INFO("send_audio_packet_to_server failed!");
    }

    return bSent;
}
