#include "LibAudioRtpClient.h"

LibAudioRtpClient::LibAudioRtpClient(std::string ip_addr, int local_port, int remote_port) : remote_ipstr_(ip_addr), local_port_(local_port), remote_port_(remote_port)
{
    destip_ = inet_addr(remote_ipstr_.c_str());
    if (destip_ == INADDR_NONE)
    {
        std::cerr << "Bad IP address specified" << std::endl;
        ROS_ERROR("Bad IP address specified");
    }
    else
    {
        destip_ = ntohl(destip_);
    } 
}

LibAudioRtpClient::~LibAudioRtpClient()
{

}

bool LibAudioRtpClient::init()
{
    int status;

    sessparams.SetOwnTimestampUnit(1.0 / 8000.0);

    sessparams.SetAcceptOwnPackets(true);
    transparams.SetPortbase(local_port_);
    status = rtp_session_.Create(sessparams, &transparams);

    if (!check_error(status))
    {
        return false;
    }

    RTPIPv4Address addr(destip_, remote_port_);

    status = rtp_session_.AddDestination(addr);

    if (!check_error(status))
    {
        return false;
    }

    return true;
}

bool LibAudioRtpClient::sent_data(uint8_t *data, uint32_t len)
{
    int status = rtp_session_.SendPacket((void *)data, len, 7, false, 10);
    return check_error(status);
}

bool LibAudioRtpClient::check_error(int status)
{
    if (status < 0)
    {
        ROS_ERROR("RTP ERROR:%s", RTPGetErrorString(status).c_str());
        return false;
    }
    else
    {
        return true;
    }
}
