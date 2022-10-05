#include "LibAudioClient.h"
#include<windows.h>
#include "LibUdpClient.h"
#include "audioPacket.h"

class LibAudioUdpClient : public LibAudioClient
{
public:
    LibAudioUdpClient(std::string ip_address, int port);
    ~LibAudioUdpClient();

    bool init();

    bool sent_data(uint8_t *data, uint32_t len);
   
private:
    LibUdpClient *udp_;
    boost::asio::ip::udp::endpoint endpoint_;
};