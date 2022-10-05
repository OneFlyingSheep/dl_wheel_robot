#ifndef __LIB_AUDIO_RTP_CLIENT_H__
#define __LIB_AUDIO_RTP_CLIENT_H__

#include "LibAudioClient.h"
#include "jrtplib3/rtpsession.h"
#include "jrtplib3/rtpudpv4transmitter.h"
#include "jrtplib3/rtpipv4address.h"
#include "jrtplib3/rtpsessionparams.h"
#include "jrtplib3/rtperrors.h"
#include "jrtplib3/rtplibraryversion.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>

using namespace jrtplib;

class LibAudioRtpClient : public LibAudioClient
{
public:
    LibAudioRtpClient(std::string ip_addr, int local_port, int remote_port);
    ~LibAudioRtpClient();

    bool init();

    bool sent_data(uint8_t *data, uint32_t len);

private:
    bool check_error(int status);
    RTPSession rtp_session_;
    RTPUDPv4TransmissionParams transparams;
    RTPSessionParams sessparams;
    std::string remote_ipstr_;
    uint16_t local_port_, remote_port_;
    uint32_t destip_;
    
};

#endif
