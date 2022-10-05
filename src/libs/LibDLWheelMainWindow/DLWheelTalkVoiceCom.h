#pragma once

#include "LibAudioClient/LibAudioRtpClient.h"
#include "LibHCNetCamera/HCNetCameraInterface.h"

class TalkVoiceCom
{
public:
	TalkVoiceCom();
	~TalkVoiceCom();

	void init();
	void startVoiceTalkCom();
	void stopVoiceTalkCom();
	void setHCNetCameraInterface(HCNetCameraInterface *object);

private:
	LibAudioClient *m_audioClient;
	HCNetCameraInterface* m_hCNetClient;

};

