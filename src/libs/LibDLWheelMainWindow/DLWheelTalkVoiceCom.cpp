#include "DLWheelTalkVoiceCom.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

TalkVoiceCom::TalkVoiceCom()
{
	init();
}

TalkVoiceCom::~TalkVoiceCom()
{

}

void TalkVoiceCom::init()
{
	m_audioClient = new LibAudioRtpClient(
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().strRtpIp.toStdString(), 
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().iRtpSendPort, 
		WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().iRtpReceivePort);

	m_audioClient->init();
}

void TalkVoiceCom::startVoiceTalkCom()
{
    try 
    {
        ROS_INFO("TalkVoiceCom: startVoiceTalkCom init_remote_audio start!");
        m_audioClient->init_remote_audio();
        ROS_INFO("TalkVoiceCom: startVoiceTalkCom startVoiceTalk start!");
        m_hCNetClient->startVoiceTalk();
        ROS_INFO("TalkVoiceCom: startVoiceTalkCom  start!");
    }
    catch (...)
    {
    }
    
}

void TalkVoiceCom::stopVoiceTalkCom()
{
    try
    {
        ROS_INFO("TalkVoiceCom: stopVoiceTalkCom stop_remote_audio end!");
        m_audioClient->stop_remote_audio();
        ROS_INFO("TalkVoiceCom: stopVoiceTalkCom stopVoiceTalk end!");
        m_hCNetClient->stopVoiceTalk();
        ROS_INFO("TalkVoiceCom: stopVoiceTalkCom  end!");
    }
    catch (...)
    {
    }
}

void TalkVoiceCom::setHCNetCameraInterface(HCNetCameraInterface *object)
{
	m_hCNetClient = object;
}
