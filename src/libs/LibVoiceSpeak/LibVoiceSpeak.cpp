#include "LibVoiceSpeak.h"

#include "LibDLWheelAlarmManager/AlarmMessageManager.h"

LibVoiceSpeak::LibVoiceSpeak()
{
    bThreadRunning_ = true;
    speakThread_ = new boost::thread(boost::bind(&LibVoiceSpeak::speakThread, this));
}

LibVoiceSpeak::~LibVoiceSpeak()
{
    if (speakThread_)
    {
        delete speakThread_;
        speakThread_ = NULL;
    }
}

bool LibVoiceSpeak::speakFile(QString strFilePath)
{
    ISpVoice * pVoice = NULL;

    if (FAILED(::CoInitialize(NULL)))
        return false;

    HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
    if (SUCCEEDED(hr))
    {
        FILE* file;
        file = fopen(strFilePath.toLatin1(), "r");
        if (file == NULL)
        {
            return 0;
        }
        else
        {
            char buff[1024];
            USES_CONVERSION;
            while (fgets(buff, 1024, file) != NULL)
            {
                wchar_t* test12 = A2W(buff);
                hr = pVoice->Speak(test12, 0, NULL);

            }
            pVoice->Release();
            pVoice = NULL;
        }
        ::CoUninitialize();
    }

    return true;
}

//bool LibVoiceSpeak::add2SpeakList(QString str)
//{
//    boost::mutex::scoped_lock lock(speakListMutex_);
//    if (speakList_.size() >= MAX_SPEAK_LIST_SIZE)
//    {
//        ROS_ERROR("speak list is full: %d", speakList_.size());
//        return false;
//    }
//    else
//    {
//        speakList_.push_back(str);
//        return true;
//    }
//}

bool LibVoiceSpeak::addSpeakErrorCode(AlarmMesgErrorCode &stAlarmMsgErrorCode)
{
	boost::mutex::scoped_lock lock(speakListMutex_);
	int index = 0;
	for (; index < m_vAloneSpeakingErrorCodes.size(); ++index)
	{
		if (stAlarmMsgErrorCode.errorId == m_vAloneSpeakingErrorCodes[index].errorId)
		{
			m_vAloneSpeakingErrorCodes[index].errorCode = stAlarmMsgErrorCode.errorCode;
		}
	}

	if (index == m_vAloneSpeakingErrorCodes.size())
	{
		m_vAloneSpeakingErrorCodes.push_back(stAlarmMsgErrorCode);
	}

	//m_vAloneSpeakingErrorCodes.push_back(stAlarmMsgErrorCode);
	if (m_mapID2ErrorCode.size() >= MAX_SPEAK_LIST_SIZE)
	{
		ROS_ERROR("speak list is full: %d", m_mapID2ErrorCode.size());
		return false;
	}
	else
	{
		std::unordered_map<int, AlarmMesgErrorCode>::iterator it = m_mapID2ErrorCode.find(stAlarmMsgErrorCode.errorId);
		if (it != m_mapID2ErrorCode.end())
		{
			AlarmMesgErrorCode &stCurrentErrorCode = it->second;
			if (stCurrentErrorCode.errorCode != stAlarmMsgErrorCode.errorCode)
			{
				stCurrentErrorCode.errorCode = stAlarmMsgErrorCode.errorCode;
				AlarmMessage stAlarmMessage;
				AlarmMessageManager::GetInstance()->GetAlarmMessage(stAlarmMsgErrorCode.errorId, stAlarmMessage);
				QString strDisplayMsgs = QString("%1%2").arg(stAlarmMessage.strDisplayAlarmMessage.c_str()).arg(stAlarmMsgErrorCode.errorCode);
				if (!strDisplayMsgs.isEmpty())
				{
					wheelRobotAlarmDisplay(strDisplayMsgs);
				}
			}
		}
		else
		{
			m_mapID2ErrorCode.insert(std::make_pair(stAlarmMsgErrorCode.errorId, stAlarmMsgErrorCode));

			AlarmMessage stAlarmMessage;
			AlarmMessageManager::GetInstance()->GetAlarmMessage(stAlarmMsgErrorCode.errorId, stAlarmMessage);
			QString strDisplayMsgs = QString("%1%2").arg(stAlarmMessage.strDisplayAlarmMessage.c_str()).arg(stAlarmMsgErrorCode.errorCode);
			if (!strDisplayMsgs.isEmpty())
			{
				wheelRobotAlarmDisplay(strDisplayMsgs);
			}
		}
		return true;
	}
}

bool LibVoiceSpeak::addSpeakErrorCode(QVector<AlarmMesgErrorCode> &vAlarmMsgErrorCodes)
{
	boost::mutex::scoped_lock lock(speakListMutex_);
	m_vSpeakingErrorCodes = vAlarmMsgErrorCodes;

	if (m_mapID2ErrorCode.size() >= MAX_SPEAK_LIST_SIZE)
	{
		ROS_ERROR("speak list is full: %d", m_mapID2ErrorCode.size());
		return false;
	}
	else
	{
		for (int index = 0; index < vAlarmMsgErrorCodes.size(); ++index)
		{
			AlarmMesgErrorCode stAlarmMesgErrorCode = vAlarmMsgErrorCodes[index];
			std::unordered_map<int, AlarmMesgErrorCode>::iterator it = m_mapID2ErrorCode.find(stAlarmMesgErrorCode.errorId);
			if (it != m_mapID2ErrorCode.end())
			{
				AlarmMesgErrorCode &stCurrentErrorCode = it->second;
				if (stCurrentErrorCode.errorCode != stAlarmMesgErrorCode.errorCode)
				{
					stCurrentErrorCode.errorCode = stAlarmMesgErrorCode.errorCode;

					AlarmMessage stAlarmMessage;
					AlarmMessageManager::GetInstance()->GetAlarmMessage(stAlarmMesgErrorCode.errorId, stAlarmMessage);
					QString strDisplayMsgs = QString("%1%2").arg(stAlarmMessage.strDisplayAlarmMessage.c_str()).arg(stAlarmMesgErrorCode.errorCode);
					if (!strDisplayMsgs.isEmpty())
					{
						wheelRobotAlarmDisplay(strDisplayMsgs);
					}
				}
			}
			else
			{
				m_mapID2ErrorCode.insert(std::make_pair(stAlarmMesgErrorCode.errorId, stAlarmMesgErrorCode));
				AlarmMessage stAlarmMessage;
				AlarmMessageManager::GetInstance()->GetAlarmMessage(stAlarmMesgErrorCode.errorId, stAlarmMessage);
				QString strDisplayMsgs = QString("%1%2").arg(stAlarmMessage.strDisplayAlarmMessage.c_str()).arg(stAlarmMesgErrorCode.errorCode);
				if (!strDisplayMsgs.isEmpty())
				{
					wheelRobotAlarmDisplay(strDisplayMsgs);
				}
			}
		}
		//m_vSpeakingErrorCodes.push_back(stAlarmMsgErrorCode);
		return true;
	}
}

bool LibVoiceSpeak::removeSpeakErrorCode(AlarmMesgErrorCode &stAlarmMsgErrorCode)
{
    QVector<AlarmMesgErrorCode>::iterator it = m_vAloneSpeakingErrorCodes.begin();
    while (it != m_vAloneSpeakingErrorCodes.end())
    {
        if (it->errorId == stAlarmMsgErrorCode.errorId)
        {
            it = m_vAloneSpeakingErrorCodes.erase(it);
            continue;
           // return true;
        }
        ++it;
    }
    return false;
}

//QVector<AlarmMesgErrorCode> LibVoiceSpeak::GetSpeakingErrorCodes()
//{
//	return m_vSpeakingErrorCodes;
//}

bool LibVoiceSpeak::speakSentence(QString sentence)
{
    ISpVoice * pVoice = NULL;

    if (FAILED(::CoInitialize(NULL)))
        return false;

    HRESULT hr = CoCreateInstance(CLSID_SpVoice, NULL, CLSCTX_ALL, IID_ISpVoice, (void **)&pVoice);
    if (SUCCEEDED(hr))
    {

        USES_CONVERSION;
        wchar_t* test12 = A2W(std::string(sentence.toLocal8Bit()).c_str());
        hr = pVoice->Speak(test12, 0, NULL);
        pVoice->Release();
        pVoice = NULL;
        ::CoUninitialize();
    }

    return true;
}

bool LibVoiceSpeak::getAndRemoveString(QString &str)
{
//    boost::mutex::scoped_lock lock(speakListMutex_);
  //  if (speakList_.size() > 0)
  //  {
  //      str = speakList_.front();
  //      speakList_.removeFirst();
		//m_vSpeakingErrorCodes.removeFirst();
  //      return true;
  //  }

	while (str.isEmpty())
	{
		if (m_mapID2ErrorCode.size() <= 0) break;
		str = GetValidSpeakMsg();
	}

	if (!str.isEmpty())
	{//如果不为空的话，发信号显示
	//	wheelRobotAlarmDisplay(m_strDisplayAlarmMsgs);
		return true;
	}
    return false;
}

QString LibVoiceSpeak::GetValidSpeakMsg()
{
	QString strvalidMsg = "";
	if (m_mapID2ErrorCode.size() > 0 && m_mapID2ErrorCode.size() > m_iCurSpeakMsgIndex)
	{
        int index = 0;
		std::unordered_map<int, AlarmMesgErrorCode>::iterator it = m_mapID2ErrorCode.begin();
		while (it != m_mapID2ErrorCode.end())
		{
            if (m_iCurSpeakMsgIndex == index)
            {
                AlarmMesgErrorCode stAlarmMesgErrorCode = it->second;

                if (m_vSpeakingErrorCodes.contains(stAlarmMesgErrorCode) || m_vAloneSpeakingErrorCodes.contains(stAlarmMesgErrorCode))
                {
                    AlarmMessage stAlarmMessage;
                    AlarmMessageManager::GetInstance()->GetAlarmMessage(stAlarmMesgErrorCode.errorId, stAlarmMessage);
                    strvalidMsg = stAlarmMessage.strSpeakAlarmMessage.c_str();
                }
                else
                {//处理显示消息
                    AlarmMessage stAlarmMessage;
                    AlarmMessageManager::GetInstance()->GetAlarmMessage(stAlarmMesgErrorCode.errorId, stAlarmMessage);
                    QString strDisplayMsgs = QString("%1%2(已复归)").arg(stAlarmMessage.strDisplayAlarmMessage.c_str()).arg(stAlarmMessage.strErrorCode.c_str());
                    if (!strDisplayMsgs.isEmpty())
                    {
                        wheelRobotAlarmDisplay(strDisplayMsgs);
                    }
                    m_mapID2ErrorCode.erase(it);  //删除一个索引值减少一个
                    --m_iCurSpeakMsgIndex;
                }
                ++m_iCurSpeakMsgIndex;
                if (m_mapID2ErrorCode.size() <= m_iCurSpeakMsgIndex)
                {
                    m_iCurSpeakMsgIndex = 0;
                }
                break;
            }
            ++index;
            ++it;
		}
	}
	return strvalidMsg;
}

void LibVoiceSpeak::speakThread()
{
    while (bThreadRunning_)
    {
	//	Sleep(1000);
		Sleep(200);
        QString str;
        if (getAndRemoveString(str))
        {
            speakSentence(str);
        }
    }
}
