#ifndef _VOICE_SPEAK_H
#define _VOICE_SPEAK_H

#include <stdio.h>
#include <atlbase.h>
#include <atlcom.h>
#include <sapi.h>
#include <QString>
#include <QStringList>
#include <unordered_map>
#include <boost/signals2.hpp>
#include <QMap>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "LibDLWheelMainWindow/WheelClientAlarmMsg.hpp"

#define MAX_SPEAK_LIST_SIZE 100

/************语音播放本地文本文件************/

class LibVoiceSpeak
{
public:
    LibVoiceSpeak();
    ~LibVoiceSpeak();

    // 设备语音播放的文件路径;
    static bool speakFile(QString strFilePath);

   // bool add2SpeakList(QString str);

	bool addSpeakErrorCode(AlarmMesgErrorCode &stAlarmMsgErrorCode);
	bool addSpeakErrorCode(QVector<AlarmMesgErrorCode> &vAlarmMsgErrorCodes);

    bool removeSpeakErrorCode(AlarmMesgErrorCode &stAlarmMsgErrorCode);

	//QVector<AlarmMesgErrorCode> GetSpeakingErrorCodes();

public:
    boost::signals2::signal<void(QString)> wheelRobotAlarmDisplay; // 机器人实时数据更新

private:
    bool speakSentence(QString sentence);

    bool getAndRemoveString(QString &str);

    void speakThread();

	QString GetValidSpeakMsg();  //获取有效的语音播报


private:
   // QStringList speakList_;
    boost::mutex speakListMutex_;
    boost::thread *speakThread_;
    bool bThreadRunning_;

	QVector<AlarmMesgErrorCode> m_vAloneSpeakingErrorCodes;     //单独的

	QVector<AlarmMesgErrorCode> m_vSpeakingErrorCodes;

	std::unordered_map<int, AlarmMesgErrorCode> m_mapID2ErrorCode;

 //   QString m_strDisplayAlarmMsgs{""};

    int m_iCurSpeakMsgIndex{0};
};

#endif // _VOICE_SPEAK_H
