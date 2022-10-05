#ifndef BASEOPERATION_H
#define BASEOPERATION_H
#include "common/DLHangRailRobotGlobalDef.hpp"
#include "HCNetCamera/GeneralDef.h"
#include "HCNetCamera/HCNetSDK.h"
#include <iostream>
#include <string>
#include <sstream>
#include <QObject>
using namespace std;
#define DATABUF_LEN 512

#define HORIZONTAL_CHECK 1
#define VERTICAL_CHECK 2

#define HORIZONTAL_SET 3
#define VERTICAL_SET 4
#define IMAGEPAIRS 59
#define PTSTEP 100

#define MAX_CMD_LENGTH 7

class HCNetCameraOper : public QObject
{
    Q_OBJECT
public:
    HCNetCameraOper();
    virtual ~HCNetCameraOper();

public:
    bool init();
    bool release();

    bool Login(const char* ip, const char* name, const char* passwd, unsigned short port);

    virtual bool connect() = 0;
    virtual bool disconnect() = 0;

    virtual bool startPlay(int index, HWND hwd) = 0;
    virtual bool stopPlay() = 0;

    virtual bool getImage(string imgName) = 0;
    virtual bool setPtzPos(NET_DVR_PTZPOS ptz) = 0;

    virtual bool startRecordAudio(string audioPath) = 0;
    virtual bool stopRecordAudio() = 0;

    virtual NET_DVR_PTZPOS getPtzPos() = 0;

    void setPtzSpeed(int speed);
    int getPtzSpeed();

    void setLoginState(bool flag);
    bool getLoginState();

    void setPlayingState(bool flag);
    bool getPlayingState();

    void setRealplayHandle(long handle);
    long getRealplayHandle();

    void setCurrentChannel(int channel);
    long getCurrentChannel();

    void setLocalDevice(LOCAL_DEVICE_INFO info);
    LOCAL_DEVICE_INFO getLocalDevice();

    void GetDeviceResoureCfg();
    void GetDecoderCfg();

private:
    char *ipAddress;
    int m_iPtzSpeed;
    bool m_bIsLogin;
    bool m_bIsPlaying;

    long m_lPlayHandle;
    int m_iCurChanIndex;
    LOCAL_DEVICE_INFO m_struDeviceInfo;

};

#endif // BASEOPERATION_H
