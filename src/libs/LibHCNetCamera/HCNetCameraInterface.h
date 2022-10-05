#ifndef __HC_CAMERA_INTERFACE__
#define __HC_CAMERA_INTERFACE__
//#include "windows.h"
#include "HCNetCameraOper.h"
#include <QString>
#include <QOBject>
#include <QPixmap>
#include <QByteArray>
#include <QSize>
#include <boost/thread.hpp>
#include <mmsystem.h>

//Wav
#define BITS_PER_SAMPLE		16
#define CHANNEL				1
#define SAMPLES_PER_SECOND	16000

#define BIT_RATE_16000		16000
#define AUDENCSIZE          1280
#define AUDDECSIZE          80
#define BUFFER_SIZE         1280
#define BUFFER_NUM           6
#define AUDIOBUF			(80*40L)

typedef struct STRU_VOICEMR_INFO {
    int iDeviceIndex;   //设备号
    int iChanIndex;     //语音通道号，1，2
    LONG hVoiceHandle;  //语音句柄

    STRU_VOICEMR_INFO()
    {
        iDeviceIndex = -1;
        iChanIndex = -1;
        hVoiceHandle = -1;
    }
}VOICEMR_INFO, *PVOICEMR_INFO;

class HCNetCameraInterface : public HCNetCameraOper
{
	Q_OBJECT
public:
    HCNetCameraInterface(char *camIp, char *username, char *password, int portNum = 8000);
    HCNetCameraInterface(QString camIp, QString userName, QString passwd, int portNum = 8000);
    virtual ~HCNetCameraInterface();

	void getCameraZF();
public:
    bool connect();
    bool disconnect();

    bool initCamera(int index);
    bool startPlay(int index, HWND hwd);
    bool stopPlay();

	bool setHCNetDvrConfig();
	bool startVoiceTalk();
    bool stopVoiceTalk();

    bool startRecord(string name);
    void stopRecord();

    bool startRecordAudio(string audioPath);
    bool stopRecordAudio();

    bool startCallback(int index);

    bool getImage(string name);

    bool setZoom(DWORD zoom);
    bool setPtzPos(NET_DVR_PTZPOS ptz);
    NET_DVR_PTZPOS getPtzPos();

    bool setFocus(DWORD focus);
    DWORD getFocus();

    bool setZoomAndFocus(DWORD zoom, DWORD focus);

	void getZoomFocus(int &zoomValue, int &focusValue);
	void ZoomIn();
    void ZoomOut();
    void ZoomOutStop();
	void ZoomInStop();
    void FocusNear();
    void FocusFar();
    void FocusStop();
    void ZoomAndFocusStop();

    void startThread(char *buff, long size, int w, int h);

	int getHCFocus()
	{
		return m_focusValue;
	}
	int getHCZoom()
	{
		return m_zoomValue;
	}
//signals:
//    void sigVisibleNewFrame(QPixmap);

public slots:
	
	// 相机操作信号;
	void OnVisibleZoomInClickedSlot(bool b);

	void OnVisibleZoomOutClickedSlot(bool b);

	void OnVisibleFocusNearClickedSlot(bool b);

	void OnVisibleFocusFarClickedSlot(bool b);

	// 获取当前相机放大倍数;
	void onGetCameraZoomValue();

    // 获取当前相机焦点值;
    void onGetCameraFocusValue();

	// 相机抓图;
	void onGetCameraImage(QString fileName);

    bool onGetCameraImageBool(QString fileName);
    // 相机录制音频;
	void onRecordCameraAudio(bool isStartRecord, QString fileName = "");

private:
    void threadFunc(char *buff, long size, int w, int h);
    boost::thread *m_thread;

    bool bRecording;

    
    MMCKINFO    m_mmckinfoParent;
    MMCKINFO    m_mmckinfoChild;
    WAVEFORMATEX m_struWaveFormat;
    LONG m_lVoiceHandle;

	int m_zoomValue = -1;
	int m_focusValue = -1;
public:
    HMMIO m_hmmio;                 //音频文件句柄
    bool bIsRecordVoice;


protected:
    char m_cameraIp[32];
    char m_name[32];
    char m_password[32];
    long m_port;

    int m_step;
};

#endif // __HIK_CAMERA_INTERFACE__
