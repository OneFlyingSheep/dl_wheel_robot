#pragma once
#include "HCNetCamera/GeneralDef.h"
#include "HCNetCamera/HCNetSDK.h"
#include <string>
#include <mmsystem.h>

class BJCameraCtrlSDK
{
public:
	BJCameraCtrlSDK(); 
    ~BJCameraCtrlSDK() {}

    void CameraZoomFocusStop();
	void CameraZoomIn();
	void CameraZoomOut();
	void CameraFocusFar();
	void CameraFocusNear();
	bool getImage(std::string name);

    bool startRecord(std::string name);
    void stopRecord();
    void setZoom(DWORD zoom);
    void setPtzPos(NET_DVR_PTZPOS ptz);
    void setFocus(DWORD focus);
private:
	LOCAL_DEVICE_INFO m_dev;
    LONG m_index = 0;
};