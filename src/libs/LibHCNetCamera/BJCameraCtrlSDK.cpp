#include "BJCameraCtrlSDK.h"
#include <QDebug>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <sstream>
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"

using namespace std;
void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
}
void CALLBACK slBack(LONG lUserID, DWORD dwType, LPNET_DVR_DEVICEINFO_V30 info, void *pUser)
{
}

BJCameraCtrlSDK::BJCameraCtrlSDK()
{
	NET_DVR_Init();

 	NET_DVR_DEVICEINFO_V30 DeviceInfoTmp;
 	memset(&DeviceInfoTmp, 0, sizeof(NET_DVR_DEVICEINFO_V30));
 

    QString hkIp = WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().robotIp;
    int hkPort = WHEEL_ROBOT_CORE_CONFIG.getCurrentRobotCfg().hcCtrlPort;


    QByteArray ba;
    ba = hkIp.toLatin1();
    char * ch = ba.data();

    LONG lLoginID = NET_DVR_Login_V30(ch, hkPort, "admin", "q1w2e3r4", &DeviceInfoTmp);
    qDebug() << "lLoginID" << lLoginID;

	if (lLoginID == -1)
	{
        qDebug() << "lLoginID error";
		DWORD err = NET_DVR_GetLastError();
	}
	LOCAL_DEVICE_INFO dev;
	dev.lLoginID = lLoginID;
	dev.iDeviceChanNum = DeviceInfoTmp.byChanNum;
//    dev.iIPChanNum = 38;//DeviceInfoTmp.byIPChanNum;
    dev.iIPChanNum = DeviceInfoTmp.byIPChanNum + 1;
	dev.iStartChan = DeviceInfoTmp.byStartChan;
	dev.iIPStartChan = DeviceInfoTmp.byStartDChan;
	m_dev = dev;

	NET_DVR_CLIENTINFO ClientInfo;
	ClientInfo.hPlayWnd = NULL;
    ClientInfo.lChannel = dev.iIPChanNum;// +1;
	ClientInfo.lLinkMode = 0;
	ClientInfo.sMultiCastIP = NULL;
	int bRet = NET_DVR_RealPlay_V30(dev.lLoginID, &ClientInfo, NULL, NULL, TRUE);
    m_index = bRet;
	if (-1 == bRet)
	{
        qDebug() << "bRet error" << bRet;
		int o = 0;
	}
	else
	{

	}
}

void BJCameraCtrlSDK::CameraZoomFocusStop()
{
	NET_DVR_PTZControlWithSpeed_Other(m_dev.lLoginID, m_dev.iIPChanNum, ZOOM_IN, 1, 4);
	NET_DVR_PTZControlWithSpeed_Other(m_dev.lLoginID, m_dev.iIPChanNum, ZOOM_OUT, 1, 4);
	NET_DVR_PTZControlWithSpeed_Other(m_dev.lLoginID, m_dev.iIPChanNum, FOCUS_NEAR, 1, 4);
	NET_DVR_PTZControlWithSpeed_Other(m_dev.lLoginID, m_dev.iIPChanNum, FOCUS_FAR, 1, 4);
}

void BJCameraCtrlSDK::CameraZoomIn()
{
	NET_DVR_PTZControlWithSpeed_Other(m_dev.lLoginID, m_dev.iIPChanNum, ZOOM_IN, 0, 4);
}

void BJCameraCtrlSDK::CameraZoomOut()
{
	NET_DVR_PTZControlWithSpeed_Other(m_dev.lLoginID, m_dev.iIPChanNum, ZOOM_OUT, 0, 4);
}

void BJCameraCtrlSDK::CameraFocusFar()
{
	NET_DVR_PTZControlWithSpeed_Other(m_dev.lLoginID, m_dev.iIPChanNum, FOCUS_FAR, 0, 4);
}

void BJCameraCtrlSDK::CameraFocusNear()
{
	NET_DVR_PTZControlWithSpeed_Other(m_dev.lLoginID, m_dev.iIPChanNum, FOCUS_NEAR, 0, 4);
}

bool BJCameraCtrlSDK::getImage(std::string name)
{
	//组建jpg结构
	NET_DVR_JPEGPARA JpgPara = { 0 };
	JpgPara.wPicSize = 0xff;
	JpgPara.wPicQuality = 1;


	NET_DVR_JPEGPARA jpegPara;
	jpegPara.wPicQuality = 9;
	jpegPara.wPicSize = 0;
	name += ".jpg";

	int getPicWhileNum = 0;

	while (!NET_DVR_CaptureJPEGPicture(m_dev.lLoginID, 1, &jpegPara, (char*)name.c_str()))
	{
		getPicWhileNum++;
		if (getPicWhileNum >= 3)
		{
			DWORD err = NET_DVR_GetLastError();
			return false;
			break;
		}
		Sleep(300);
	}
	return true;
}

bool BJCameraCtrlSDK::startRecord(std::string name)
{
    if (!NET_DVR_SaveRealData_V30(m_index, STREAM_3GPP, (char *)name.c_str()))
    {
        DWORD err = NET_DVR_GetLastError();

        return false;
    }
    else
    {
        return true;
    }
}

void BJCameraCtrlSDK::stopRecord()
{
    NET_DVR_StopSaveRealData(m_index);
}

void BJCameraCtrlSDK::setZoom(DWORD zoom)
{
    NET_DVR_PTZPOS ptz;
    ptz.wAction = 4;
    ptz.wZoomPos = zoom;
    setPtzPos(ptz);
}

void BJCameraCtrlSDK::setPtzPos(NET_DVR_PTZPOS ptz)
{
    DWORD dwBufLen = 10 * 1024;
    char *pBuf = new char[dwBufLen];
    memset(pBuf, 0, dwBufLen);
    int dwXmlLen = 0;

    std::string param = "<ZoomFocus><pqrsZoom>" +
        std::to_string(ptz.wZoomPos) +
        "</pqrsZoom></ZoomFocus >";

    sprintf(pBuf, param.c_str());
    dwXmlLen = strlen(pBuf);
    char szUrl[256] = { 0 };
    sprintf(szUrl, "PUT /ISAPI/PTZCtrl/channels/1/zoomFocus\r\n");
    NET_DVR_XML_CONFIG_INPUT    struInput = { 0 };
    NET_DVR_XML_CONFIG_OUTPUT   struOuput = { 0 };
    struInput.dwSize = sizeof(struInput);
    struOuput.dwSize = sizeof(struOuput);
    struInput.lpRequestUrl = szUrl;
    struInput.dwRequestUrlLen = strlen(szUrl);
    struInput.lpInBuffer = pBuf;
    struInput.dwInBufferSize = dwXmlLen;
    if (!NET_DVR_STDXMLConfig(m_dev.lLoginID, &struInput, &struOuput))
    {
    //    ROS_ERROR("set focus failed");
    //    return false;
    }
    else
    {
    //    ROS_INFO("set focus succeed");
    //    return true;
    }
}


void BJCameraCtrlSDK::setFocus(DWORD focus)
{
    DWORD dwBufLen = 10 * 1024;
    char *pBuf = new char[dwBufLen];
    memset(pBuf, 0, dwBufLen);
    int dwXmlLen = 0;

    std::string param = "<ZoomFocus><mnstFocus>" +
        std::to_string(focus) +
        "</mnstFocus></ZoomFocus >";

    sprintf(pBuf, param.c_str());
    dwXmlLen = strlen(pBuf);
    char szUrl[256] = { 0 };
    sprintf(szUrl, "PUT /ISAPI/PTZCtrl/channels/1/zoomFocus\r\n");
    NET_DVR_XML_CONFIG_INPUT    struInput = { 0 };
    NET_DVR_XML_CONFIG_OUTPUT   struOuput = { 0 };
    struInput.dwSize = sizeof(struInput);
    struOuput.dwSize = sizeof(struOuput);
    struInput.lpRequestUrl = szUrl;
    struInput.dwRequestUrlLen = strlen(szUrl);
    struInput.lpInBuffer = pBuf;
    struInput.dwInBufferSize = dwXmlLen;
    if (!NET_DVR_STDXMLConfig(m_dev.lLoginID, &struInput, &struOuput))
    {
    //    return false;
    }
    else
    {
    //    return true;
    }
}
