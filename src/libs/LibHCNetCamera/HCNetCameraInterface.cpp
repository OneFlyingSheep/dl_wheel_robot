#include "HCNetCameraInterface.h"
#include "iostream"
#include <QThread>
#include <QDebug>
#include <QCoreApplication>
#include "LibDLHangRailConfigData/DLHangRailRobotStatusData.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <sstream>

using namespace std;

LONG nPort = -1;

void CALLBACK fVoiceTalkDataCallBack(LONG lVoiceComHandle, char *pRecvDataBuffer, DWORD dwBufSize, BYTE byAudioFlag, void *pUser)
{
    ROS_INFO("startVoiceTalk fVoiceTalkDataCallBack");
    assert(pRecvDataBuffer);
    HCNetCameraInterface *pVoiceTalkDlg = (HCNetCameraInterface*)pUser;
    if (!pVoiceTalkDlg->bIsRecordVoice)  //录音键未按下
    {
        ROS_INFO("startVoiceTalk return");
        return;
    }
        
    //0-本地数据，1-DVR发过来的数据
    if (byAudioFlag == 1)
    {
        ROS_INFO("startVoiceTalk talk s");
        ::mmioWrite(pVoiceTalkDlg->m_hmmio, pRecvDataBuffer, dwBufSize);
        ROS_INFO("startVoiceTalk talk e");
    }
}

bool yv12ToRGB888(const unsigned char *yv12, unsigned char *rgb888, int width, int height)
{
    if ((width < 1) || (height < 1) || (yv12 == nullptr) || (rgb888 == nullptr)) { return false; }

    const auto &&len = width * height;
    unsigned char const *yData = yv12;
    unsigned char const *vData = &yData[len];
    unsigned char const *uData = &vData[len >> 2];

    int rgb[3];
    int yIdx, uIdx, vIdx, idx;

    for (auto i = 0; i < height; ++i)
    {
        for (auto j = 0; j < width; ++j)
        {
            yIdx = i * width + j;
            vIdx = (i / 2) * (width / 2) + (j / 2);
            uIdx = vIdx;

            rgb[0] = static_cast<int>(yData[yIdx] + 1.370705 * (vData[uIdx] - 128));
            rgb[1] = static_cast<int>(yData[yIdx] - 0.698001 * (uData[uIdx] - 128) - 0.703125 * (vData[vIdx] - 128));
            rgb[2] = static_cast<int>(yData[yIdx] + 1.732446 * (uData[vIdx] - 128));

            for (auto k = 0; k < 3; ++k)
            {
                idx = (i * width + j) * 3 + k;

                if ((rgb[k] >= 0) && (rgb[k] <= 255))
                {
                    rgb888[idx] = static_cast<unsigned char>(rgb[k]);
                }
                else
                {
                    rgb888[idx] = (rgb[k] < 0) ? (0) : (255);
                }
            }
        }
    }
    return true;
}

//解码回调 视频为YUV数据(YV12)，音频为PCM数据
void CALLBACK DecCBFun(long nPort, char* pBuf, long nSize, FRAME_INFO* pFrameInfo, long nUser, long nReserved2)
{
    DWORD start = GetTickCount();
    long lFrameType = pFrameInfo->nType;
    HCNetCameraInterface *p = (HCNetCameraInterface*)(void*)nUser;
    if (lFrameType == T_YV12 && p != NULL)
    {
        //p->startThread(pBuf, nSize, pFrameInfo->nWidth, pFrameInfo->nHeight);
        //QByteArray imgBuf(pBuf, nSize);
        //
        //QSize imgSize(pFrameInfo->nWidth, pFrameInfo->nHeight);

        //cv::Mat yuvImg(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (unsigned char*)pBuf);
        //cv::Mat rgbImg;
        //cv::cvtColor(yuvImg, rgbImg, cv::COLOR_YUV2BGR_YV12);
        //imshow("hc", rgbImg);
        //cv::waitKey(15);
        ////QImage img = QImage((uchar*)rgbImg.data, rgbImg.cols, rgbImg.rows, QImage::Format_RGB30);
        ////img.bits();
        ////QPixmap map = QPixmap::fromImage(img);

        //emit p->sigVisibleNewFrame(imgBuf, imgSize);
    }
    DWORD end = GetTickCount();
    ROS_INFO("time:%d", end - start);
}

///实时流回调
void CALLBACK fRealDataCallBack(LONG lRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* pUser)
{
    DWORD dRet;
    switch (dwDataType)
    {
    case NET_DVR_SYSHEAD:    //系统头
        if (!PlayM4_GetPort(&nPort)) //获取播放库未使用的通道号
        {
            break;
        }
        if (dwBufSize > 0)
        {
            if (!PlayM4_OpenStream(nPort, pBuffer, dwBufSize, 1024 * 1024))
            {
                dRet = PlayM4_GetLastError(nPort);
                break;
            }

            //设置解码回调函数 解码且显示
            if (!PlayM4_SetDecCallBackExMend(nPort, NULL, NULL, NULL, reinterpret_cast<long>(pUser)))
            {
                dRet = PlayM4_GetLastError(nPort);
                break;
            }

            if (!PlayM4_Play(nPort, NULL))
            {
                dRet = PlayM4_GetLastError(nPort);
                break;
            }
        }
        break;

    case NET_DVR_STREAMDATA:   //码流数据
        if (dwBufSize > 0 && nPort != -1)
        {
            BOOL inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
            while (!inData)
            {
                Sleep(10);
                inData = PlayM4_InputData(nPort, pBuffer, dwBufSize);
                //OutputDebugString(L"PlayM4_InputData failed \n");
            }
        }
        break;
    }
}

HCNetCameraInterface::HCNetCameraInterface(char *camIp, char *username, char *password, int portNum /*= 8000*/)
{
    m_thread = new boost::thread(boost::bind(&HCNetCameraInterface::getCameraZF, this));
    memset(m_cameraIp, 0, 32);
    memcpy(m_cameraIp, camIp, 32);
    memset(m_name, 0, 32);
    memcpy(m_name, username, 32);
    memset(m_password, 0, 32);
    memcpy(m_password, password, 32);
    m_port = portNum;
    bRecording = false;
    init();
}

HCNetCameraInterface::HCNetCameraInterface(QString camIp, QString userName, QString passwd, int portNum /*= 8000*/)
{
	m_thread = new boost::thread(boost::bind(&HCNetCameraInterface::getCameraZF, this));
    memset(m_cameraIp, 0, 32);
    memcpy(m_cameraIp, camIp.toStdString().c_str(), 32);
    memset(m_name, 0, 32);
    memcpy(m_name, userName.toStdString().c_str(), 32);
    memset(m_password, 0, 32);
    memcpy(m_password, passwd.toStdString().c_str(), 32);
    m_port = portNum;
    bRecording = false;
    init();
}

HCNetCameraInterface::~HCNetCameraInterface()
{
    disconnect();
    release();
}

void HCNetCameraInterface::getCameraZF()
{
	while (1)
	{
		getZoomFocus(m_zoomValue, m_focusValue);
		Sleep(1000);
	}
}

bool HCNetCameraInterface::connect()
{
    if(getLoginState())
        return true;

    if(Login(m_cameraIp, m_name, m_password, m_port))
    {
        return true;
    }
    return false;
}

bool HCNetCameraInterface::disconnect()
{
    if(getPlayingState())
    {
        stopPlay();
    }

    if(getLoginState())
    {
        LOCAL_DEVICE_INFO dev = getLocalDevice();
        NET_DVR_Logout_V30(dev.lLoginID);
        dev.lLoginID = -1;
        setLocalDevice(dev);
        setLoginState(false);
        //release();
        return true;
    }
}

bool HCNetCameraInterface::initCamera(int index)
{
    setCurrentChannel(index);
    GetDeviceResoureCfg();
    GetDecoderCfg();
    return true;
}

bool HCNetCameraInterface::startPlay(int index, HWND hwd)
{
    NET_DVR_CLIENTINFO ClientInfo;
    ClientInfo.hPlayWnd     = hwd;
    ClientInfo.lChannel     = index;
    ClientInfo.lLinkMode    = 0;
    ClientInfo.sMultiCastIP = NULL;

    LOCAL_DEVICE_INFO dev;
    dev = getLocalDevice();

    setRealplayHandle(NET_DVR_RealPlay_V30(dev.lLoginID,&ClientInfo,NULL,NULL,TRUE));
    if(-1 == getRealplayHandle())
    {
        DWORD err = NET_DVR_GetLastError();
        return false;
    }
    else
    {
        setPlayingState(true);
        return true;
    }
}

bool HCNetCameraInterface::stopPlay()
{
    if (getPlayingState() != -1)
    {
        NET_DVR_StopRealPlay(getRealplayHandle());
        setRealplayHandle(-1);
        setPlayingState(false);
        setCurrentChannel(-1);
    }
    return true;
}

bool HCNetCameraInterface::setHCNetDvrConfig()
{
	// 初始化语音对讲控件;
// 	int chann = getCurrentChannel();
// 
// 	NET_DVR_COMPRESSION_AUDIO cfg;
// 	cfg.byAudioEncType = 0;
// 	cfg.byAudioSamplingRate = 1;//音频采样率 0-默认，1-16kHZ，2-32kHZ，3-48kHZ, 4- 44.1kHZ,5-8kHZ
// 	cfg.byAudioBitRate = 16;// 音频码率 参考 BITRATE_ENCODE_INDEX
// 	long ds = NET_DVR_SetDVRConfig(getLocalDevice().lLoginID, NET_DVR_SET_COMPRESSCFG_AUD, chann, &cfg, sizeof(NET_DVR_COMPRESSION_AUDIO));
	return true;
}

bool HCNetCameraInterface::startVoiceTalk()
{
	int chann = getCurrentChannel();
	LOCAL_DEVICE_INFO dev;
	dev = getLocalDevice();
    ROS_INFO("startVoiceTalk :start");
//    m_lVoiceHandle = NET_DVR_StartVoiceCom_MR_V30(dev.lLoginID, getCurrentChannel(), fVoiceTalkDataCallBack, (void*)this);
    try
    {
        ROS_INFO("startVoiceTalk :dev.lLoginID:%d   chann:%d", dev.lLoginID, chann);
        m_lVoiceHandle = NET_DVR_StartVoiceCom_V30(dev.lLoginID, getCurrentChannel(), chann, fVoiceTalkDataCallBack, (void*)this);
    }
    catch (std::exception&) 
    {
    }
    
    if (-1 == m_lVoiceHandle)
    {
        DWORD err = NET_DVR_GetLastError();
        ROS_INFO("Voice talk failed! err:%d", err);
        return false;
    }
    else
    {
        ROS_INFO("Voice talk succeed!");
        return true;
    }
}

bool HCNetCameraInterface::stopVoiceTalk()
{
    NET_DVR_StopVoiceCom(m_lVoiceHandle);
    return true;
}

bool HCNetCameraInterface::startRecord(string name)
{
    //if (!NET_DVR_SaveRealData_V30(getRealplayHandle(), STREAM_3GPP, (char *)name.c_str()))
    if (!NET_DVR_SaveRealData(getRealplayHandle(), (char *)name.c_str()))
    {
//     }
// 
//     if (!NET_DVR_SaveRealData(getRealplayHandle(), (char *)name.c_str()))
//     {
        DWORD err = NET_DVR_GetLastError();
        ROS_ERROR("启动录像失败，error:%d", err);
        return false;
    }
    else
    {
        ROS_INFO("开始录像：%s", name.c_str());
        return true;
    }
}

void HCNetCameraInterface::stopRecord()
{
        NET_DVR_StopSaveRealData(getRealplayHandle());
}

bool HCNetCameraInterface::startRecordAudio(string audioPath)
{
    m_struWaveFormat.cbSize = sizeof(m_struWaveFormat);
    m_struWaveFormat.nBlockAlign = CHANNEL * BITS_PER_SAMPLE / 8;
    m_struWaveFormat.nChannels = CHANNEL;
    m_struWaveFormat.nSamplesPerSec = SAMPLES_PER_SECOND;
    m_struWaveFormat.wBitsPerSample = BITS_PER_SAMPLE;
    m_struWaveFormat.nAvgBytesPerSec = SAMPLES_PER_SECOND * m_struWaveFormat.nBlockAlign;
    m_struWaveFormat.wFormatTag = WAVE_FORMAT_PCM;

    audioPath += ".wav";

    if (!(m_hmmio = mmioOpen((char*)audioPath.c_str(), NULL, MMIO_CREATE | MMIO_WRITE | MMIO_EXCLUSIVE)))
    {
        //printf("opem file error\n");
        return FALSE;
    }

    ZeroMemory(&m_mmckinfoParent, sizeof(MMCKINFO));
    m_mmckinfoParent.fccType = mmioFOURCC('W', 'A', 'V', 'E');
    MMRESULT mmResult = ::mmioCreateChunk(m_hmmio, &m_mmckinfoParent, MMIO_CREATERIFF);

    ZeroMemory(&m_mmckinfoChild, sizeof(MMCKINFO));
    m_mmckinfoChild.ckid = mmioFOURCC('f', 'm', 't', ' ');
    m_mmckinfoChild.cksize = sizeof(WAVEFORMATEX) + m_struWaveFormat.cbSize;
    mmResult = ::mmioCreateChunk(m_hmmio, &m_mmckinfoChild, 0);

    mmResult = ::mmioWrite(m_hmmio, (char*)&m_struWaveFormat, sizeof(WAVEFORMATEX) + m_struWaveFormat.cbSize);
    mmResult = ::mmioAscend(m_hmmio, &m_mmckinfoChild, 0);
    m_mmckinfoChild.ckid = mmioFOURCC('d', 'a', 't', 'a');
    mmResult = ::mmioCreateChunk(m_hmmio, &m_mmckinfoChild, 0);
    bIsRecordVoice = true;
    return TRUE;
}

bool HCNetCameraInterface::stopRecordAudio()
{
    if (m_hmmio)
    {
        bIsRecordVoice = false;
        ::mmioAscend(m_hmmio, &m_mmckinfoChild, 0);
        ::mmioAscend(m_hmmio, &m_mmckinfoParent, 0);
        ::mmioClose(m_hmmio, 0);
        m_hmmio = NULL;
    }
    return TRUE;
}

bool HCNetCameraInterface::startCallback(int index)
{
    NET_DVR_PREVIEWINFO previewInfo = { 0 };
    previewInfo.hPlayWnd = NULL;
    previewInfo.lChannel = index;
    previewInfo.dwStreamType = 0;
    previewInfo.dwLinkMode = 0;
    //NET_DVR_CLIENTINFO ClientInfo;
    //ClientInfo.hPlayWnd = NULL;
    //ClientInfo.lChannel = index;
    //ClientInfo.lLinkMode = 0;
    //ClientInfo.sMultiCastIP = NULL;

    LOCAL_DEVICE_INFO dev;
    dev = getLocalDevice();

    setRealplayHandle(NET_DVR_RealPlay_V40(dev.lLoginID, &previewInfo, fRealDataCallBack, this));
    if (-1 == getRealplayHandle())
    {
        DWORD err = NET_DVR_GetLastError();
        return false;
    }
    else
    {
        setPlayingState(true);
        setCurrentChannel(index);
        GetDeviceResoureCfg();
        GetDecoderCfg();
        return true;
    }
}

bool HCNetCameraInterface::getImage(string name)
{
    if (getRealplayHandle() == -1)
    {
        ROS_INFO("GetImage:2021 getRealplayHandle error!");
        return false;
    }

    //组建jpg结构
    NET_DVR_JPEGPARA JpgPara = {0};
    JpgPara.wPicSize = 0xff;
    JpgPara.wPicQuality = 1;
    if (getCurrentChannel() < 0)
    {
        ROS_INFO("GetImage:2021 getCurrentChannel error!");
        return false;
    }

    NET_DVR_JPEGPARA jpegPara;
    jpegPara.wPicQuality = 0;
    jpegPara.wPicSize = 0;
    name += ".jpg";
	int getPicWhileNum = 0;

// 	while (!NET_DVR_CaptureJPEGPicture(getLocalDevice().lLoginID, getCurrentChannel(), &jpegPara, (char*)name.c_str()))
// 	{
// 		getPicWhileNum++;
// 		if (getPicWhileNum >= 3)
// 		{
// 			DWORD err = NET_DVR_GetLastError();
// 			cout << err << endl;
// 			return false;
// 			break;
// 		}
// 		Sleep(300);
// 	}

    while (!NET_DVR_CapturePicture(getLocalDevice().lLoginID, (char*)name.c_str()))
    {
        getPicWhileNum++;
        DWORD err = NET_DVR_GetLastError();
        cout << err << endl;
        ROS_INFO("GetImage:2021 take photo error! %d %d", getPicWhileNum, err);
        if (getPicWhileNum >= 3)
        {
            return false;
            break;
        }
        Sleep(300);
    }

//     NET_DVR_SetCapturePictureMode(0);
// 	if (NET_DVR_CapturePicture(getLocalDevice().lLoginID, (char*)name.c_str()))
// 	{
// 		return true;
// 	}
// 	else
// 	{
// 		DWORD err = NET_DVR_GetLastError();
// 		cout << err << endl;
// 		return false;
// 	}

    return true;
}

bool HCNetCameraInterface::setZoom(DWORD zoom)
{
    NET_DVR_PTZPOS ptz;
    ptz.wAction = 4;
    ptz.wZoomPos = zoom;
    setPtzPos(ptz);
    return true;
}

bool HCNetCameraInterface::setPtzPos(NET_DVR_PTZPOS ptz)
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
	if (!NET_DVR_STDXMLConfig(getLocalDevice().lLoginID, &struInput, &struOuput))
	{
		ROS_ERROR("set focus failed");
		return false;
	}
	else
	{
		ROS_INFO("set focus succeed");
		return true;
	}
//    return NET_DVR_SetDVRConfig(getLocalDevice().lLoginID, NET_DVR_SET_PTZPOS, getCurrentChannel(), &ptz, sizeof(NET_DVR_PTZPOS));
}

NET_DVR_PTZPOS  HCNetCameraInterface::getPtzPos()
{
    BOOL ret = 0;
    NET_DVR_PTZPOS ptz;
    memset(&ptz, 0, sizeof(NET_DVR_PTZPOS));
//     DWORD dRet = 0;
//     ret = NET_DVR_GetDVRConfig(getLocalDevice().lLoginID, NET_DVR_GET_PTZPOS, getCurrentChannel(), &ptz, sizeof(NET_DVR_PTZPOS), &dRet);

	NET_DVR_XML_CONFIG_INPUT    struInput = { 0 };
    NET_DVR_XML_CONFIG_OUTPUT   struOuput = { 0 };
    struInput.dwSize = sizeof(struInput);
    struOuput.dwSize = sizeof(struOuput);
    char szUrl[256] = { 0 };
    sprintf(szUrl, "GET /ISAPI/PTZCtrl/channels/1/zoomFocus\r\n");

    struInput.lpRequestUrl = szUrl;
    struInput.dwRequestUrlLen = strlen(szUrl);
    DWORD dwOutputLen = 512;
    char *pOutBuf = new char[dwOutputLen];
    memset(pOutBuf, 0, dwOutputLen);
    struOuput.lpOutBuffer = pOutBuf;
    struOuput.dwOutBufferSize = dwOutputLen;

    if (!NET_DVR_STDXMLConfig(getLocalDevice().lLoginID, &struInput, &struOuput))
    {
        ROS_ERROR("Get Focus Pose Failed, ERRNO: %u", NET_DVR_GetLastError());
        delete[]pOutBuf;
        pOutBuf = NULL;
 //       return 0;
    }
    else
    {
        boost::property_tree::ptree pt, focusDirectPt,focusPt;
        stringstream ss;
        ss << pOutBuf << "\0";

        boost::property_tree::xml_parser::read_xml(ss, pt);
        focusDirectPt = pt.get_child("ZoomFocus");
        focusPt = focusDirectPt.get_child("pqrsZoom");

        std::string strFocus = focusPt.data();
		delete[]pOutBuf;
		pOutBuf = NULL;
		ptz.wZoomPos = atoi(strFocus.c_str());
    }

    return ptz;
}

bool HCNetCameraInterface::setFocus(DWORD focus)
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
    if (!NET_DVR_STDXMLConfig(getLocalDevice().lLoginID, &struInput, &struOuput))
    {
        ROS_ERROR("set focus failed");
        return false;
    }
    else
    {
        ROS_INFO("set focus succeed");
        return true;
    }
}

DWORD HCNetCameraInterface::getFocus()
{
    NET_DVR_XML_CONFIG_INPUT    struInput = { 0 };
    NET_DVR_XML_CONFIG_OUTPUT   struOuput = { 0 };
    struInput.dwSize = sizeof(struInput);
    struOuput.dwSize = sizeof(struOuput);
    char szUrl[256] = { 0 };
    sprintf(szUrl, "GET /ISAPI/PTZCtrl/channels/1/zoomFocus\r\n");

    struInput.lpRequestUrl = szUrl;
    struInput.dwRequestUrlLen = strlen(szUrl);
    DWORD dwOutputLen = 512;
    char *pOutBuf = new char[dwOutputLen];
    memset(pOutBuf, 0, dwOutputLen);
    struOuput.lpOutBuffer = pOutBuf;
    struOuput.dwOutBufferSize = dwOutputLen;

    if (!NET_DVR_STDXMLConfig(getLocalDevice().lLoginID, &struInput, &struOuput))
    {
        ROS_ERROR("Get Focus Pose Failed, ERRNO: %u", NET_DVR_GetLastError());
        delete[]pOutBuf;
        pOutBuf = NULL;
        return 0;
    }
    else
    {
        boost::property_tree::ptree pt, focusDirectPt,focusPt;
        stringstream ss;
        ss << pOutBuf << "\0";

        boost::property_tree::xml_parser::read_xml(ss, pt);
        focusDirectPt = pt.get_child("ZoomFocus");
        focusPt = focusDirectPt.get_child("mnstFocus");

        std::string strFocus = focusPt.data();
		delete[]pOutBuf;
		pOutBuf = NULL;
        return atoi(strFocus.c_str());
    }
}

bool HCNetCameraInterface::setZoomAndFocus(DWORD zoom, DWORD focus)
{
    DWORD dwBufLen = 10 * 1024;
    char *pBuf = new char[dwBufLen];
    memset(pBuf, 0, dwBufLen);
    int dwXmlLen = 0;

    std::string param = "<ZoomFocus><pqrsZoom>" +
        std::to_string(zoom) +
        "</pqrsZoom>" + "<mnstFocus>" +
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
    if (!NET_DVR_STDXMLConfig(getLocalDevice().lLoginID, &struInput, &struOuput))
    {
        ROS_ERROR("set focus failed");
        return false;
    }
    else
    {
        ROS_INFO("set focus succeed");
        return true;
    }
}

void HCNetCameraInterface::getZoomFocus(int &zoomValue, int &focusValue)
{
	NET_DVR_XML_CONFIG_INPUT    struInput = { 0 };
	NET_DVR_XML_CONFIG_OUTPUT   struOuput = { 0 };
	struInput.dwSize = sizeof(struInput);
	struOuput.dwSize = sizeof(struOuput);
	char szUrl[256] = { 0 };
	sprintf(szUrl, "GET /ISAPI/PTZCtrl/channels/1/zoomFocus\r\n");

	struInput.lpRequestUrl = szUrl;
	struInput.dwRequestUrlLen = strlen(szUrl);
	DWORD dwOutputLen = 512;
	char *pOutBuf = new char[dwOutputLen];
	memset(pOutBuf, 0, dwOutputLen);
	struOuput.lpOutBuffer = pOutBuf;
	struOuput.dwOutBufferSize = dwOutputLen;

	if (!NET_DVR_STDXMLConfig(getLocalDevice().lLoginID, &struInput, &struOuput))
	{
		ROS_ERROR("Get Focus Pose Failed, ERRNO: %u", NET_DVR_GetLastError());
		delete[]pOutBuf;
		pOutBuf = NULL;
		return;
	}
	else
	{
		boost::property_tree::ptree pt, focusDirectPt, zoomPt, focusPt;
		stringstream ss;
		ss << pOutBuf << "\0";

		boost::property_tree::xml_parser::read_xml(ss, pt);
		focusDirectPt = pt.get_child("ZoomFocus");
		zoomPt = focusDirectPt.get_child("pqrsZoom");
		focusPt = focusDirectPt.get_child("mnstFocus");

		std::string strFocus = focusPt.data();
		std::string strZoom = zoomPt.data();
		delete[]pOutBuf;
		pOutBuf = NULL;
		zoomValue = atoi(strZoom.c_str());
		focusValue = atoi(strFocus.c_str());
	}
}

void HCNetCameraInterface::ZoomIn()
{
	NET_DVR_PTZControlWithSpeed_Other(getLocalDevice().lLoginID, getCurrentChannel(), ZOOM_IN, 0, 4);
}

void HCNetCameraInterface::ZoomOut()
{
    NET_DVR_PTZControlWithSpeed_Other(getLocalDevice().lLoginID, getCurrentChannel(), ZOOM_OUT, 0, 4);
}

void HCNetCameraInterface::ZoomOutStop()
{
    NET_DVR_PTZControlWithSpeed_Other(getLocalDevice().lLoginID, getCurrentChannel(), ZOOM_OUT, 1, 4);
}

void HCNetCameraInterface::ZoomInStop()
{
	NET_DVR_PTZControlWithSpeed_Other(getLocalDevice().lLoginID, getCurrentChannel(), ZOOM_IN, 1, 4);
}

void HCNetCameraInterface::FocusNear()
{
    NET_DVR_PTZControlWithSpeed_Other(getLocalDevice().lLoginID, getCurrentChannel(), FOCUS_NEAR, 0, 4);
}

void HCNetCameraInterface::FocusFar()
{
    NET_DVR_PTZControlWithSpeed_Other(getLocalDevice().lLoginID, getCurrentChannel(), FOCUS_FAR, 0, 4);
}

void HCNetCameraInterface::FocusStop()
{
    NET_DVR_PTZControlWithSpeed_Other(getLocalDevice().lLoginID, getCurrentChannel(), FOCUS_NEAR, 1, 4);
    NET_DVR_PTZControlWithSpeed_Other(getLocalDevice().lLoginID, getCurrentChannel(), FOCUS_FAR, 1, 4);
}

void HCNetCameraInterface::ZoomAndFocusStop()
{
    FocusStop();
    ZoomInStop();
    ZoomOutStop();
}

void HCNetCameraInterface::OnVisibleZoomInClickedSlot(bool b)
{
	if (b)
	{
		this->ZoomIn();
	}
	else
	{
		this->ZoomInStop();
	}
}

void HCNetCameraInterface::OnVisibleZoomOutClickedSlot(bool b)
{
	if (b)
	{
		this->ZoomOut();
	}
	else
	{
		this->ZoomOutStop();
	}
}

void HCNetCameraInterface::OnVisibleFocusNearClickedSlot(bool b)
{
	if (b)
	{
		this->FocusNear();
	}
	else
	{
		this->FocusStop();
	}
}

void HCNetCameraInterface::OnVisibleFocusFarClickedSlot(bool b)
{
	if (b)
	{
		this->FocusFar();
	}
	else
	{
		this->FocusStop();
	}
}

void HCNetCameraInterface::onGetCameraZoomValue()
{
	NET_DVR_PTZPOS ptzPos = this->getPtzPos();
	ROBOTSTATUS.setRobotStatusPtzZoomValue(ptzPos.wZoomPos);
}

void HCNetCameraInterface::onGetCameraFocusValue()
{
    DWORD ptzPos = this->getFocus();
    ROBOTSTATUS.setRobotStatusFocusValue(ptzPos);
}

void HCNetCameraInterface::onGetCameraImage(QString fileName)
{
	QByteArray byteData = fileName.toLocal8Bit();
	this->getImage(byteData.data());
}

bool HCNetCameraInterface::onGetCameraImageBool(QString fileName)
{
    QByteArray byteData = fileName.toLocal8Bit();
    return this->getImage(byteData.data());
}

void HCNetCameraInterface::onRecordCameraAudio(bool isStartRecord, QString fileName /* = "" */)
{
	if (isStartRecord)
	{
		QByteArray byteData = fileName.toLocal8Bit();
		this->startRecordAudio(fileName.toStdString());
	}
	else
	{
        this->stopRecordAudio();
	}
}
