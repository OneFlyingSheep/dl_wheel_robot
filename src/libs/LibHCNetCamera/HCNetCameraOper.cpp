#include "HCNetCameraOper.h"
#include "iostream"
using namespace std;

void CALLBACK g_ExceptionCallBack(DWORD dwType, LONG lUserID, LONG lHandle, void *pUser)
{
    HCNetCameraOper *p = (HCNetCameraOper *)pUser;
    char tempbuf[256] = { 0 };
    switch (dwType)
    {
    case EXCEPTION_RECONNECT:    //预览时重连
        ROS_INFO("----------reconnect--------%d\n", time(NULL));
        break;
    case EXCEPTION_PREVIEW:
        p->stopPlay();
        p->disconnect();
        //p->release();
        break;
    default:
        break;
    }
}

void CALLBACK slBack(LONG lUserID, DWORD dwType, LPNET_DVR_DEVICEINFO_V30 info, void *pUser)
{
}

HCNetCameraOper::HCNetCameraOper()
{
    m_iPtzSpeed = 4;
    m_bIsLogin = false;
    m_bIsPlaying = false;

    m_lPlayHandle = 0;
    m_iCurChanIndex = 0;
    memset(&m_struDeviceInfo, 0, sizeof(LOCAL_DEVICE_INFO));
}

HCNetCameraOper::~HCNetCameraOper()
{
    release();
}

bool HCNetCameraOper::init()
{
    NET_DVR_Init();
    NET_DVR_SetConnectTime(2000, 1);
    NET_DVR_SetReconnect(10000, true);
    NET_DVR_SetExceptionCallBack_V30(0, NULL, g_ExceptionCallBack, this);
    return true;
}

bool HCNetCameraOper::release()
{
    return NET_DVR_Cleanup();
}

bool HCNetCameraOper::Login(const char *ip, const char *name, const char *passwd, unsigned short port)
{
    NET_DVR_DEVICEINFO_V30 DeviceInfoTmp;
    memset(&DeviceInfoTmp,0,sizeof(NET_DVR_DEVICEINFO_V30));

    LONG lLoginID = NET_DVR_Login_V30((char*)ip, port, (char*)name, (char*)passwd, &DeviceInfoTmp);

// 	NET_DVR_USER_LOGIN_INFO v40;
// 	memset(&v40, 0, sizeof(NET_DVR_USER_LOGIN_INFO));
// 	v40.wPort = port;
// 	strcpy(v40.sDeviceAddress, ip);
// 	strcpy(v40.sUserName, name);
// 	strcpy(v40.sPassword, passwd);
// 	v40.cbLoginResult = slBack;
// 	NET_DVR_DEVICEINFO_V40 dsgg;
// 	LONG lLoginID = NET_DVR_Login_V40(&v40, &dsgg);
// 	DeviceInfoTmp = dsgg.struDeviceV30;


    if(lLoginID == -1)
    {
        DWORD err = NET_DVR_GetLastError();
        std::cout<< err << endl;
        return false;
    }
    LOCAL_DEVICE_INFO dev;
    dev.lLoginID = lLoginID;
    dev.iDeviceChanNum = DeviceInfoTmp.byChanNum;
    dev.iIPChanNum = DeviceInfoTmp.byIPChanNum;
    dev.iStartChan  = DeviceInfoTmp.byStartChan;
    dev.iIPStartChan = DeviceInfoTmp.byStartDChan;
    setLocalDevice(dev);
    setLoginState(true);
    return true;
}

void HCNetCameraOper::GetDeviceResoureCfg()
{
    NET_DVR_IPPARACFG_V40 IpAccessCfg;
    memset(&IpAccessCfg,0,sizeof(IpAccessCfg));
    DWORD  dwReturned;

    m_struDeviceInfo.bIPRet = NET_DVR_GetDVRConfig(m_struDeviceInfo.lLoginID,NET_DVR_GET_IPPARACFG_V40,0,&IpAccessCfg,sizeof(NET_DVR_IPPARACFG_V40),&dwReturned);

    int i;
    if(!m_struDeviceInfo.bIPRet)
    {
        for(i=0; i<MAX_ANALOG_CHANNUM; i++)
        {
            if (i < m_struDeviceInfo.iDeviceChanNum)
            {
                sprintf(m_struDeviceInfo.struChanInfo[i].chChanName,"camera%d",i+m_struDeviceInfo.iStartChan);
                m_struDeviceInfo.struChanInfo[i].iChanIndex=i+m_struDeviceInfo.iStartChan;
                m_struDeviceInfo.struChanInfo[i].bEnable = TRUE;

            }
            else
            {
                m_struDeviceInfo.struChanInfo[i].iChanIndex = -1;
                m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
                sprintf(m_struDeviceInfo.struChanInfo[i].chChanName, "");
            }
        }
    }
    else
    {
        for(i=0; i<MAX_ANALOG_CHANNUM; i++)
        {
            if (i < m_struDeviceInfo.iDeviceChanNum)
            {
                sprintf(m_struDeviceInfo.struChanInfo[i].chChanName,"camera%d",i+m_struDeviceInfo.iStartChan);
                m_struDeviceInfo.struChanInfo[i].iChanIndex=i+m_struDeviceInfo.iStartChan;
                if(IpAccessCfg.byAnalogChanEnable[i])
                {
                    m_struDeviceInfo.struChanInfo[i].bEnable = TRUE;
                }
                else
                {
                    m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
                }

            }
            else//clear the state of other channel
            {
                m_struDeviceInfo.struChanInfo[i].iChanIndex = -1;
                m_struDeviceInfo.struChanInfo[i].bEnable = FALSE;
                sprintf(m_struDeviceInfo.struChanInfo[i].chChanName, "");
            }
        }

        for(i=0; i<MAX_IP_CHANNEL; i++)
        {
            if(IpAccessCfg.struStreamMode[i].uGetStream.struChanInfo.byEnable)
            {
                m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].bEnable = TRUE;
                m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].iChanIndex = i+IpAccessCfg.dwStartDChan;
                sprintf(m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].chChanName,"IP Camera %d",i+1);

            }
            else
            {
                m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].bEnable = FALSE;
                m_struDeviceInfo.struChanInfo[i+MAX_ANALOG_CHANNUM].iChanIndex = -1;
            }
        }
    }
}

void HCNetCameraOper::GetDecoderCfg()
{
    NET_DVR_DECODERCFG_V30 DecoderCfg;
    DWORD  dwReturned;
    BOOL bRet;

    for(int i=0; i<MAX_CHANNUM_V30; i++)
    {
        if(m_struDeviceInfo.struChanInfo[i].bEnable)
        {
            memset(&DecoderCfg,0,sizeof(NET_DVR_DECODERCFG_V30));
            bRet = NET_DVR_GetDVRConfig(m_struDeviceInfo.lLoginID,NET_DVR_GET_DECODERCFG_V30 , \
                                        m_struDeviceInfo.struChanInfo[i].iChanIndex,&DecoderCfg,sizeof(NET_DVR_DECODERCFG_V30),&dwReturned);
            if(!bRet)
            {
//                char err[256];
//                sprintf(err,"Failed to Get PTZ encoder, Channel:%d",m_struDeviceInfo.struChanInfo[i].iChanIndex);
//                ROS_ERROR(err);
                continue;
            }
            memcpy(&m_struDeviceInfo.struChanInfo[i].struDecodercfg,&DecoderCfg,sizeof(NET_DVR_DECODERCFG_V30));
        }
    }
}

void HCNetCameraOper::setPtzSpeed(int speed)
{
    m_iPtzSpeed = speed;
}

int HCNetCameraOper::getPtzSpeed()
{
    return m_iPtzSpeed;
}

void HCNetCameraOper::setLoginState(bool flag)
{
    m_bIsLogin = flag;
}

bool HCNetCameraOper::getLoginState()
{
    return m_bIsLogin;
}

void HCNetCameraOper::setPlayingState(bool flag)
{
    m_bIsPlaying = flag;
}

bool HCNetCameraOper::getPlayingState()
{
    return m_bIsPlaying;
}

void HCNetCameraOper::setRealplayHandle(long handle)
{
    m_lPlayHandle = handle;
}

long HCNetCameraOper::getRealplayHandle()
{
    return m_lPlayHandle;
}

void HCNetCameraOper::setCurrentChannel(int channel)
{
    m_iCurChanIndex = channel;
}

long HCNetCameraOper::getCurrentChannel()
{
    return m_iCurChanIndex;
}

void HCNetCameraOper::setLocalDevice(LOCAL_DEVICE_INFO info)
{
    memcpy(&m_struDeviceInfo, &info, sizeof(LOCAL_DEVICE_INFO));
}

LOCAL_DEVICE_INFO HCNetCameraOper::getLocalDevice()
{
    return m_struDeviceInfo;
}
