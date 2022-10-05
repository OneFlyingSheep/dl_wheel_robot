#include "JuGeNewInfrared.h"
#include <QDebug>
#include "common/DLRobotCommonDef.h"
#define INFRARED_MAX_CHANNEL 128

JuGeNewInfrared::JuGeNewInfrared()
{
    while (1)
    {
        if (MAG_IsChannelAvailable(m_iChannelIndex))
        {
            m_iChannelIndex++;
        }
        else
        {
            if (MAG_NewChannel(m_iChannelIndex))
            {
                m_bInitChannel = true;
                ROS_INFO("JuGeNewInfrared::MAG_NewChannel true");
                break;
            }
            else
            {
                m_bInitChannel = false;
                ROS_INFO("JuGeNewInfrared::MAG_NewChannel false");
                break;
            }
        }
        if (m_iChannelIndex > INFRARED_MAX_CHANNEL)
        {
            m_bInitChannel = false;
            break;
        }
    }
}

JuGeNewInfrared::~JuGeNewInfrared()
{
    MAG_DelChannel(m_iChannelIndex);
}

void JuGeNewInfrared::getInfraredRectTemp(QString filePath, CoorInfrared coor, ImageInfraredTemp &reData)
{
    ROS_INFO("JuGeNewInfrared::m_iChannelIndex %d!", m_iChannelIndex);
    OutputPara paraOut;
    paraOut.dwFPAWidth = 640;
    paraOut.dwFPAHeight = 480;
    paraOut.dwBMPWidth = 640;
    paraOut.dwBMPHeight = 480;
    paraOut.dwColorBarWidth = 20;
    paraOut.dwColorBarHeight = 100;
    ROS_INFO("JuGeNewInfrared::getInfraredRectTemp filePath: %s", filePath.toStdString().data());

    WCHAR* charFilename;
    QString str = filePath.replace("-temp.jpg", ".ddt");
    QByteArray ba = str.toLocal8Bit();
    char *s = ba.data();
    int w_nlen = MultiByteToWideChar(CP_ACP, 0, s, -1, NULL, 0);
    WCHAR *ret;
    ret = (WCHAR*)malloc(sizeof(WCHAR)*w_nlen);
    memset(ret, 0, sizeof(ret));
    MultiByteToWideChar(CP_ACP, 0, s, -1, ret, w_nlen);
    charFilename = ret;

    int x0 = coor.coor_x;
    int y0 = 480 - coor.coor_y;
    int x1 = coor.coor_x + coor.coor_width;
    int y1 = 480 - (coor.coor_y + coor.coor_height);
    ROS_INFO("JuGeNewInfrared::getInfraredRectTemp filePath: %s", filePath.toStdString().data());
    ROS_INFO("JuGeNewInfrared::getInfraredRectTemp str: %s", str.toStdString().data());


    if (!MAG_LoadDDT(m_iChannelIndex, &paraOut, charFilename, NULL, 0))
    {
        ROS_INFO("JuGeNewInfrared::MAG_LoadDDT!");

        reData.temp = -1;
        reData.y = -1;
        reData.x = -1;

        reData.temp_min = -1;
        reData.y_min = -1;
        reData.x_min = -1;
        return;
    }
    int info[5] = { 0 };
    if (!MAG_GetRectTemperatureInfo(m_iChannelIndex, x0, y1, x1, y0, info))
    {
        ROS_INFO("JuGeNewInfrared::MAG_GetRectTemperatureInfo!");
        ROS_INFO("JuGeNewInfrared::MAG_GetRectTemperatureInfo!x:%d,y:%d,x:%d,y%d", x0, y0, x1, y1);
        reData.temp = -1;
        reData.y = -1;
        reData.x = -1;

        reData.temp_min = -1;
        reData.y_min = -1;
        reData.x_min = -1;
        return;
    }
    ROS_INFO("JuGeNewInfrared::ye!");
    reData.temp = info[1] / 1000.0;
    reData.y = info[4] / 640;
    reData.x = info[4] - reData.y * 640;

    reData.temp_min = info[0] / 1000.0;
    reData.y_min = info[3] / 640;
    reData.x_min = info[3] - reData.y * 640;
}

void JuGeNewInfrared::getInfraredPointTemp(QString filePath, InfraredPointBase pointBase, float &val)
{
    int temperature = MAG_GetTemperatureProbe(m_iChannelIndex, pointBase.locate_x, 480 - pointBase.locate_y, 1);
    val = temperature / 1000.0;
}

bool JuGeNewInfrared::setInfraredDDTFile(QString filePath)
{
    ROS_INFO("JuGeNewInfrared::m_iChannelIndex %d!", m_iChannelIndex);
    OutputPara paraOut;
    paraOut.dwFPAWidth = 640;
    paraOut.dwFPAHeight = 480;
    paraOut.dwBMPWidth = 640;
    paraOut.dwBMPHeight = 480;
    paraOut.dwColorBarWidth = 20;
    paraOut.dwColorBarHeight = 100;
    ROS_INFO("JuGeNewInfrared::getInfraredRectTemp filePath: %s", filePath.toStdString().data());

    WCHAR* charFilename;
    QString str = filePath.replace("_result.jpg", ".ddt");
    QByteArray ba = str.toLocal8Bit();
    char *s = ba.data();
    int w_nlen = MultiByteToWideChar(CP_ACP, 0, s, -1, NULL, 0);
    WCHAR *ret;
    ret = (WCHAR*)malloc(sizeof(WCHAR)*w_nlen);
    memset(ret, 0, sizeof(ret));
    MultiByteToWideChar(CP_ACP, 0, s, -1, ret, w_nlen);
    charFilename = ret;

    if (!MAG_LoadDDT(m_iChannelIndex, &paraOut, charFilename, NULL, 0))
    {
        ROS_INFO("JuGeNewInfrared::MAG_LoadDDT!");

        return false;
    }
}