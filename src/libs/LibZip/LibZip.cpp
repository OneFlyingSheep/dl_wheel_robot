#include <iostream>
#include <chrono>
#include <thread>
#include "LibZip.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include <QFileInfo>

LibZip::LibZip()
{
}

LibZip::~LibZip()
{
}

void LibZip::create_taskfile_zip(QString taskUUid)
{
    std::thread t1(std::bind(&LibZip::create_zip, this, taskUUid));
    t1.detach();
}

void LibZip::create_zip(QString taskUUid)
{
    QString rootPath = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath;
    std::string zipPath = QString(rootPath + "/task_zip/" + taskUUid + ".zip").toStdString();
    HZIP hz = CreateZip(zipPath.c_str(), 0);
    QStringList deviceList;
    WHEEL_ROBOT_DB.getDeviceUUidWhihTaskUUid(taskUUid, deviceList);
    for (int i = 0; i < deviceList.size(); i++)
    {
        std::string strPath = QString(taskUUid + "/" + deviceList[i] + "/").toStdString();
        std::string readPath = QString(rootPath + "/task/").toStdString();
        std::string strOriName = QString(deviceList[i] + ".jpg").toStdString();
        std::string strResName = QString(deviceList[i] + "_result.jpg").toStdString();
        ZipAdd(hz, std::string(strPath + strOriName).c_str(), std::string(readPath + strPath + strOriName).c_str());
        ZipAdd(hz, std::string(strPath + strResName).c_str(), std::string(readPath + strPath + strResName).c_str());
    }

    CloseZip(hz);
}

void LibZip::unzip_taskfile_zip(QString taskUUid,QString unzipPath)
{
    QString rootPath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath;
    std::string curPath;
    std::string filePath = QString(rootPath + "/task_zip/" + taskUUid + ".zip").toStdString();
    if (unzipPath.isEmpty())
	{
        curPath = QString(rootPath + "/DeviceImage").toStdString();
    }
    else
    {
        curPath = unzipPath.toStdString();
    }
    SetCurrentDirectory(curPath.c_str());
    HZIP hz = OpenZip(filePath.c_str(), 0);
    ZIPENTRY ze; 
    GetZipItem(hz, -1, &ze); 
    int fileSize = ze.index;
    for (int i = 0; i < fileSize; i++)
    {
        GetZipItem(hz, i, &ze);
        UnzipItem(hz, i, ze.name);
    }
    CloseZip(hz);
}

bool LibZip::is_exits_file(QString taskUUid)
{
    QString rootPath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath;
    QString filePath = QString(rootPath + "/task_zip/" + taskUUid + ".zip");
    QFileInfo file(filePath);
    if (file.exists())
        return true;
    return false;
}
