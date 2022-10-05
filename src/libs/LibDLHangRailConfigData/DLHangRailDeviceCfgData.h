#ifndef __DL_HANG_ROBOT_DEVICE_DATA_H__
#define __DL_HANG_ROBOT_DEVICE_DATA_H__

#include "common/DLHangRailRobotGlobalDef.hpp"
#include "LibDLHangRailRobotDBOperation/LibDLHangRailRobotDBOperation.h"
#include "common/Singleton.hpp"
#include <boost/thread/mutex.hpp>
#include <QList>
#include <map>

class DLHangRailDeviceCfgData : public Singleton<DLHangRailDeviceCfgData>
{
public:
    DLHangRailDeviceCfgData();
    ~DLHangRailDeviceCfgData();

public:
    bool addInDevice(deviceConfigType& dev);
    bool deleteDeviceBySsid(QString Ssid);
    bool addInVirtualDevice(virtualDeviceType &dev);
    bool deleteVirtualDeviceBySsid(QString Ssid);
	bool updateSingleDeviceBySsid(deviceConfigType dev);
    bool getDeviceBySsId(QString Ssid, deviceConfigType &dev);
    bool getVirDeviceBySsid(QString Ssid, virtualDeviceType &dev);
    bool getDeviceNameById(int id, QString &devName);
    bool getDevicesByAreaName(QString areaName, QList<deviceConfigType> &dev);
    bool getDeviceByAreaNameUnionDeviceName(QString areaName, QList<QString> &dev);

    int getDeviceDetailIdByDeviceNameId(int nameId);
    QString getDeviceUnitNameByNameId(int nameId);

    int getDeviceTypeIdByNameId(int nameId);
    QString getDeviceTypeNameByNameId(int nameId);
    
    QString getDeviceAlterNameByAlterNameId(int nameId);
    QString getClassifierNameByNameId(int nameId);
    QString getDeviceNameByNameId(int nameId);

    QStringList getDevicesExistsAreaName();
    std::map<int, deviceNameType> getDeviceNameTypeList();
    std::map<int, deviceAreaType> getDeviceAreaTypeList();
    std::map<int, deviceDetailType> getDeviceDetailTypeList();
    std::map<int, deviceUnitType> getDeviceUnitTypeList();
    std::map<int, deviceType> getDeviceTypeList();
    std::map<int, deviceAlternateNameType> getDeviceAllAlternateNameTypeList();
    std::map<int, deviceAlternateNameType> getDeviceRealityAlternateNameTypeList();
    std::map<int, deviceAlternateNameType> getDeviceVirtualAlternateNameTypeList();

    QList<deviceConfigType> getDevices();
	QList<QString> getDeviceAreaNameList();

	//虚拟设备初始化列表
	QList<RelevanceDevice> getVirtualDeviceInitialList();
	//虚拟设备id和名称--负载检测--三项温差--一致性
	std::map<int ,QString> getVirtualDeviceNameMap();
	//某区域下的未关联的设备列表
	QList<QString> getSearchAreaDeviceList(QString m_areaName);

public:
    void loadDevice();
    void loadDeviceSortByAreaName();
    void saveDevice();
    void loadDeviceExistAreaNames();
    void loadDeviceDetailType();
    void loadDeviceNameType();
    void loadDeviceAreaType();
    void loadDevicePatrolType();
    void loadDeviceUnitType();
    void loadDeviceType();

    void loadDeviceAlternateName();
	void loadDeviceAreaName();

	void loadVirtualDeviceInitialList();
	void loadVirtualDeviceNameMap();
	void loadSearchAreaDeviceList(QString m_areaName);

private:
    QList<deviceConfigType> m_deviceList;
	QList<QString> m_deviceAreaNameList;
	QList<RelevanceDevice> m_virDeviceInitializeList;
	QList<QString> m_searchAreaDeviceList;
    QStringList m_deviceExistNames;

	std::map<int, QString> m_virDeviceNameMap;
    std::map<int, deviceDetailType> m_deviceDetailTypeList;
    std::map<int, deviceNameType> m_deviceNameTypeList;
    std::map<int, deviceAreaType> m_deviceAreaTypeList;
    std::map<int, devicePatrolType> m_devicePatrolTypeList;
    std::map<int, deviceUnitType> m_deviceUnitTypeList;
    std::map<int, deviceType> m_deviceTypeList;
    std::map<int, deviceAlternateNameType> m_deviceAllAlternateNameTypeList;
    std::map<int, deviceAlternateNameType> m_deviceRealityAlternateNameTypeList;
    std::map<int, deviceAlternateNameType> m_deviceVirtualAlternateNameTypeList;

    QString getSessionId();
};

#define ROBOT_DEVICE_CFG DLHangRailDeviceCfgData::GetSingleton()

#endif