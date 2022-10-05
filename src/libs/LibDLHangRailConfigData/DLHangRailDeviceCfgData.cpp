#include "DLHangRailDeviceCfgData.h"
#include <fstream>
#include <boost/uuid/uuid_generators.hpp>  
#include <boost/uuid/uuid_io.hpp>  
#include <boost/uuid/uuid.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string.hpp>

DLHangRailDeviceCfgData::DLHangRailDeviceCfgData()
{
    loadDeviceSortByAreaName();
    loadDeviceDetailType();
    loadDeviceNameType();
    loadDeviceAreaType();
    loadDevicePatrolType();
    loadDeviceUnitType();
	loadDeviceType();
	loadDeviceAreaName();
	loadVirtualDeviceNameMap();
    loadDeviceAlternateName();
}

DLHangRailDeviceCfgData::~DLHangRailDeviceCfgData()
{

}

bool DLHangRailDeviceCfgData::addInDevice(deviceConfigType& dev)
{
//     dev.device_ssid = getSessionId();
//     if (ROBOTDB_DB.insertSingleDevice(dev))
//     {
//         loadDeviceSortByAreaName();
//         loadDeviceExistAreaNames();
// 		return true;
//     }
	return false;
}

bool DLHangRailDeviceCfgData::deleteDeviceBySsid(QString Ssid)
{
// 	if (ROBOTDB_DB.deleteSingleDeviceBySsId(Ssid))
// 	{
	loadDeviceSortByAreaName();
	loadDeviceExistAreaNames();
// 		return true;
// 	}
    return false;
}

bool DLHangRailDeviceCfgData::addInVirtualDevice(virtualDeviceType &dev)
{
    return ROBOTDB_DB.insertSingleVirDevice(dev);
}

bool DLHangRailDeviceCfgData::deleteVirtualDeviceBySsid(QString Ssid)
{
    return ROBOTDB_DB.deleteSingleVirDeviceBySsId(Ssid);
}

bool DLHangRailDeviceCfgData::updateSingleDeviceBySsid(deviceConfigType dev)
{
	if (ROBOTDB_DB.updateSingleDeviceBySsid(dev))
	{
		loadDeviceSortByAreaName();
		loadDeviceExistAreaNames();
		return true;
	}
	return false;
}

bool DLHangRailDeviceCfgData::getDeviceBySsId(QString Ssid, deviceConfigType &dev)
{
    return ROBOTDB_DB.getSingleDeviceBySsId(Ssid, dev);
}

bool DLHangRailDeviceCfgData::getVirDeviceBySsid(QString Ssid, virtualDeviceType &dev)
{
    return ROBOTDB_DB.getSingleVirDeviceBySsId(Ssid, dev);
}

QStringList DLHangRailDeviceCfgData::getDevicesExistsAreaName()
{
    return m_deviceExistNames;
}

bool DLHangRailDeviceCfgData::getDeviceNameById(int id, QString &devName)
{
    return ROBOTDB_DB.getDeviceNameByDeviceNameId(id, devName);
}

bool DLHangRailDeviceCfgData::getDevicesByAreaName(QString areaName, QList<deviceConfigType> &dev)
{
    return ROBOTDB_DB.getMultiDevicesByAreaName(areaName, dev);
}

bool DLHangRailDeviceCfgData::getDeviceByAreaNameUnionDeviceName(QString areaName, QList<QString> &dev)
{
    return ROBOTDB_DB.getDeviceByAreaNameInnerJoinDeviceName(areaName, dev);
}

int DLHangRailDeviceCfgData::getDeviceDetailIdByDeviceNameId(int nameId)
{
    int detailId = 0;
    if (m_deviceNameTypeList.find(nameId) != m_deviceNameTypeList.end())
    {
        detailId = m_deviceNameTypeList.at(nameId).device_detail_id;
    }
    return detailId;
}

QString DLHangRailDeviceCfgData::getDeviceUnitNameByNameId(int nameId)
{
    int deviceUnitId = 0;
    QString devUnitName = "";
    if (m_deviceNameTypeList.find(nameId) != m_deviceNameTypeList.end())
    {
        deviceUnitId = m_deviceNameTypeList.at(nameId).device_unit_id;
    }
    else
    {
        return devUnitName;
    }
    if (m_deviceUnitTypeList.find(deviceUnitId) != m_deviceUnitTypeList.end());
    {
        devUnitName = m_deviceUnitTypeList.at(deviceUnitId).device_unit_name;
    }
    return devUnitName;
}

QString DLHangRailDeviceCfgData::getDeviceTypeNameByNameId(int nameId)
{
    int deviceTypeId = 0;
    QString devTypeName = "";
    if (m_deviceNameTypeList.find(nameId) != m_deviceNameTypeList.end())
    {
        deviceTypeId = m_deviceNameTypeList.at(nameId).device_type_id;
    }
    else
    {
        return devTypeName;
    }
    if (m_deviceTypeList.find(deviceTypeId) != m_deviceTypeList.end());
    {
        devTypeName = m_deviceTypeList.at(deviceTypeId).device_type_name;
    }
    return devTypeName;
}

QString DLHangRailDeviceCfgData::getDeviceAlterNameByAlterNameId(int nameId)
{
    QString alterName = "";
    if (m_deviceAllAlternateNameTypeList.find(nameId) != m_deviceAllAlternateNameTypeList.end())
    {
        alterName = m_deviceAllAlternateNameTypeList.at(nameId).device_alternate_name;
    }
    return alterName;
}

QString DLHangRailDeviceCfgData::getClassifierNameByNameId(int nameId)
{
    int deviceDetailId = 0;
    QString devClassifierName = "";
    if (m_deviceNameTypeList.find(nameId) != m_deviceNameTypeList.end())
    {
        deviceDetailId = m_deviceNameTypeList.at(nameId).device_detail_id;
    }
    else
    {
        return devClassifierName;
    }
    if (m_deviceDetailTypeList.find(deviceDetailId) != m_deviceDetailTypeList.end());
    {
        devClassifierName = m_deviceDetailTypeList.at(deviceDetailId).device_classifier_name;
    }
    return devClassifierName;
}

std::map<int, deviceNameType> DLHangRailDeviceCfgData::getDeviceNameTypeList()
{
    return m_deviceNameTypeList;
}

std::map<int, deviceAreaType> DLHangRailDeviceCfgData::getDeviceAreaTypeList()
{
    return m_deviceAreaTypeList;
}

std::map<int, deviceDetailType> DLHangRailDeviceCfgData::getDeviceDetailTypeList()
{
    return m_deviceDetailTypeList;
}

std::map<int, deviceUnitType> DLHangRailDeviceCfgData::getDeviceUnitTypeList()
{
    return m_deviceUnitTypeList;
}

std::map<int, deviceType> DLHangRailDeviceCfgData::getDeviceTypeList()
{
    return m_deviceTypeList;
}

std::map<int, deviceAlternateNameType> DLHangRailDeviceCfgData::getDeviceAllAlternateNameTypeList()
{
    return m_deviceAllAlternateNameTypeList;
}

std::map<int, deviceAlternateNameType> DLHangRailDeviceCfgData::getDeviceRealityAlternateNameTypeList()
{
    return m_deviceRealityAlternateNameTypeList;
}

std::map<int, deviceAlternateNameType> DLHangRailDeviceCfgData::getDeviceVirtualAlternateNameTypeList()
{
    return m_deviceVirtualAlternateNameTypeList;
}

QList<deviceConfigType> DLHangRailDeviceCfgData::getDevices()
{
    return m_deviceList;
}

QList<QString> DLHangRailDeviceCfgData::getDeviceAreaNameList()
{
	loadDeviceAreaName();
	return m_deviceAreaNameList;
}

void DLHangRailDeviceCfgData::loadDevice()
{
	m_deviceList.clear();
    if (ROBOTDB_DB.getAllDevices(m_deviceList))
    {

    }
    else
    {

    }
}

void DLHangRailDeviceCfgData::loadDeviceSortByAreaName()
{
    m_deviceList.clear();
    if (ROBOTDB_DB.getAllDevicesSortByDeviceAreaName(m_deviceList))
    {

    }
    else
    {

    }
}

void DLHangRailDeviceCfgData::saveDevice()
{

}

void DLHangRailDeviceCfgData::loadDeviceExistAreaNames()
{
    m_deviceExistNames.clear();
    ROBOTDB_DB.getExistDeviceAreaNames(m_deviceExistNames);
}

void DLHangRailDeviceCfgData::loadDeviceDetailType()
{
    m_deviceDetailTypeList.clear();
    ROBOTDB_DB.getDeviceDetailType(m_deviceDetailTypeList);
}

void DLHangRailDeviceCfgData::loadDeviceNameType()
{
    m_deviceNameTypeList.clear();
    ROBOTDB_DB.getDeviceNameType(m_deviceNameTypeList);
}

void DLHangRailDeviceCfgData::loadDeviceAreaType()
{
    m_deviceAreaTypeList.clear();
    ROBOTDB_DB.getDeviceAreaType(m_deviceAreaTypeList);
}

void DLHangRailDeviceCfgData::loadDevicePatrolType()
{
    m_devicePatrolTypeList.clear();
    ROBOTDB_DB.getDevicePatrolType(m_devicePatrolTypeList);
}

void DLHangRailDeviceCfgData::loadDeviceUnitType()
{
    m_deviceUnitTypeList.clear();
    ROBOTDB_DB.getDeviceUnitType(m_deviceUnitTypeList);
}

void DLHangRailDeviceCfgData::loadDeviceType()
{
    m_deviceTypeList.clear();
    ROBOTDB_DB.getDeviceType(m_deviceTypeList);
}

void DLHangRailDeviceCfgData::loadDeviceAlternateName()
{
    m_deviceAllAlternateNameTypeList.clear();
    m_deviceVirtualAlternateNameTypeList.clear();
    m_deviceRealityAlternateNameTypeList.clear();
    ROBOTDB_DB.getDeviceAlternateName(m_deviceAllAlternateNameTypeList);
    std::map<int, deviceAlternateNameType>::iterator itr = m_deviceAllAlternateNameTypeList.begin();
    for (; itr != m_deviceAllAlternateNameTypeList.end(); itr++)
    {
        deviceAlternateNameType dev = itr->second;
        int deviceNameId = dev.device_alternate_name_id;
        if (DEVICE_DETAIL_LOAD_DETECTION == deviceNameId || DEVICE_DETAIL_THREE_PHASE_TEMPERATURE == deviceNameId || DEVICE_DETAIL_CONSISTENCY == deviceNameId)
        {
            m_deviceVirtualAlternateNameTypeList.insert(std::make_pair(deviceNameId, dev));
        }
        else
        {
            m_deviceRealityAlternateNameTypeList.insert(std::make_pair(deviceNameId, dev));
        }
    }
}

void DLHangRailDeviceCfgData::loadDeviceAreaName()
{
	m_deviceAreaNameList.clear();
	ROBOTDB_DB.getDeviceAreaNameList(m_deviceAreaNameList);
}

QString DLHangRailDeviceCfgData::getSessionId()
{
    boost::uuids::random_generator rgen;//随机生成器  
    boost::uuids::uuid ssid = rgen();//生成一个随机的UUID
    std::string tmp = boost::lexical_cast<std::string>(ssid);
    boost::erase_all(tmp, "-");
    QString qsid = QString::fromStdString(tmp);
    return qsid;
}

void DLHangRailDeviceCfgData::loadVirtualDeviceInitialList()
{
	m_virDeviceInitializeList.clear();
	ROBOTDB_DB.getVirtualDeviceList(m_virDeviceInitializeList);
}

QList<RelevanceDevice> DLHangRailDeviceCfgData::getVirtualDeviceInitialList()
{
	loadVirtualDeviceInitialList();
	return m_virDeviceInitializeList;
}

void DLHangRailDeviceCfgData :: loadVirtualDeviceNameMap()
{
	m_virDeviceNameMap.clear();
	ROBOTDB_DB.getVirtualDeviceNameMap(m_virDeviceNameMap);
}

std::map<int, QString> DLHangRailDeviceCfgData::getVirtualDeviceNameMap()
{
	return m_virDeviceNameMap;
}

void DLHangRailDeviceCfgData::loadSearchAreaDeviceList(QString m_areaName)
{
	m_searchAreaDeviceList.clear();
	ROBOTDB_DB.getSearchAreaDeviceList(m_areaName, m_searchAreaDeviceList);
}

QList<QString> DLHangRailDeviceCfgData::getSearchAreaDeviceList(QString m_areaName)
{
	loadSearchAreaDeviceList(m_areaName);
	return m_searchAreaDeviceList;
}