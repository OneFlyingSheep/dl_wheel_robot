#include "DLWheelRobotVirtualDevice.h"
#include <QUuid>

DLWheelRobotVirtualDevice::DLWheelRobotVirtualDevice()
{

}

DLWheelRobotVirtualDevice::~DLWheelRobotVirtualDevice()
{

}

void DLWheelRobotVirtualDevice::autoRelevanceDevice()
{
	WHEEL_ROBOT_DB.dataClearForVirtual();
	autoRelevanceDeviceThreeCompare();
	autoRelevanceDeviceRevealAmpere();
}

void DLWheelRobotVirtualDevice::autoRelevanceDeviceThreeCompare()
{
	QMap<QString, DeviceVirtualSort> deviceMap;
	WHEEL_ROBOT_DB.getWheelVirtualDeviceThreeCompareMap(deviceMap);

	while (deviceMap.size())
	{
		QMap<QString, DeviceVirtualSort>::iterator it;
		it = deviceMap.begin();

		QString deviceUUid = it.key();
		QString emIntervalUUid = it.value().equipment_interval_uuid;
		QString subDeviceUUid = it.value().sub_device_type_uuid;
		QString devPointTypeUUid = it.value().device_point_type_uuid;
		QString devPhaseId = it.value().device_phase_id;

		int count = 0;
		QList<QString> updateDevice;
		QMap<QString, DeviceVirtualSort>::iterator it_m;
		QList<QMap<QString, DeviceVirtualSort>::iterator> it_l;

		for (it_m = deviceMap.begin(); it_m != deviceMap.end(); it_m++)
		{
// 			if (emIntervalUUid == it_m.value().equipment_interval_uuid && devPointTypeUUid == it_m.value().device_point_type_uuid)
// 			{
// 				count++;
// 				updateDevice.append(it_m.key());
// 				it_l.append(it_m);
// 			}
            if (emIntervalUUid == it_m.value().equipment_interval_uuid && subDeviceUUid == it_m.value().sub_device_type_uuid)
            {
                count++;
                updateDevice.append(it_m.key());
                it_l.append(it_m);
            }
		}
		for (int i = 0; i < it_l.size(); i++)
		{
			deviceMap.erase(it_l[i]);
		}
		if (count == 3)
		{
			QUuid u = QUuid::createUuid();
			QString virtualUUid = u.toString().remove("{").remove("}").remove("-");
			WHEEL_ROBOT_DB.updateDeviceVirtualAndInsertVirtual(virtualUUid, updateDevice, subDeviceUUid);
		}
	}
}

void DLWheelRobotVirtualDevice::autoRelevanceDeviceRevealAmpere()
{
	QMap<QString, DeviceVirtualSort> deviceMap;
	WHEEL_ROBOT_DB.getWheelVirtualDeviceRevealAmpereMap(deviceMap);

	while (deviceMap.size())
	{
		QMap<QString, DeviceVirtualSort>::iterator it;
		it = deviceMap.begin();

		QString deviceUUid = it.key();
		QString emIntervalUUid = it.value().equipment_interval_uuid;
		QString subDeviceTypeUUid = it.value().sub_device_type_uuid;
		QString devPointTypeUUid= it.value().device_point_type_uuid;
		QString devPhaseId = it.value().device_phase_id;

		int count = 0;
		QList<QString> updateDevice;
		QMap<QString, DeviceVirtualSort>::iterator it_m;
		QList<QMap<QString, DeviceVirtualSort>::iterator> it_l;

		for (it_m = deviceMap.begin(); it_m != deviceMap.end(); it_m++)
		{
			if (emIntervalUUid == it_m.value().equipment_interval_uuid && 
				subDeviceTypeUUid == it_m.value().sub_device_type_uuid &&
				devPhaseId == it.value().device_phase_id)
			{
				count++;
				updateDevice.append(it_m.key());
				it_l.append(it_m);
			}
		}
		for (int i = 0; i < it_l.size(); i++)
		{
			deviceMap.erase(it_l[i]);
		}
		if (count == 2)
		{
			QUuid u = QUuid::createUuid();
			QString virtualUUid = u.toString().remove("{").remove("}").remove("-");
			WHEEL_ROBOT_DB.updateDeviceVirtualAndInsertVirtual(virtualUUid, updateDevice, subDeviceTypeUUid);
		}
	}
}
