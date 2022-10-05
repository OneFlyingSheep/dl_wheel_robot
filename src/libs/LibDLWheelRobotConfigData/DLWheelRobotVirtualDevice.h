#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "common/Singleton.hpp"

class DLWheelRobotVirtualDevice : public Singleton<DLWheelRobotVirtualDevice>
{
public:
	DLWheelRobotVirtualDevice();
	~DLWheelRobotVirtualDevice();

	//自动关联设备
	void autoRelevanceDevice();
	//三项温差
	void autoRelevanceDeviceThreeCompare();
	//泄露电流表
	void autoRelevanceDeviceRevealAmpere();

private:

};

#define WHEEL_VIRTUAL_DEVICE DLWheelRobotVirtualDevice::GetSingleton()
