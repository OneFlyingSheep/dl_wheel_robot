#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "common/Singleton.hpp"

class DLWheelRobotVirtualDevice : public Singleton<DLWheelRobotVirtualDevice>
{
public:
	DLWheelRobotVirtualDevice();
	~DLWheelRobotVirtualDevice();

	//�Զ������豸
	void autoRelevanceDevice();
	//�����²�
	void autoRelevanceDeviceThreeCompare();
	//й¶������
	void autoRelevanceDeviceRevealAmpere();

private:

};

#define WHEEL_VIRTUAL_DEVICE DLWheelRobotVirtualDevice::GetSingleton()
