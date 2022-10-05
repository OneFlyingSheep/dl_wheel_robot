#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "common/Singleton.hpp"
#include <QList>
class DLWheelRobotTaskConfig : public Singleton<DLWheelRobotTaskConfig>
{
public:
	DLWheelRobotTaskConfig();
	~DLWheelRobotTaskConfig();

	void loadWheelTaskEndActionData();
	std::map<int, QString> getWheelTaskEndActionDataMap();
	QList<QString> getWheelTaskEndActionNameQList();
	QString getWheelTaskEndActionNameQString(int m_task_end_action_id);
	int getWheelTaskEndActionIdInt(QString m_task_end_action_name);

	void loadWheelTaskEndTypeData();
	std::map<int, QString> getWheelTaskEndTypeDataMap();
	QList<QString> getWheelTaskEndTypeNameQList();
	QString getWheelTaskEndTypeNameQString(int m_task_end_type_id);
	int getWheelTaskEndTypeIdInt(QString m_task_end_type_name);

	void loadWheelTaskTypeData();
	std::map<int, QString> getWheelTaskTypeDataMap();
	QList<QString> getWheelTaskTypeNameQList();
	QString getWheelTaskTypeNameQString(int m_task_end_type_id);
	int getWheelTaskTypeIdInt(QString m_task_end_type_name);

	void loadWheelTaskStatusData();
	std::map<int, QString> getWheelTaskStatusDataMap();

private:
	std::map<int, QString> m_WheelTaskEndActionData;
	QList<QString> m_WheelTaskEndActionNameQList;

	std::map<int, QString> m_WheelTaskEndTypeData;
	QList<QString> m_WheelTaskEndTypeQList;

	std::map<int, QString> m_WheelTaskTypeData;
	QList<QString> m_WheelTaskTypeQList;

	std::map<int, QString> m_WheelTaskStatusData;
};

#define WHEEL_TASK_END_ACTION DLWheelRobotTaskConfig::GetSingleton()

