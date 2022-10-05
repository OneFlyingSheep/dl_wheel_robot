#pragma once
#include <LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h>
#include "common/DLWheelRobotGlobalDef.hpp"

class DLWheelPointTreeData
{
public:
	DLWheelPointTreeData();
	~DLWheelPointTreeData();

	QList<QStringList> GetEquipmentTreeData();

	QList<QStringList> GetAllLevelDatas();
	QList<QStringList> GetAllEquipmentTypeDatas();
	QList<QStringList> GetAllEquipmentDatas(QString strTypeID);


	QList<WheelPointTreeDataStruct> getPointTreeDataFromDB();
	QList<WheelPointTreeDataStruct> getPointTreeDataForSearchFromDB(WheelPatrolParameter m_wheelPatrolPara);
	QStandardItemModel* addTreeItemModel(QList<WheelPointTreeDataStruct> treeDataList);
	WheelRobortDevicesAlarmColorStruct getDeviceAlarmColorFromDB();
	//节点告警状态
	QList<WheelRobortAlarmPathStruct> getNodeAlarmStatusFromDevices(QString m_device_uuid);
	//设备节点路径
	QStringList getDeviceNodePathFromDevices(QString m_device_uuid);

	void setTreeSave(int choose, WheelPatrolParameter m_wheelPatrolPara);

	void setLevelCompare(int newLevel2, int newLevel3, int newLevel4);

	QList<WheelTreeSaveStruct> getPointTreeChooseSecond();
	QList<WheelTreeSaveStruct> getPointTreeChooseThird(QStringList m_path);
	QList<WheelTreeSaveStruct> getPointTreeChooseFourth(QStringList m_path);
	QList<WheelTreeSaveStruct> getPointTreeChooseFifth(QStringList m_path);
	
	//点位树节点查询
	QList<WheelTreeSaveStruct> getPointTreeRootNode(QStringList m_path);
	//公司节点 第一节点
	void getPointTreeFirstRootNode(QString &m_name, int &m_level);
	//站所节点 第二节点
	void getPointTreeSecondRootNode(QString &m_name, int &m_level);

	//根据树节点获取下面所有的device_uuid
	QList<QString> getDeviceUUidForRootNode(QStringList m_path);

	QList<QString> getDeviceUUidChooseSecond();
	QList<QString> getDeviceUUidChooseThird(QStringList m_path);
	QList<QString> getDeviceUUidChooseFourth(QStringList m_path);
	QList<QString> getDeviceUUidChooseFifth(QStringList m_path);

	//刷新,重新拉取设备数结构数据
	void updataTree();
	//刷新保留
	void updataTree(WheelPatrolParameter m_wheelPatrolPara);
	//返回树所有节点数据
	QList<WheelTreeSaveStruct> getTreeEntiretyData()
	{
		return m_treeSaveQList;
	}
	//发送树查询条件
	void setTreeConditionForSearchData(WheelPatrolParameter m_wheelPatrolPara);
	
	//返回查询后的QMap结构
	QMap<QString, WheelPointTreeDataStruct>getTreeSearchData()
	{
		return m_treeSerachQMap;
	}

	bool isExiThisNode(int nodeID, QString stdUUid);

	//初始化设备树、刷新设备树各个节点，以RootNodeType枚举为查询类型
	//参数：节点id list ，枚举类型，初始化全查询不使用参数
	QList<QStringList> getCollectTreeDataFromDB(QStringList selectUUid = {""}, RootNodeType selectType = RootNode_Init);

private:
	int maxLevel2;
	int maxLevel3;
	int maxLevel4;
	QList<WheelTreeSaveStruct> m_treeSaveQList;
	QMap<QString, WheelPointTreeDataStruct> m_treeSerachQMap;
};

