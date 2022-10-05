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
	//�ڵ�澯״̬
	QList<WheelRobortAlarmPathStruct> getNodeAlarmStatusFromDevices(QString m_device_uuid);
	//�豸�ڵ�·��
	QStringList getDeviceNodePathFromDevices(QString m_device_uuid);

	void setTreeSave(int choose, WheelPatrolParameter m_wheelPatrolPara);

	void setLevelCompare(int newLevel2, int newLevel3, int newLevel4);

	QList<WheelTreeSaveStruct> getPointTreeChooseSecond();
	QList<WheelTreeSaveStruct> getPointTreeChooseThird(QStringList m_path);
	QList<WheelTreeSaveStruct> getPointTreeChooseFourth(QStringList m_path);
	QList<WheelTreeSaveStruct> getPointTreeChooseFifth(QStringList m_path);
	
	//��λ���ڵ��ѯ
	QList<WheelTreeSaveStruct> getPointTreeRootNode(QStringList m_path);
	//��˾�ڵ� ��һ�ڵ�
	void getPointTreeFirstRootNode(QString &m_name, int &m_level);
	//վ���ڵ� �ڶ��ڵ�
	void getPointTreeSecondRootNode(QString &m_name, int &m_level);

	//�������ڵ��ȡ�������е�device_uuid
	QList<QString> getDeviceUUidForRootNode(QStringList m_path);

	QList<QString> getDeviceUUidChooseSecond();
	QList<QString> getDeviceUUidChooseThird(QStringList m_path);
	QList<QString> getDeviceUUidChooseFourth(QStringList m_path);
	QList<QString> getDeviceUUidChooseFifth(QStringList m_path);

	//ˢ��,������ȡ�豸���ṹ����
	void updataTree();
	//ˢ�±���
	void updataTree(WheelPatrolParameter m_wheelPatrolPara);
	//���������нڵ�����
	QList<WheelTreeSaveStruct> getTreeEntiretyData()
	{
		return m_treeSaveQList;
	}
	//��������ѯ����
	void setTreeConditionForSearchData(WheelPatrolParameter m_wheelPatrolPara);
	
	//���ز�ѯ���QMap�ṹ
	QMap<QString, WheelPointTreeDataStruct>getTreeSearchData()
	{
		return m_treeSerachQMap;
	}

	bool isExiThisNode(int nodeID, QString stdUUid);

	//��ʼ���豸����ˢ���豸�������ڵ㣬��RootNodeTypeö��Ϊ��ѯ����
	//�������ڵ�id list ��ö�����ͣ���ʼ��ȫ��ѯ��ʹ�ò���
	QList<QStringList> getCollectTreeDataFromDB(QStringList selectUUid = {""}, RootNodeType selectType = RootNode_Init);

private:
	int maxLevel2;
	int maxLevel3;
	int maxLevel4;
	QList<WheelTreeSaveStruct> m_treeSaveQList;
	QMap<QString, WheelPointTreeDataStruct> m_treeSerachQMap;
};

