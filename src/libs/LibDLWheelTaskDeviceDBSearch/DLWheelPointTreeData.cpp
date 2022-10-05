#include "DLWheelPointTreeData.h"
#include <LibDLWheelRobotConfigData/DLWheelRobotStationConfig.h>
#include <QStandardItemModel>

DLWheelPointTreeData::DLWheelPointTreeData()
{
	updataTree();
}

DLWheelPointTreeData::~DLWheelPointTreeData()
{
}

QList<QStringList> DLWheelPointTreeData::GetEquipmentTreeData()
{
	QList<QStringList> lstEquipmentDatas;
	WHEEL_ROBOT_DB.getWheelEquipmentTreeDataDB(lstEquipmentDatas);
	return lstEquipmentDatas;
}

QList<QStringList> DLWheelPointTreeData::GetAllLevelDatas()
{
	QList<QStringList> lstAllLevelDatas;
	WHEEL_ROBOT_DB.getWheelLevelDataDB(lstAllLevelDatas);
	return lstAllLevelDatas;
}

QList<QStringList> DLWheelPointTreeData::GetAllEquipmentTypeDatas()
{
	QList<QStringList> lstAllEquipmentTypeDatas;
	WHEEL_ROBOT_DB.getWheelEquipmentTypeDataDB(lstAllEquipmentTypeDatas);
	return lstAllEquipmentTypeDatas;
}


QList<QStringList> DLWheelPointTreeData::GetAllEquipmentDatas(QString strTypeID)
{
	QList<QStringList> lstAllEquipmentDatas;
	WHEEL_ROBOT_DB.getWheelEquipmentDataDB(lstAllEquipmentDatas, strTypeID);
	return lstAllEquipmentDatas;
}

QList<WheelPointTreeDataStruct> DLWheelPointTreeData::getPointTreeDataFromDB()
{
	QList<WheelPointTreeDataStruct> m_wheelPointTreeDataStru;
	bool bRet = WHEEL_ROBOT_DB.getWheelPointTreeDataDB(m_wheelPointTreeDataStru);
	if (bRet)
	{
		QString companyName = WHEEL_STATION_CONFIG.getWheelStationConfigData().at(1).company_name;
		QString stationName = WHEEL_STATION_CONFIG.getWheelStationConfigData().at(1).station_name;
		for (int i = 0; i < m_wheelPointTreeDataStru.size(); i++)
		{
			m_wheelPointTreeDataStru[i].wheel_pointTree_data.prepend(stationName);
			m_wheelPointTreeDataStru[i].wheel_pointTree_data.prepend(companyName);
		}
	}
	return m_wheelPointTreeDataStru;
}

QList<WheelPointTreeDataStruct> DLWheelPointTreeData::getPointTreeDataForSearchFromDB(WheelPatrolParameter m_wheelPatrolPara)
{
	QList<WheelPointTreeDataStruct> m_wheelPointTreeDataStru;
	bool bRet = WHEEL_ROBOT_DB.getWheelPointTreeDataDB(m_wheelPointTreeDataStru, m_wheelPatrolPara);
	if (bRet)
	{
		QString companyName = WHEEL_STATION_CONFIG.getWheelStationConfigData().at(1).company_name;
		QString stationName = WHEEL_STATION_CONFIG.getWheelStationConfigData().at(1).station_name;
		for (int i = 0; i < m_wheelPointTreeDataStru.size(); i++)
		{
			m_wheelPointTreeDataStru[i].wheel_pointTree_data.prepend(stationName);
			m_wheelPointTreeDataStru[i].wheel_pointTree_data.prepend(companyName);
		}
	}
	return m_wheelPointTreeDataStru;
}

QStandardItemModel* DLWheelPointTreeData::addTreeItemModel(QList<WheelPointTreeDataStruct> treeDataList)
{
	QMap<QString, QIcon> m_publicIconMap;///< 存放公共图标
	m_publicIconMap[QStringLiteral("tP")] = QIcon(QStringLiteral("F:/folder.png"));

	QStandardItemModel * m_ConfirmPatrolModel=new QStandardItemModel;
	m_ConfirmPatrolModel->setHorizontalHeaderLabels(QStringList() << QString("%1").arg(treeDataList.at(0).wheel_pointTree_data.at(0)));

	QStandardItem* StationName = new QStandardItem(m_publicIconMap[QString("tP")], QString("%1").arg(treeDataList.at(0).wheel_pointTree_data.at(1)));
	//QStandardItem* StationName = new QStandardItem(QString("%1").arg(treeDataList.at(0).wheel_pointTree_data.at(1)));
	m_ConfirmPatrolModel->appendRow(StationName);
	QStandardItem* VoltageLevel = new QStandardItem;
	QStandardItem* IntervalNode = new QStandardItem;
	QStandardItem* DeviceTypeNode = new QStandardItem;
	QStandardItem* PointNameNode = new QStandardItem;

	QString strVoltageLevelNode = "";
	QString strIntervalNode = "";
	QString strDeviceTypeNode = "";

	for (int i = 0; i < treeDataList.count(); i++)
	{
		WheelPointTreeDataStruct treeData = treeDataList.at(i);
		QString curretnVoltageLevelNode = treeData.wheel_pointTree_data[2];//
		QString currentIntervalNode = treeData.wheel_pointTree_data[3];    //
		QString currentDeviceTypeNode = treeData.wheel_pointTree_data[4];  //
		QString currentDataNode = treeData.wheel_pointTree_data[5];

		// 第一级节点(电压等级);
		if (curretnVoltageLevelNode == strVoltageLevelNode)
		{
			if (currentIntervalNode == strIntervalNode)
			{
				if (currentDeviceTypeNode == strDeviceTypeNode)
				{
					PointNameNode = new QStandardItem(m_publicIconMap[QString("tP")], currentDataNode);
				//	PointNameNode = new QStandardItem(currentDataNode);
					PointNameNode->setEditable(false);
					PointNameNode->setCheckable(true);
					DeviceTypeNode->appendRow(PointNameNode);
					
				}
				else
				{
					DeviceTypeNode = new QStandardItem(m_publicIconMap[QString("tP")], currentDeviceTypeNode);
				//	DeviceTypeNode = new QStandardItem(currentDeviceTypeNode);
					DeviceTypeNode->setEditable(false);
					DeviceTypeNode->setCheckable(true);
					IntervalNode->appendRow(DeviceTypeNode);
				}
			}
			else
			{
				IntervalNode = new QStandardItem(m_publicIconMap[QString("tP")], currentIntervalNode);
			//	IntervalNode = new QStandardItem(QString("%1").arg(currentIntervalNode));
				IntervalNode->setEditable(false);
				IntervalNode->setCheckable(true);
				VoltageLevel->appendRow(IntervalNode);
			}
		}
		else
		{
			VoltageLevel = new QStandardItem(m_publicIconMap[QString("tP")], curretnVoltageLevelNode);
			VoltageLevel = new QStandardItem(curretnVoltageLevelNode);
			VoltageLevel->setEditable(false);
			VoltageLevel->setCheckable(true);
			StationName->appendRow(VoltageLevel);
		}

		strVoltageLevelNode = curretnVoltageLevelNode;
		strIntervalNode = currentIntervalNode;
		strDeviceTypeNode = currentDeviceTypeNode;
	}
	return m_ConfirmPatrolModel;
}

WheelRobortDevicesAlarmColorStruct DLWheelPointTreeData::getDeviceAlarmColorFromDB()
{
	WheelRobortDevicesAlarmColorStruct m_devicesAlarmColorStru;
	WHEEL_ROBOT_DB.getWheelDeviceAlarmColorDataDB(m_devicesAlarmColorStru);
	return m_devicesAlarmColorStru;
}

QList<WheelRobortAlarmPathStruct> DLWheelPointTreeData::getNodeAlarmStatusFromDevices(QString m_device_uuid)
{
	QList<WheelRobortAlarmPathStruct> c_alarmPath;
	bool bRet = WHEEL_ROBOT_DB.getNodeAlarmStatusFromDevicesDB(m_device_uuid, c_alarmPath);
	if (bRet)
	{
		QString companyName = WHEEL_STATION_CONFIG.getWheelStationConfigData().at(1).company_name;
		QString stationName = WHEEL_STATION_CONFIG.getWheelStationConfigData().at(1).station_name;
		c_alarmPath[0].alarm_path_name = companyName;
		c_alarmPath[1].alarm_path_name = stationName;
	}
	return c_alarmPath;
}

QStringList DLWheelPointTreeData::getDeviceNodePathFromDevices(QString m_device_uuid)
{
	QStringList c_devicePath;
	bool bRet = WHEEL_ROBOT_DB.getDeviceNodePathFromDevicesDB(m_device_uuid, c_devicePath);
	if (bRet)
	{
		QString companyName = WHEEL_STATION_CONFIG.getWheelStationConfigData().at(1).company_name;
		QString stationName = WHEEL_STATION_CONFIG.getWheelStationConfigData().at(1).station_name;
		c_devicePath.prepend(stationName);
		c_devicePath.prepend(companyName);
	}
	return c_devicePath;
}

void DLWheelPointTreeData::setTreeSave(int choose, WheelPatrolParameter m_wheelPatrolPara)
{
	QList<WheelPointTreeDataStruct> treeDataList;
	if (choose == 0)
	{
		treeDataList = getPointTreeDataFromDB();
	}
	if (choose == 1)
	{
		treeDataList = getPointTreeDataForSearchFromDB(m_wheelPatrolPara);
	}
//	QList<WheelPointTreeDataStruct> treeDataList = m_wheelPointTreeDataStru;
	QString strVoltageLevelNode = "";
	QString strIntervalNode = "";
	QString strDeviceTypeNode = "";

	QList<WheelTreeSaveStruct> m_treeSaveQList1;
	QList<WheelTreeSaveStruct> m_treeSaveQList2;
	QList<WheelTreeSaveStruct> m_treeSaveQList3;
	QList<WheelTreeSaveStruct> m_treeSaveQList4;

	WheelTreeSaveStruct m_treeSave1;
	WheelTreeSaveStruct m_treeSave2;
	WheelTreeSaveStruct m_treeSave3;
	WheelTreeSaveStruct m_treeSave4;

	maxLevel2 = 0;
	maxLevel3 = 0;
	maxLevel4 = 0;
	m_treeSaveQList.clear();

	for (int i = 0; i < treeDataList.count(); i++)
	{
		WheelPointTreeDataStruct treeData = treeDataList.at(i);

		QString curretnVoltageLevelNode = treeData.wheel_pointTree_data[2];//
		QString currentIntervalNode = treeData.wheel_pointTree_data[3];    //
		QString currentDeviceTypeNode = treeData.wheel_pointTree_data[4];  //
		QString currentDataNode = treeData.wheel_pointTree_data[5];

		if (i == treeDataList.count() - 1)
		{
			strVoltageLevelNode = "";
			strIntervalNode = "";
			strDeviceTypeNode = "";
		}
		else
		{
			strVoltageLevelNode = treeDataList[i + 1].wheel_pointTree_data[2];
			strIntervalNode = treeDataList[i + 1].wheel_pointTree_data[3];
			strDeviceTypeNode = treeDataList[i + 1].wheel_pointTree_data[4];
		}
		
		if (curretnVoltageLevelNode == strVoltageLevelNode)
		{
			if (currentIntervalNode == strIntervalNode)
			{
				if (currentDeviceTypeNode == strDeviceTypeNode)
				{
					m_treeSave4.node_name = currentDataNode;
					m_treeSave4.alarm_level = treeData.alarm_level;
					m_treeSave4.node_uuid = treeData.device_uuid;
					setLevelCompare(0, 0, treeData.alarm_level);
					m_treeSaveQList4.append(m_treeSave4);
				}
				else
				{
					m_treeSave4.node_name = currentDataNode;
					m_treeSave4.alarm_level = treeData.alarm_level;
					m_treeSave4.node_uuid = treeData.device_uuid;
					setLevelCompare(0, 0, treeData.alarm_level);
					m_treeSaveQList4.append(m_treeSave4);


					m_treeSave3.node_name = currentDeviceTypeNode;
					m_treeSave3.iter = m_treeSaveQList4;
					m_treeSave3.alarm_level = (DeviceAlarmLevel)maxLevel4;
					setLevelCompare(0, maxLevel4, 0);
					m_treeSaveQList3.append(m_treeSave3);
					m_treeSaveQList4.clear();
					maxLevel4 = 0;
				}
			}
			else 
			{
				m_treeSave4.node_name = currentDataNode;
				m_treeSave4.alarm_level = treeData.alarm_level;
				m_treeSave4.node_uuid = treeData.device_uuid;
				setLevelCompare(0, 0, treeData.alarm_level);
				m_treeSaveQList4.append(m_treeSave4);

				m_treeSave3.node_name = currentDeviceTypeNode;
				m_treeSave3.iter = m_treeSaveQList4;
				m_treeSave3.alarm_level = (DeviceAlarmLevel)maxLevel4;
				setLevelCompare(0, maxLevel4, 0);
				m_treeSaveQList3.append(m_treeSave3);
				m_treeSaveQList4.clear();
				maxLevel4 = 0;

				m_treeSave2.node_name = currentIntervalNode;
				m_treeSave2.iter = m_treeSaveQList3;
				m_treeSave2.alarm_level = (DeviceAlarmLevel)maxLevel3;
				setLevelCompare(maxLevel3, 0, 0);
				m_treeSaveQList2.append(m_treeSave2);
				m_treeSaveQList3.clear();
				maxLevel3 = 0;
			}
		}
		else
		{
			m_treeSave4.node_name = currentDataNode;
			m_treeSave4.alarm_level = treeData.alarm_level;
			m_treeSave4.node_uuid = treeData.device_uuid;
			setLevelCompare(0, 0, treeData.alarm_level);
			m_treeSaveQList4.append(m_treeSave4);

			m_treeSave3.node_name = currentDeviceTypeNode;
			m_treeSave3.iter = m_treeSaveQList4;
			m_treeSave3.alarm_level = (DeviceAlarmLevel)maxLevel4;
			setLevelCompare(0, maxLevel4, 0);
			m_treeSaveQList3.append(m_treeSave3);
			m_treeSaveQList4.clear();
			maxLevel4 = 0;

			m_treeSave2.node_name = currentIntervalNode;
			m_treeSave2.iter = m_treeSaveQList3;
			m_treeSave2.alarm_level = (DeviceAlarmLevel)maxLevel3;
			setLevelCompare(maxLevel3, 0, 0);
			m_treeSaveQList2.append(m_treeSave2);
			m_treeSaveQList3.clear();
			maxLevel3 = 0;

			m_treeSave1.node_name = curretnVoltageLevelNode;
			m_treeSave1.alarm_level = (DeviceAlarmLevel)maxLevel2;
			m_treeSave1.iter = m_treeSaveQList2;
			m_treeSaveQList1.append(m_treeSave1);
			m_treeSaveQList2.clear();
			maxLevel2 = 0;
		}
	}
	m_treeSaveQList = m_treeSaveQList1;
}

void DLWheelPointTreeData::setLevelCompare(int newLevel2, int newLevel3, int newLevel4)
{
	if (newLevel4 > maxLevel4)
	{
		maxLevel4 = newLevel4;
	}
	if (newLevel3 > maxLevel3)
	{
		maxLevel3 = newLevel3;
	}
	if (newLevel2 > maxLevel2)
	{
		maxLevel2 = newLevel2;
	}
}

QList<WheelTreeSaveStruct> DLWheelPointTreeData::getPointTreeRootNode(QStringList m_path)
{
	QList<WheelTreeSaveStruct> m_data;
	int m_choose = m_path.size();
	switch (m_choose)
	{
	case Tree_zeroRootNode:
		m_data = getPointTreeChooseSecond();
		break;
	case Tree_oneRootNode:
		m_data = getPointTreeChooseThird(m_path);
		break;
	case Tree_twoRootNode:
		m_data = getPointTreeChooseFourth(m_path);
		break;
	case Tree_threeRootNode:
		m_data = getPointTreeChooseFifth(m_path);
		break;
	default:
 		break;
	}
	return m_data;
}

QList<WheelTreeSaveStruct> DLWheelPointTreeData::getPointTreeChooseSecond()
{
	QList<WheelTreeSaveStruct> m_data;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		WheelTreeSaveStruct m_tree;
		m_tree.node_name = m_treeSaveQList[i].node_name;
		m_tree.alarm_level = m_treeSaveQList[i].alarm_level;
		m_data.append(m_tree);
	}
	return m_data;
}

QList<WheelTreeSaveStruct> DLWheelPointTreeData::getPointTreeChooseThird(QStringList m_path)
{
	QList<WheelTreeSaveStruct> m_data;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		if (m_treeSaveQList[i].node_name == m_path[0])
		{
			for (int j = 0; j < m_treeSaveQList[i].iter.size(); j++)
			{
				WheelTreeSaveStruct m_tree;
				m_tree.node_name = m_treeSaveQList[i].iter[j].node_name;
				m_tree.alarm_level = m_treeSaveQList[i].iter[j].alarm_level;
				m_data.append(m_tree);
			}
			break;
		}
	}
	return m_data;
}

QList<WheelTreeSaveStruct> DLWheelPointTreeData::getPointTreeChooseFourth(QStringList m_path)
{
	QList<WheelTreeSaveStruct> m_data;
	int m_count_i = 0;
	int m_count_j = 0;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		if (m_treeSaveQList[i].node_name == m_path[0])
		{
			m_count_i = i;
			for (int j = 0; j < m_treeSaveQList[i].iter.size(); j++)
			{
				if (m_treeSaveQList[i].iter[j].node_name == m_path[1])
				{
					m_count_j = j;
					break;
				}
			}
			break;
		}
	}
	for (int i = 0; i < m_treeSaveQList[m_count_i].iter[m_count_j].iter.size(); i++)
	{
		WheelTreeSaveStruct m_tree;
		m_tree.node_name = m_treeSaveQList[m_count_i].iter[m_count_j].iter[i].node_name;
		m_tree.alarm_level = m_treeSaveQList[m_count_i].iter[m_count_j].iter[i].alarm_level;
		m_data.append(m_tree);
	}
	return m_data;
}

QList<WheelTreeSaveStruct> DLWheelPointTreeData::getPointTreeChooseFifth(QStringList m_path)
{
	QList<WheelTreeSaveStruct> m_data;
	int m_count_i = 0;
	int m_count_j = 0;
	int m_count_k = 0;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		if (m_treeSaveQList[i].node_name == m_path[0])
		{
			m_count_i = i;
			for (int j = 0; j < m_treeSaveQList[i].iter.size(); j++)
			{
				if (m_treeSaveQList[i].iter[j].node_name == m_path[1])
				{
					m_count_j = j;
					for (int k = 0; k < m_treeSaveQList[i].iter[j].iter.size(); k++)
					{
						if (m_treeSaveQList[i].iter[j].iter[k].node_name == m_path[2])
						{
							m_count_k = k;
							break;
						}
					}
					break;
				}
			}
			break;
		}
	}
	for (int i = 0; i < m_treeSaveQList[m_count_i].iter[m_count_j].iter[m_count_k].iter.size(); i++)
	{
		WheelTreeSaveStruct m_tree;
		m_tree.node_name = m_treeSaveQList[m_count_i].iter[m_count_j].iter[m_count_k].iter[i].node_name;
		m_tree.alarm_level = m_treeSaveQList[m_count_i].iter[m_count_j].iter[m_count_k].iter[i].alarm_level;
		m_tree.node_uuid = m_treeSaveQList[m_count_i].iter[m_count_j].iter[m_count_k].iter[i].node_uuid;
		m_data.append(m_tree);
	}
	return m_data;
}

void DLWheelPointTreeData::getPointTreeFirstRootNode(QString &m_name, int &m_level)
{
	m_level = 0;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		if (m_treeSaveQList[i].alarm_level > m_level)
		{
			m_level = m_treeSaveQList[i].alarm_level;
		}
	}
	m_name = WHEEL_STATION_CONFIG.getWheelStationConfigData()[1].company_name;
}

void DLWheelPointTreeData::getPointTreeSecondRootNode(QString &m_name, int &m_level)
{
	m_level = 0;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		if (m_treeSaveQList[i].alarm_level > m_level)
		{
			m_level = m_treeSaveQList[i].alarm_level;
		}
	}
	m_name = WHEEL_STATION_CONFIG.getWheelStationConfigData()[1].station_name;
}

void DLWheelPointTreeData::updataTree()
{
	WheelPatrolParameter m_wheelPatrolPara;
	setTreeSave(0, m_wheelPatrolPara);
}

void DLWheelPointTreeData::updataTree(WheelPatrolParameter m_wheelPatrolPara)
{
//YY	setTreeSave(1, m_wheelPatrolPara);
}

/////////////////////////////////////////////////////////////////////////////////////
QList<QString> DLWheelPointTreeData::getDeviceUUidForRootNode(QStringList m_path)
{
	QList<QString> m_data;
	int m_choose = m_path.size();
	switch (m_choose)
	{
	case Tree_zeroRootNode:
		m_data = getDeviceUUidChooseSecond();
		break;
	case Tree_oneRootNode:
		m_data = getDeviceUUidChooseThird(m_path);
		break;
	case Tree_twoRootNode:
		m_data = getDeviceUUidChooseFourth(m_path);
		break;
	case Tree_threeRootNode:
		m_data = getDeviceUUidChooseFifth(m_path);
		break;
	default:
		break;
	}
	return m_data;
}

QList<QString> DLWheelPointTreeData::getDeviceUUidChooseSecond()
{
	QList<QString> m_data;

	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		for (int j = 0; j < m_treeSaveQList[i].iter.size(); j++)
		{
			for (int k = 0; k < m_treeSaveQList[i].iter[j].iter.size(); k++)
			{
				for (int p = 0; p < m_treeSaveQList[i].iter[j].iter[k].iter.size(); p++)
				{
					m_data.append(m_treeSaveQList[i].iter[j].iter[k].iter[p].node_uuid);
				}
			}
		}
	}
	return m_data;
}

QList<QString> DLWheelPointTreeData::getDeviceUUidChooseThird(QStringList m_path)
{
	QList<QString> m_data;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		if (m_treeSaveQList[i].node_name == m_path[0])
		{
			for (int j = 0; j < m_treeSaveQList[i].iter.size(); j++)
			{
				for (int k = 0; k < m_treeSaveQList[i].iter[j].iter.size(); k++)
				{
					for (int p = 0; p < m_treeSaveQList[i].iter[j].iter[k].iter.size(); p++)
					{
						m_data.append(m_treeSaveQList[i].iter[j].iter[k].iter[p].node_uuid);
					}
				}
			}
			break;
		}
	}
	return m_data;
}

QList<QString> DLWheelPointTreeData::getDeviceUUidChooseFourth(QStringList m_path)
{
	QList<QString> m_data;
	int m_count_i = 0;
	int m_count_j = 0;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		if (m_treeSaveQList[i].node_name == m_path[0])
		{
			m_count_i = i;
			for (int j = 0; j < m_treeSaveQList[i].iter.size(); j++)
			{
				if (m_treeSaveQList[i].iter[j].node_name == m_path[1])
				{
					m_count_j = j;
					break;
				}
			}
			break;
		}
	}
	for (int i = 0; i < m_treeSaveQList[m_count_i].iter[m_count_j].iter.size(); i++)
	{
		for (int p = 0; p < m_treeSaveQList[m_count_i].iter[m_count_j].iter[i].iter.size(); p++)
		{
			m_data.append(m_treeSaveQList[m_count_i].iter[m_count_j].iter[i].iter[p].node_uuid);
		}
	}
	return m_data;
}

QList<QString> DLWheelPointTreeData::getDeviceUUidChooseFifth(QStringList m_path)
{
	QList<QString> m_data;
	int m_count_i = 0;
	int m_count_j = 0;
	int m_count_k = 0;
	for (int i = 0; i < m_treeSaveQList.size(); i++)
	{
		if (m_treeSaveQList[i].node_name == m_path[0])
		{
			m_count_i = i;
			for (int j = 0; j < m_treeSaveQList[i].iter.size(); j++)
			{
				if (m_treeSaveQList[i].iter[j].node_name == m_path[1])
				{
					m_count_j = j;
					for (int k = 0; k < m_treeSaveQList[i].iter[j].iter.size(); k++)
					{
						if (m_treeSaveQList[i].iter[j].iter[k].node_name == m_path[2])
						{
							m_count_k = k;
							break;
						}
					}
					break;
				}
			}
			break;
		}
	}
	for (int i = 0; i < m_treeSaveQList[m_count_i].iter[m_count_j].iter[m_count_k].iter.size(); i++)
	{
		m_data.append(m_treeSaveQList[m_count_i].iter[m_count_j].iter[m_count_k].iter[i].node_uuid);
	}
	return m_data;
}

void DLWheelPointTreeData::setTreeConditionForSearchData(WheelPatrolParameter m_wheelPatrolPara)
{
	m_treeSerachQMap.clear();
	QList<WheelPointTreeDataStruct> m_wheelPointTreeDataStru;
	bool bRet = WHEEL_ROBOT_DB.getWheelPointTreeDataDB(m_wheelPointTreeDataStru, m_wheelPatrolPara);
	for (auto _it : m_wheelPointTreeDataStru)
	{
		m_treeSerachQMap.insert(_it.device_uuid, _it);
	}
}

bool DLWheelPointTreeData::isExiThisNode(int nodeID, QString stdUUid)
{
	return true;
}

QList<QStringList> DLWheelPointTreeData::getCollectTreeDataFromDB(QStringList selectUUid, RootNodeType selectType)
{
	QList<QStringList> m_collectTreeDat;
	bool bRet = WHEEL_ROBOT_DB.getNewWheelCollectTreeDataDB(m_collectTreeDat, selectUUid, selectType);
	return m_collectTreeDat;
}
