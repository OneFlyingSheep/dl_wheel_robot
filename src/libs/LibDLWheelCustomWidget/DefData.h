#ifndef DEF_DATA_H
#define DEF_DATA_H


enum TreeItemWidgetType
{
	ColorRect_CheckBox_With,
	ColorRect_CheckBox_Without,
    ColorRect_Menu_With,
	FolderRect,
	FolderRect_CheckBox_With,
};

struct TreeItemInfo
{
	QString stationName;
	QString deviceArea;
	QString deviceType;
	QString deviceName;
	TreeItemInfo(QString stationName, QString deviceArea, QString deviceType, QString deviceName)
	{
		this->stationName = stationName;
		this->deviceArea = deviceArea;
		this->deviceType = deviceType;
		this->deviceName = deviceName;
	}
};

enum ItemCheckStatus
{//checkbox��״̬
	UnChecked_Status,									//δѡ��״̬
	PartiallyChecked_Status,							//��ѡ״̬
	Checked_Status										//ѡ��״̬
};




#define ITEM_UUID (Qt::UserRole + 1)					//��¼�ڵ��ID
#define ITEM_TYPE (Qt::UserRole + 2)					//��¼�ڵ�ĵȼ�(һ���ڵ㣬�����ڵ�...)
#define ITEM_LEVEL (Qt::UserRole + 3)					//��¼�ڵ�ĵ�ѹ�ȼ�
#define ITEM_TEXT (Qt::UserRole + 4)					//��¼�ڵ���ı�
#define ITEM_STATE (Qt::UserRole + 5)					//��¼�ڵ��ѡ��״̬
#define ITEM_DEVICE (Qt::UserRole + 6)					//device �� uid



enum ACTION_TYPE 
{
	ADD_INTERVAL_ACTION,						//��Ӽ��
	COPY_INTERVAL_ACTION,						//���Ƽ�� 
	PASTE_INTERVAL_ACTION,						//ճ�����
	DEL_INTERVAL_ACTION,						//ɾ�����
	RENAME_INTERVAL_ACTION,						//�����������
	ADD_EQUIPMENT_TYPE_ACTION,					//����豸����
	ADD_EQUIPMENT_AND_TYPE_ACTION,				//����豸���豸����
	DEL_EQUIPMENT_TYPE_ACTION,					//ɾ���豸����
	ADD_EQUIPMENT_ACTION,						//����豸
	DEL_EQUIPMENT_ACTION,						//ɾ���豸
	RUN_POINT_ACTION,							//���ߵ��õ�
	ACTION_NUM
};

enum EQUIPMENT_INFO
{
	VOLTAGE_LEVEL_UID,							//��ѹ�ȼ� uid
	VOLTAGE_LEVEL_NAME,							//��ѹ�ȼ� ����
	INTERVAL_UID,								//���  uid
	INTERVAL_NAME,								//���  name
	TYPE_UID,									//���� uid
	TYPE_NAME,									//����  name
	EQUIPMENT_UID,								//�豸 uid
	EQUIPMENT_NAME,								//�豸 name
	DEVICE_UID,									//device uid
	EQUIPMENT_STATE,							//�豸״̬
	EQUIPMENT_INFO_NUM
};


#endif // !DEF_DATA_H
