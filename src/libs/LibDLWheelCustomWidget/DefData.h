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
{//checkbox的状态
	UnChecked_Status,									//未选中状态
	PartiallyChecked_Status,							//半选状态
	Checked_Status										//选中状态
};




#define ITEM_UUID (Qt::UserRole + 1)					//记录节点的ID
#define ITEM_TYPE (Qt::UserRole + 2)					//记录节点的等级(一级节点，二级节点...)
#define ITEM_LEVEL (Qt::UserRole + 3)					//记录节点的电压等级
#define ITEM_TEXT (Qt::UserRole + 4)					//记录节点的文本
#define ITEM_STATE (Qt::UserRole + 5)					//记录节点的选中状态
#define ITEM_DEVICE (Qt::UserRole + 6)					//device 的 uid



enum ACTION_TYPE 
{
	ADD_INTERVAL_ACTION,						//添加间隔
	COPY_INTERVAL_ACTION,						//复制间隔 
	PASTE_INTERVAL_ACTION,						//粘贴间隔
	DEL_INTERVAL_ACTION,						//删除间隔
	RENAME_INTERVAL_ACTION,						//重命名间隔名
	ADD_EQUIPMENT_TYPE_ACTION,					//添加设备类型
	ADD_EQUIPMENT_AND_TYPE_ACTION,				//添加设备及设备类型
	DEL_EQUIPMENT_TYPE_ACTION,					//删除设备类型
	ADD_EQUIPMENT_ACTION,						//添加设备
	DEL_EQUIPMENT_ACTION,						//删除设备
	RUN_POINT_ACTION,							//行走到该点
	ACTION_NUM
};

enum EQUIPMENT_INFO
{
	VOLTAGE_LEVEL_UID,							//电压等级 uid
	VOLTAGE_LEVEL_NAME,							//电压等级 名字
	INTERVAL_UID,								//间隔  uid
	INTERVAL_NAME,								//间隔  name
	TYPE_UID,									//类型 uid
	TYPE_NAME,									//类型  name
	EQUIPMENT_UID,								//设备 uid
	EQUIPMENT_NAME,								//设备 name
	DEVICE_UID,									//device uid
	EQUIPMENT_STATE,							//设备状态
	EQUIPMENT_INFO_NUM
};


#endif // !DEF_DATA_H
