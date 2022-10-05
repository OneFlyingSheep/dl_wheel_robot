#ifndef DL_ADD_EQUIPMENT_TREE_WGT_H
#define DL_ADD_EQUIPMENT_TREE_WGT_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QToolButton>
#include <QLineEdit>
#include <QMenu>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"
#include "AddEquipTreeDelegate.h"

#pragma execution_character_set("utf-8")

class CollectEquipMentWgt;
class DLWheelPointTreeData;
class AddTypeAndEquipmentDlg;
class BaseWidget;
class InputWidget;
enum RootNodeType;

class AddEquipmentTreeWgt : public QWidget
{
	Q_OBJECT

	enum INTERVAL_TYPE 
	{
		ADD_INTERVAL,					//添加间隔
		RENAME_INTERVAL,				//重命名间隔
		PASTE_INTERVAL,					//粘贴间隔
		INTERVAL_TYPE_NUM
	};

public:
	AddEquipmentTreeWgt(QWidget* parent = NULL);
	~AddEquipmentTreeWgt();
	void UpdateTreeNode(QStringList lstUpdateTreeNode, RootNodeType iType);				//更新树节点信息

private:
	void initWidget();								//初始化界面
	void InitModel();								//初始化model数据
	//QStandardItem * CheckAddItem(QStandardItem *pParentItem, const QString &strPath, int iLevel, int iType, QString strUUid = "");  //检查路径的item是否存在，不存在添加item

	void LoadEquipmentItem(const QList<QStringList> &lstEquipments);								//加载设备item
	void InitAddTypeAndEquipmentDlg();																//初始化添加设备类型和设备的弹窗
	void InitEditIntervalDlg();

signals:
	// 通知刷新树;
	//void signalRefreshTreeWidget();
	// 通知删除树节点;
	//void signalDeleteTreeItemNode(RootNodeType, QString, QString);
	// 通知修改树节点;
	void signalModifyDeviceInfo(QString strDeviceId, QString strPointId);
	// 通知移动至该点;
	//void signalMoveToDevicePoint(QString strDeviceId);
	// 树控件是否收缩;
	void signalTreeWidgetShrink(bool isShrink);
	//void CollectEquipmentSignal(QString strDeviceID);				//收集设备

private slots:
	void CustomMenuSlot(const QPoint &pos);												//自定义菜单的槽函数

	//////////////////////////////////////////////////////////////////////////菜单
	void TreeMenuActionSlot(QAction *pAction);											//菜单的槽函数
	void EquipmentItemDoubleClickedSlot(const QModelIndex &index);						//设备节点双击槽函数

	//////////////////////////////////////////////////////////////////////////添加类型和设备
	void AddEquipmentTypeOkBtnSlot();								//确定按钮
	void AddEquipmentTypeCancelBtnSlot();							//取消按钮


	//////////////////////////////////////////////////////////////////////////添加间隔名
	void EditIntervalOKBtnSlot();
	void EditIntervalCancelBtnSlot();
private:
	//////////////////////////////////////////////////////////////////////////
	QList<QStringList> GetSelectDatas();				//获取选中的数据
	QStringList GetItemDatas(QStandardItem *pItem);				//获取选中的数据
	void RemoveChildItem(QStandardItem *pCurItem, QString strCurItemID);				//移除子节点

private:
	QWidget * m_titleBackWidget;
	QLabel* m_titleLabel;
	QPushButton* m_pButtonRefreshTree;
	QPushButton* m_pButtonShrink;
	QPushButton* m_pButtonPop;

	QWidget* m_searchBackWidget;
	QLineEdit* m_searchLineEdit;
	QToolButton* m_pButtonDeviceTreeSearch;

	QWidget* m_mainBackWidget;
	QWidget* m_popButtonBackWidget;

	DLWheelPointTreeData* m_wheelPointTreeData;				// 树控件数据查询对象;
	//QMap<QString, QString> m_choosedDeviceIdMap;			// 保存当前选中的设备节点ID;
	//QMap<QString, QStandardItem *> m_mapUid2Item;			//记录所有的末尾节点

	//===================================================================
	QTreeView *m_pTreeView;									//树的view
	QStandardItemModel *m_pTreeModel;						//树的model
	AddEquipTreeDelegate *m_pTreeDelegate;						//树节点代理
	QMenu *m_pTreeMenu;										//树节点菜单
	QStandardItem *m_pRootStandardItem;						//根节点
	QStandardItem *m_pSecondStandardItem;					//第二层节点
	QMap<QString, QStandardItem *> m_mapUid2Items;			//记录所有节点，包括电压等级、间隔、类型、设备

	BaseWidget *m_pAddTypeAndEquipmentBaseWgt;					//添加设备类型及设备
	AddTypeAndEquipmentDlg *m_pAddTypeAndEquipmentDlg;			

	BaseWidget *m_pAddIntervalBaseWgt;							//添加及重命名间隔
	InputWidget *m_pEditIntervalWgt;
	int m_iCurIntervalType;
	QModelIndex m_curModelIndex;

	//////////////////////////////////////////////////////////////////////////
	//复制/粘贴间隔的变量
	bool m_bIsCopyed;
	//QString m_strCopyIntervalID;
	QStringList m_lstCopyIntervalID;
};

#endif // !DL_ADD_EQUIPMENT_TREE_WGT_H
