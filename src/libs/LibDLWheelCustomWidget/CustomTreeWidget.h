#ifndef DL_CUSTOM_TREE_WIDGET_H
#define DL_CUSTOM_TREE_WIDGET_H

#include <QWidget>
#include <QTreeWidget>
#include <QLabel>
#include <QCheckBox>
#include <QHBoxLayout>
#include <QPushButton>
#include <QToolButton>
#include <QLineEdit>
#include "common/DLWheelRobotGlobalDef.hpp"
#include <QPainter>
#include <QMouseEvent>
#include <QMenu>
#include <QApplication>
#include <QClipboard>
#include <QThread>
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"
#include "EquipTreeDelegate.h"

#pragma execution_character_set("utf-8")

class DLWheelPointTreeData;
enum TreeItemWidgetType;
enum RootNodeType;
struct TreeItemInfo;

class CustomTreeWidget : public QWidget
{
	Q_OBJECT

public:
	CustomTreeWidget(QWidget* parent = NULL);
	~CustomTreeWidget();
	void setTreeWidgetType(TreeItemWidgetType type);									// 设置树控件类型;
	QString getItemDeviceId(QString itemText, RootNodeType rootNodeType);				// 获取item的deviceId
	QStandardItem *getCurrentSelectedItem();											//获取选中节点
	void updataTreeData(WheelPatrolParameter m_wheelPatrolPara);						// 更新树控件数据;
	void setSearchLineEditVisible(bool isVisible);										// 设置是否显示搜索框;
	void setTitleWidgetVisible(bool isVisible);											// 设置是否显示titleBackWidget;
	void setDevelopTreeWidget();														// 设置是否是采集工具中设备树;
	QStringList getChoosedDeviceIdList();												// 获取所有选中的设备id List;
    void setIsShrink(bool isShrink);													// 设置是否可以收缩;
    void expandTreeNodeByDevice(QStringList strDevicePath, QString strDeviceId);		// 根据某个device展开该item所在所有节点;
	void ClearAllItemState();															//清空所有节点的状态

public slots:
	void addItemToTree(QString strDeviceId, QList<WheelRobortAlarmPathStruct> treeItemData);	// 插入一条节点数据;
	void onDeleteItem();																		// 删除节点;
    void refreshTreeItemList(WheelPatrolParameter wheelPatrolPara);								// 刷新树控件;
	void refreshTreeItemList();

private:
	void initWidget();								//初始化界面
	void InitModel();								//初始化model数据
	void LoadChildItems(QStandardItem *pParentItem, QList<WheelTreeSaveStruct> lstChildInfo, int iChildType);		//加载子节点
	//void updateAlarmLevel(int currentLevel);
	QStandardItem * CheckAddItem(QStandardItem *pParentItem, const QString &strPath, int iLevel, int iType, QString strUUid = "");  //检查路径的item是否存在，不存在添加item

signals:
	// 通知刷新树;
	void signalRefreshTreeWidget();
	// 通知删除树节点;
	void signalDeleteTreeItemNode(RootNodeType, QString, QString);
	// 通知修改树节点;
	void signalModifyDeviceInfo(QString strDeviceId, QString strDeviceName);
	// 通知移动至该点;
	void signalMoveToDevicePoint(QString strDeviceId);
	// 树控件是否收缩;
	void signalTreeWidgetShrink(bool isShrink);
    // 设备节点点击;
    void signalDeviceNodeClicked(QString strDeviceId);

private slots:
	void ItemStateChangeSlot(const QModelIndex &index);									//check状态发生改变的槽函数
	void ItemChangedSlot(QStandardItem *pTreeItem);										//节点data值发生改变的槽函数
	void CustomMenuSlot(const QPoint &pos);												//自定义菜单的槽函数
	void UpdateTreeItemLevelSlot(QString strUUid, DeviceAlarmLevel iLevel);				//更新树节点level的值

private:
	void UpdateChildState(QStandardItem *pParentItem);									//更新子节点的状态
	void UpdateParentState(QStandardItem *pCurItem);									//更新父节点的状态
	void ClearChildState(QStandardItem *pParnetItem);									//清空子节点的状态
	void ChangedItemState(QStack<QStandardItem *> &stackItems, QStandardItem *pParentItem);   //遍历修改每个节点的状态
	void InitItemWidgetType(TreeItemWidgetType type);									//初始化是否有checkbox和菜单
	void ExpandSelectItem(QStringList lstPath, int index, QStandardItem *pParentItem, QString strUUid = "");		//展开选中的节点
	void UpdateParentLevel(QStandardItem *pChildItem);			//更新父节点的level
private:
	QWidget * m_titleBackWidget;
	QLabel* m_titleLabel;
	QPushButton* m_pButtonRefreshTree;
	QPushButton* m_pButtonShrink;
	QPushButton* m_pButtonPop;

	QWidget* m_searchBackWidget;
	QLineEdit* m_searchLineEdit;
	QToolButton* m_pButtonDeviceTreeSearch;

	// 树控件的类型;
	TreeItemWidgetType m_treeWidgetType;

	int m_voltageLevelAlarmLevel;
	int m_intervalAlarmLevel;
	int m_deviceTypeAlarmLevel;

	// 当前是否是采集工具中的树控件;
	bool m_isDevelopTreeWidget;

	QWidget* m_mainBackWidget;
	QWidget* m_popButtonBackWidget;

	DLWheelPointTreeData* m_wheelPointTreeData;				// 树控件数据查询对象;
	//QMap<QString, QString> m_choosedDeviceIdMap;			// 保存当前选中的设备节点ID;
	QTreeView *m_pTreeView;									//树的view
	QStandardItemModel *m_pTreeModel;						//树的model
	QMap<QString, QStandardItem *> m_mapUid2Item;			//记录所有的末尾节点
	bool m_bIsExistCheckBox;								//记录是否存在checkbox
	bool m_bIsShowMenu;										// 当前是否显示右键菜单(主要用于站点和公司名称节点，不需要右键菜单);
	EquipTreeDelegate *m_pTreeDelegate;						//树节点代理
	QMenu *m_pTreeMenu;										//树节点菜单
};

#endif // !DL_CUSTOM_TREE_WIDGET_H
