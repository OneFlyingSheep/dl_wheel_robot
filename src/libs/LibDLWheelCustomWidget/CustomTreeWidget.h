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
	void setTreeWidgetType(TreeItemWidgetType type);									// �������ؼ�����;
	QString getItemDeviceId(QString itemText, RootNodeType rootNodeType);				// ��ȡitem��deviceId
	QStandardItem *getCurrentSelectedItem();											//��ȡѡ�нڵ�
	void updataTreeData(WheelPatrolParameter m_wheelPatrolPara);						// �������ؼ�����;
	void setSearchLineEditVisible(bool isVisible);										// �����Ƿ���ʾ������;
	void setTitleWidgetVisible(bool isVisible);											// �����Ƿ���ʾtitleBackWidget;
	void setDevelopTreeWidget();														// �����Ƿ��ǲɼ��������豸��;
	QStringList getChoosedDeviceIdList();												// ��ȡ����ѡ�е��豸id List;
    void setIsShrink(bool isShrink);													// �����Ƿ��������;
    void expandTreeNodeByDevice(QStringList strDevicePath, QString strDeviceId);		// ����ĳ��deviceչ����item�������нڵ�;
	void ClearAllItemState();															//������нڵ��״̬

public slots:
	void addItemToTree(QString strDeviceId, QList<WheelRobortAlarmPathStruct> treeItemData);	// ����һ���ڵ�����;
	void onDeleteItem();																		// ɾ���ڵ�;
    void refreshTreeItemList(WheelPatrolParameter wheelPatrolPara);								// ˢ�����ؼ�;
	void refreshTreeItemList();

private:
	void initWidget();								//��ʼ������
	void InitModel();								//��ʼ��model����
	void LoadChildItems(QStandardItem *pParentItem, QList<WheelTreeSaveStruct> lstChildInfo, int iChildType);		//�����ӽڵ�
	//void updateAlarmLevel(int currentLevel);
	QStandardItem * CheckAddItem(QStandardItem *pParentItem, const QString &strPath, int iLevel, int iType, QString strUUid = "");  //���·����item�Ƿ���ڣ����������item

signals:
	// ֪ͨˢ����;
	void signalRefreshTreeWidget();
	// ֪ͨɾ�����ڵ�;
	void signalDeleteTreeItemNode(RootNodeType, QString, QString);
	// ֪ͨ�޸����ڵ�;
	void signalModifyDeviceInfo(QString strDeviceId, QString strDeviceName);
	// ֪ͨ�ƶ����õ�;
	void signalMoveToDevicePoint(QString strDeviceId);
	// ���ؼ��Ƿ�����;
	void signalTreeWidgetShrink(bool isShrink);
    // �豸�ڵ���;
    void signalDeviceNodeClicked(QString strDeviceId);

private slots:
	void ItemStateChangeSlot(const QModelIndex &index);									//check״̬�����ı�Ĳۺ���
	void ItemChangedSlot(QStandardItem *pTreeItem);										//�ڵ�dataֵ�����ı�Ĳۺ���
	void CustomMenuSlot(const QPoint &pos);												//�Զ���˵��Ĳۺ���
	void UpdateTreeItemLevelSlot(QString strUUid, DeviceAlarmLevel iLevel);				//�������ڵ�level��ֵ

private:
	void UpdateChildState(QStandardItem *pParentItem);									//�����ӽڵ��״̬
	void UpdateParentState(QStandardItem *pCurItem);									//���¸��ڵ��״̬
	void ClearChildState(QStandardItem *pParnetItem);									//����ӽڵ��״̬
	void ChangedItemState(QStack<QStandardItem *> &stackItems, QStandardItem *pParentItem);   //�����޸�ÿ���ڵ��״̬
	void InitItemWidgetType(TreeItemWidgetType type);									//��ʼ���Ƿ���checkbox�Ͳ˵�
	void ExpandSelectItem(QStringList lstPath, int index, QStandardItem *pParentItem, QString strUUid = "");		//չ��ѡ�еĽڵ�
	void UpdateParentLevel(QStandardItem *pChildItem);			//���¸��ڵ��level
private:
	QWidget * m_titleBackWidget;
	QLabel* m_titleLabel;
	QPushButton* m_pButtonRefreshTree;
	QPushButton* m_pButtonShrink;
	QPushButton* m_pButtonPop;

	QWidget* m_searchBackWidget;
	QLineEdit* m_searchLineEdit;
	QToolButton* m_pButtonDeviceTreeSearch;

	// ���ؼ�������;
	TreeItemWidgetType m_treeWidgetType;

	int m_voltageLevelAlarmLevel;
	int m_intervalAlarmLevel;
	int m_deviceTypeAlarmLevel;

	// ��ǰ�Ƿ��ǲɼ������е����ؼ�;
	bool m_isDevelopTreeWidget;

	QWidget* m_mainBackWidget;
	QWidget* m_popButtonBackWidget;

	DLWheelPointTreeData* m_wheelPointTreeData;				// ���ؼ����ݲ�ѯ����;
	//QMap<QString, QString> m_choosedDeviceIdMap;			// ���浱ǰѡ�е��豸�ڵ�ID;
	QTreeView *m_pTreeView;									//����view
	QStandardItemModel *m_pTreeModel;						//����model
	QMap<QString, QStandardItem *> m_mapUid2Item;			//��¼���е�ĩβ�ڵ�
	bool m_bIsExistCheckBox;								//��¼�Ƿ����checkbox
	bool m_bIsShowMenu;										// ��ǰ�Ƿ���ʾ�Ҽ��˵�(��Ҫ����վ��͹�˾���ƽڵ㣬����Ҫ�Ҽ��˵�);
	EquipTreeDelegate *m_pTreeDelegate;						//���ڵ����
	QMenu *m_pTreeMenu;										//���ڵ�˵�
};

#endif // !DL_CUSTOM_TREE_WIDGET_H
