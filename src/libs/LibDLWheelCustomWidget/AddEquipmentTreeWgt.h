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
		ADD_INTERVAL,					//��Ӽ��
		RENAME_INTERVAL,				//���������
		PASTE_INTERVAL,					//ճ�����
		INTERVAL_TYPE_NUM
	};

public:
	AddEquipmentTreeWgt(QWidget* parent = NULL);
	~AddEquipmentTreeWgt();
	void UpdateTreeNode(QStringList lstUpdateTreeNode, RootNodeType iType);				//�������ڵ���Ϣ

private:
	void initWidget();								//��ʼ������
	void InitModel();								//��ʼ��model����
	//QStandardItem * CheckAddItem(QStandardItem *pParentItem, const QString &strPath, int iLevel, int iType, QString strUUid = "");  //���·����item�Ƿ���ڣ����������item

	void LoadEquipmentItem(const QList<QStringList> &lstEquipments);								//�����豸item
	void InitAddTypeAndEquipmentDlg();																//��ʼ������豸���ͺ��豸�ĵ���
	void InitEditIntervalDlg();

signals:
	// ֪ͨˢ����;
	//void signalRefreshTreeWidget();
	// ֪ͨɾ�����ڵ�;
	//void signalDeleteTreeItemNode(RootNodeType, QString, QString);
	// ֪ͨ�޸����ڵ�;
	void signalModifyDeviceInfo(QString strDeviceId, QString strPointId);
	// ֪ͨ�ƶ����õ�;
	//void signalMoveToDevicePoint(QString strDeviceId);
	// ���ؼ��Ƿ�����;
	void signalTreeWidgetShrink(bool isShrink);
	//void CollectEquipmentSignal(QString strDeviceID);				//�ռ��豸

private slots:
	void CustomMenuSlot(const QPoint &pos);												//�Զ���˵��Ĳۺ���

	//////////////////////////////////////////////////////////////////////////�˵�
	void TreeMenuActionSlot(QAction *pAction);											//�˵��Ĳۺ���
	void EquipmentItemDoubleClickedSlot(const QModelIndex &index);						//�豸�ڵ�˫���ۺ���

	//////////////////////////////////////////////////////////////////////////������ͺ��豸
	void AddEquipmentTypeOkBtnSlot();								//ȷ����ť
	void AddEquipmentTypeCancelBtnSlot();							//ȡ����ť


	//////////////////////////////////////////////////////////////////////////��Ӽ����
	void EditIntervalOKBtnSlot();
	void EditIntervalCancelBtnSlot();
private:
	//////////////////////////////////////////////////////////////////////////
	QList<QStringList> GetSelectDatas();				//��ȡѡ�е�����
	QStringList GetItemDatas(QStandardItem *pItem);				//��ȡѡ�е�����
	void RemoveChildItem(QStandardItem *pCurItem, QString strCurItemID);				//�Ƴ��ӽڵ�

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

	DLWheelPointTreeData* m_wheelPointTreeData;				// ���ؼ����ݲ�ѯ����;
	//QMap<QString, QString> m_choosedDeviceIdMap;			// ���浱ǰѡ�е��豸�ڵ�ID;
	//QMap<QString, QStandardItem *> m_mapUid2Item;			//��¼���е�ĩβ�ڵ�

	//===================================================================
	QTreeView *m_pTreeView;									//����view
	QStandardItemModel *m_pTreeModel;						//����model
	AddEquipTreeDelegate *m_pTreeDelegate;						//���ڵ����
	QMenu *m_pTreeMenu;										//���ڵ�˵�
	QStandardItem *m_pRootStandardItem;						//���ڵ�
	QStandardItem *m_pSecondStandardItem;					//�ڶ���ڵ�
	QMap<QString, QStandardItem *> m_mapUid2Items;			//��¼���нڵ㣬������ѹ�ȼ�����������͡��豸

	BaseWidget *m_pAddTypeAndEquipmentBaseWgt;					//����豸���ͼ��豸
	AddTypeAndEquipmentDlg *m_pAddTypeAndEquipmentDlg;			

	BaseWidget *m_pAddIntervalBaseWgt;							//��Ӽ����������
	InputWidget *m_pEditIntervalWgt;
	int m_iCurIntervalType;
	QModelIndex m_curModelIndex;

	//////////////////////////////////////////////////////////////////////////
	//����/ճ������ı���
	bool m_bIsCopyed;
	//QString m_strCopyIntervalID;
	QStringList m_lstCopyIntervalID;
};

#endif // !DL_ADD_EQUIPMENT_TREE_WGT_H
