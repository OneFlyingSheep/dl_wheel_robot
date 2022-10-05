#include "AddEquipmentTreeWgt.h"
#include <QHeaderView>
#include <QDebug>
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPointTreeData.h"
#include "DefData.h"
#include "AddTypeAndEquipmentDlg.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"

AddEquipmentTreeWgt::AddEquipmentTreeWgt(QWidget* parent /* = NULL */)
	: QWidget(parent)
	, m_pTreeView(NULL)
	, m_pTreeDelegate(NULL)
	, m_pTreeMenu(NULL)
	, m_pAddTypeAndEquipmentDlg(NULL)
	, m_pAddTypeAndEquipmentBaseWgt(NULL)
	, m_iCurIntervalType(-1)
	, m_bIsCopyed(false)
{
	m_mapUid2Items.clear();
	m_wheelPointTreeData = new DLWheelPointTreeData;
	initWidget();
	InitModel();
	InitAddTypeAndEquipmentDlg();				//��ʼ����豸���ͺ��豸�ĵ���
	InitEditIntervalDlg();						//��ʼ���������

	this->setStyleSheet("QTreeView{\
									border: 1px solid lightgray;\
									}\
									QTreeView::item {\
											height: 25px;\
											border: none;\
											background: transparent;\
											color: black;\
											outline:none;\
									}\
									QTreeView::item:hover{\
											background: rgba(33, 150, 243, 80);\
									}\
                                    QTreeView::item:selected{\
											background: rgba(33, 150, 243, 150);;\
									}\
									QTreeView::branch:open:has-children {\
											image: url(:/Resources/Common/image/branch_Open.png);\
									}\
									QTreeView::branch:closed:has-children {\
											image: url(:/Resources/Common/image/branch_Close.png);\
									}\
							QWidget#ButtonBackWidget{background:rgb(175, 191, 255);border:1px solid rgb(166, 233, 210);}\
							QWidget#SearchLineEditWidget{background:rgb(175, 191, 255);border:none;}");

	this->setFixedWidth(300);
	this->setFocusPolicy(Qt::NoFocus);
	this->setAttribute(Qt::WA_TranslucentBackground);

}

AddEquipmentTreeWgt::~AddEquipmentTreeWgt()
{
	delete m_wheelPointTreeData;
}

void AddEquipmentTreeWgt::initWidget()
{
	m_pTreeView = new QTreeView(this);
	m_pTreeView->setHeaderHidden(true);
	m_pTreeView->setFocusPolicy(Qt::NoFocus);
	m_pTreeView->setContextMenuPolicy(Qt::CustomContextMenu);
	m_pTreeView->setEditTriggers(QAbstractItemView::NoEditTriggers);
	connect(m_pTreeView, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(CustomMenuSlot(const QPoint &)));

	m_titleBackWidget = new QWidget;
	m_titleBackWidget->setObjectName("ButtonBackWidget");
	m_titleBackWidget->setFixedHeight(30);

	m_titleLabel = new QLabel;
	m_titleLabel->setText("�豸��");
	m_titleLabel->setStyleSheet("font-weight:bold;");

// 	m_pButtonRefreshTree = new QPushButton("ˢ��");
// 	m_pButtonRefreshTree->setFixedSize(QSize(60, 25));
// 	m_pButtonRefreshTree->setStyleSheet("QPushButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
// 						QPushButton:hover{background-color:rgb(44 , 137 , 255);}\
// 						QPushButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");
    m_pButtonRefreshTree = new QPushButton();
    m_pButtonRefreshTree->setStyleSheet("QPushButton{border:1px solid rgb(159,168,218);border-radius:3px;}\
									QPushButton:pressed{margin:1px;}");
    m_pButtonRefreshTree->setFixedSize(QSize(25, 25));
    m_pButtonRefreshTree->setIcon(QIcon(":/Resources/Common/image/RefreshPage.png"));
    m_pButtonRefreshTree->setIconSize(QSize(QSize(10, 10)));
    m_pButtonRefreshTree->setCheckable(true);
	connect(m_pButtonRefreshTree, &QPushButton::clicked, this, [=] {
// 		QList<QStringList> lstEquipments = m_wheelPointTreeData->getCollectTreeDataFromDB();
// 		LoadEquipmentItem(lstEquipments);
        InitModel();
	});
	

	m_pButtonShrink = new QPushButton();
	m_pButtonShrink->setStyleSheet("QPushButton{border:1px solid rgb(159,168,218);border-radius:3px;}\
									QPushButton:pressed{margin:1px;}");
	m_pButtonShrink->setFixedSize(QSize(25, 25));
	m_pButtonShrink->setIcon(QIcon(":/Resources/Common/image/doubleLeftArrow.png"));
	m_pButtonShrink->setIconSize(QSize(QSize(10, 10)));
	m_pButtonShrink->setCheckable(true);
	connect(m_pButtonShrink, &QPushButton::clicked, this, [=] {
		m_mainBackWidget->setVisible(false);
		m_popButtonBackWidget->setVisible(true);
		this->setFixedWidth(20);
		emit signalTreeWidgetShrink(true);
	});

	m_pButtonPop = new QPushButton;
	m_pButtonPop->setFixedSize(QSize(20, 50));
	m_pButtonPop->setIcon(QIcon(":/Resources/Common/image/doubleRightArrow.png"));
	m_pButtonPop->setIconSize(QSize(QSize(10, 10)));
	m_pButtonPop->setStyleSheet("QPushButton{border:1px solid rgb(159,168,218);border-radius:3px;}\
									QPushButton:pressed{margin:1px;}");
	connect(m_pButtonPop, &QPushButton::clicked, this, [=] {
		m_popButtonBackWidget->setVisible(false);
		m_mainBackWidget->setVisible(true);
		this->setFixedWidth(300);
		emit signalTreeWidgetShrink(false);
	});

	m_popButtonBackWidget = new QWidget;
	QVBoxLayout* vlayout = new QVBoxLayout(m_popButtonBackWidget);
	vlayout->addStretch();
	vlayout->addWidget(m_pButtonPop);
	vlayout->addStretch();
	vlayout->setSpacing(0);
	vlayout->setMargin(0);

	m_popButtonBackWidget->setVisible(false);

//	m_pButtonRefreshTree->setVisible(false);

	QHBoxLayout* hTitleLayout = new QHBoxLayout(m_titleBackWidget);
	hTitleLayout->addWidget(m_titleLabel);
	hTitleLayout->addStretch();
	hTitleLayout->addWidget(m_pButtonRefreshTree);
//	hTitleLayout->addWidget(m_pButtonShrink);
	hTitleLayout->setMargin(0);
	hTitleLayout->setSpacing(15);

	m_searchBackWidget = new QWidget;
	m_searchBackWidget->setObjectName("SearchLineEditWidget");
	m_searchBackWidget->setFixedHeight(30);

	m_searchLineEdit = new QLineEdit;
	m_searchLineEdit->setFixedSize(QSize(230, 20));

	m_pButtonDeviceTreeSearch = new QToolButton;
	m_pButtonDeviceTreeSearch->setText("��ѯ");
	m_pButtonDeviceTreeSearch->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonDeviceTreeSearch->setFixedSize(QSize(60, 20));
	m_pButtonDeviceTreeSearch->setIcon(QIcon(":/Resources/Common/image/Search.png"));
	m_pButtonDeviceTreeSearch->setIconSize(QSize(16, 16));
	m_pButtonDeviceTreeSearch->setStyleSheet("border:none;");

	QHBoxLayout* hSearchLayout = new QHBoxLayout(m_searchBackWidget);
	hSearchLayout->addWidget(m_searchLineEdit);
	hSearchLayout->addWidget(m_pButtonDeviceTreeSearch);
	hSearchLayout->addStretch();
	hSearchLayout->setContentsMargins(10, 0, 0, 0);
	hSearchLayout->setSpacing(10);

	m_mainBackWidget = new QWidget;
	QVBoxLayout* hBottomLayout = new QVBoxLayout(m_mainBackWidget);
	hBottomLayout->addWidget(m_titleBackWidget);
	//hBottomLayout->addWidget(m_searchBackWidget);
	//hBottomLayout->addWidget(m_treeWidget);
	hBottomLayout->addWidget(m_pTreeView);
	hBottomLayout->setMargin(0);
	hBottomLayout->setSpacing(1);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_mainBackWidget);
	hMainLayout->addWidget(m_popButtonBackWidget);
	hMainLayout->setSpacing(0);
	hMainLayout->setMargin(0);

	m_pTreeModel = new QStandardItemModel( 1, 1, m_pTreeView);
	//connect(m_pTreeModel, SIGNAL(itemChanged(QStandardItem *)), this, SLOT(ItemChangedSlot(QStandardItem *)));
	m_pTreeView->setModel(m_pTreeModel);

	m_pTreeDelegate = new AddEquipTreeDelegate(m_pTreeView);
	connect(m_pTreeDelegate, SIGNAL(EquipmentItemDoubleClickedSignal(const QModelIndex &)), this, SLOT(EquipmentItemDoubleClickedSlot(const QModelIndex &)));
	m_pTreeDelegate->SetView(m_pTreeView);
	m_pTreeView->setItemDelegate(m_pTreeDelegate);
}

void AddEquipmentTreeWgt::InitModel()
{
    m_pTreeModel->clear();
    m_pTreeModel->removeRows(0, m_pTreeModel->rowCount());
    m_mapUid2Items.clear();
	//m_mapUid2Item.clear();
	QString rootItemName, stationName;
	int rootItemLevel, stationLevel;
	m_wheelPointTreeData->getPointTreeFirstRootNode(rootItemName, rootItemLevel);
	m_wheelPointTreeData->getPointTreeSecondRootNode(stationName, stationLevel);

	m_pRootStandardItem = new QStandardItem("");
	m_pTreeModel->setItem(0, 0, m_pRootStandardItem);
	m_pTreeModel->setData(m_pRootStandardItem->index(), QVariant(rootItemLevel), ITEM_LEVEL);
	m_pTreeModel->setData(m_pRootStandardItem->index(), QVariant(Robot_Company), ITEM_TYPE);
	m_pTreeModel->setData(m_pRootStandardItem->index(), QVariant(rootItemName), ITEM_TEXT);

	m_pSecondStandardItem = new QStandardItem("");
	m_pRootStandardItem->setChild(0, m_pSecondStandardItem);
	m_pTreeModel->setData(m_pSecondStandardItem->index(), QVariant(stationLevel), ITEM_LEVEL);
	m_pTreeModel->setData(m_pSecondStandardItem->index(), QVariant(Robot_Station), ITEM_TYPE);
	m_pTreeModel->setData(m_pSecondStandardItem->index(), QVariant(stationName), ITEM_TEXT);

	if (NULL != m_wheelPointTreeData)
	{
		QList<QStringList> lstEquipments = m_wheelPointTreeData->getCollectTreeDataFromDB();
		LoadEquipmentItem(lstEquipments);
	}

	m_pTreeView->expand(m_pRootStandardItem->index());
	m_pTreeView->expand(m_pSecondStandardItem->index());

}


void AddEquipmentTreeWgt::EquipmentItemDoubleClickedSlot(const QModelIndex &index)
{
	//m_pTreeModel->setData(index, "1", ITEM_STATE);
	QString strDeviceID = m_pTreeModel->data(index, ITEM_DEVICE).toString();
	QString strPointID = m_pTreeModel->data(index, ITEM_UUID).toString();
	//emit CollectEquipmentSignal(strDeviceID);
	if (!strDeviceID.isEmpty() && !strPointID.isEmpty())
	{
		emit signalModifyDeviceInfo(strDeviceID, strPointID);
	}
}

void AddEquipmentTreeWgt::AddEquipmentTypeOkBtnSlot()
{
	m_pAddTypeAndEquipmentBaseWgt->close();
	QList<QStringList> lstSelectDatas = m_pAddTypeAndEquipmentDlg->GetDatas();
	int iEquipmentType = m_pAddTypeAndEquipmentDlg->GetWgtType();
	switch (iEquipmentType)
	{
	case ADD_TYPE:
	{//����豸����
		QStandardItem *pCurItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
		if (pCurItem)
		{
			QStringList lstCurSelectDatas = GetItemDatas(pCurItem);
			QStringList lstDelDatas = lstCurSelectDatas;		//��ȡ��Ҫɾ���Ľڵ�
			foreach(QString strCurSelectID, lstCurSelectDatas)
			{
				foreach(QStringList lstSelectData, lstSelectDatas)
				{
					if (strCurSelectID == lstSelectData.at(0))
					{
						lstDelDatas.removeOne(strCurSelectID);
					}
				}
			}

			int iNodeType = m_pTreeModel->data(pCurItem->index(), ITEM_TYPE).toInt();
			QString strNodeID = "";
			QString strIntervalID = m_pTreeModel->data(pCurItem->index(), ITEM_UUID).toString();
			QString strLevelID = "";
			if (RootNode_Interval == iNodeType)
			{//����ڵ�
				QStandardItem *pParentItem = pCurItem->parent();
				if (pParentItem)
				{
					strLevelID = m_pTreeModel->data(pParentItem->index(), ITEM_UUID).toString();
					strNodeID = QString("%1/%2").arg(strLevelID).arg(strIntervalID);
				}
			}


			//������ɾ���ڵ�
			foreach(QString strDelID, lstDelDatas)
			{
				QString strTemp = strNodeID;
				strTemp += "/";
				strTemp += strDelID;
				QMap<QString, QStandardItem *>::iterator it = m_mapUid2Items.find(strTemp);
				if (it != m_mapUid2Items.end())
				{
					QStandardItem *pCurItem = it.value();
					if (pCurItem)
					{
						RemoveChildItem(pCurItem, strTemp);				//ɾ���ӽڵ�
						QStandardItem *pParentItem = pCurItem->parent();
						if (pParentItem)
						{//ɾ����ǰ�ڵ�
							pParentItem->removeRow(pCurItem->row());
							m_mapUid2Items.remove(strTemp);
						}
					}
				}
			}

			if (lstDelDatas.size() > 0 && !strLevelID.isEmpty() && !strIntervalID.isEmpty())
			{
				WHEEL_BACK_TO_CORE_SOCKET.robot_delete_device_type_req(strLevelID, strIntervalID, lstDelDatas);
			}

// 			foreach(QString strTypeID, lstDelDatas)
// 			{//ɾ���豸����
// 				WHEEL_BACK_TO_CORE_SOCKET.robot_delete_device_type_req(strLevelID, strIntervalID, strTypeID);
// 			}

			foreach(QStringList lstSelectData, lstSelectDatas)
			{//����豸����
				QString strTemp = strNodeID;
				strTemp += "/";
				strTemp += lstSelectData.at(0);
				QMap<QString, QStandardItem *>::iterator it = m_mapUid2Items.find(strTemp);
				if (it != m_mapUid2Items.end())
				{
					continue;
				}
				else
				{
					QStandardItem *pChildItem = new QStandardItem("");
					pCurItem->setChild(pCurItem->rowCount(), pChildItem);
					m_mapUid2Items.insert(strTemp, pChildItem);
					m_pTreeModel->setData(pChildItem->index(), QVariant(RootNode_DeviceType), ITEM_TYPE);								//�ڵ�����
					m_pTreeModel->setData(pChildItem->index(), QVariant(lstSelectData.at(1)), ITEM_TEXT);				//�ڵ��ı�
					m_pTreeModel->setData(pChildItem->index(), QVariant(lstSelectData.at(0)), ITEM_UUID);				//�ڵ�id
				}
			}
		}
	}break;
	case ADD_TYPE_AND_EQUIPMENT:
	{//����豸���ͺ��豸
		QStandardItem *pCurItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
		if (pCurItem)
		{
			QStringList lstCurSelectDatas = GetItemDatas(pCurItem);
			QStringList lstDelDatas = lstCurSelectDatas;		//��ȡ��Ҫɾ���Ľڵ�
			foreach(QString strCurSelectID, lstCurSelectDatas)
			{
				foreach(QStringList lstSelectData, lstSelectDatas)
				{
					if (strCurSelectID == lstSelectData.at(0))
					{
						lstDelDatas.removeOne(strCurSelectID);
					}
				}
			}

			int iNodeType = m_pTreeModel->data(pCurItem->index(), ITEM_TYPE).toInt();
			QString strNodeID = "";
			QString strIntervalID = m_pTreeModel->data(pCurItem->index(), ITEM_UUID).toString();
			QString strLevelID = "";
			if (RootNode_Interval == iNodeType)
			{//����ڵ�
				QStandardItem *pParentItem = pCurItem->parent();
				if (pParentItem)
				{
					strLevelID = m_pTreeModel->data(pParentItem->index(), ITEM_UUID).toString();
					strNodeID = QString("%1/%2").arg(strLevelID).arg(strIntervalID);
				}
			}

			//������ɾ���ڵ�
			foreach(QString strDelID, lstDelDatas)
			{
				QString strTemp = strNodeID;
				strTemp += "/";
				strTemp += strDelID;
				QMap<QString, QStandardItem *>::iterator it = m_mapUid2Items.find(strTemp);
				if (it != m_mapUid2Items.end())
				{
					QStandardItem *pCurItem = it.value();
					if (pCurItem)
					{
						RemoveChildItem(pCurItem, strTemp);
						QStandardItem *pParentItem = pCurItem->parent();
						if (pParentItem)
						{
							pParentItem->removeRow(pCurItem->row());
							m_mapUid2Items.remove(strTemp);
						}
					}
				}
			}

// 			foreach(QString strTypeID, lstDelDatas)
// 			{//ɾ���豸����
// 				if (!strLevelID.isEmpty() && !strIntervalID.isEmpty() && !strTypeID.isEmpty())
// 				{
// 					WHEEL_BACK_TO_CORE_SOCKET.robot_delete_device_type_req(strLevelID, strIntervalID, strTypeID);
// 				}
// 			}

			if (lstDelDatas.size() > 0 && !strLevelID.isEmpty() && !strIntervalID.isEmpty())
			{
				WHEEL_BACK_TO_CORE_SOCKET.robot_delete_device_type_req(strLevelID, strIntervalID, lstDelDatas);
			}

			QStringList lstAddTypeIDs;
			foreach(QStringList lstSelectData, lstSelectDatas)
			{//����豸����
				QString strTemp = strNodeID;
				strTemp += "/";
				strTemp += lstSelectData.at(0);
				QMap<QString, QStandardItem *>::iterator it = m_mapUid2Items.find(strTemp);
				if (it != m_mapUid2Items.end())
				{
					continue;
				}
				else
				{
					lstAddTypeIDs << lstSelectData.at(0);
				}
			}

			if (!strLevelID.isEmpty() && !strIntervalID.isEmpty() && lstAddTypeIDs.size() > 0)
			{
				WHEEL_BACK_TO_CORE_SOCKET.robot_add_deviceType_andDevices_req(strLevelID, strIntervalID, lstAddTypeIDs);
			}
		}

	}break;
	case ADD_EQUIPMENT:
	{//����豸
		QStandardItem *pCurItem = m_pTreeModel->itemFromIndex(m_curModelIndex);				//�豸���ͽڵ�
		if (pCurItem)
		{
			QStringList lstCurSelectDatas = GetItemDatas(pCurItem);
			QStringList lstDelDatas = lstCurSelectDatas;		//��ȡ��Ҫɾ���Ľڵ�
			foreach(QString strCurSelectID, lstCurSelectDatas)
			{
				foreach(QStringList lstSelectData, lstSelectDatas)
				{
					if (strCurSelectID == lstSelectData.at(0))
					{
						lstDelDatas.removeOne(strCurSelectID);
					}
				}

			}

			int iNodeType = m_pTreeModel->data(pCurItem->index(), ITEM_TYPE).toInt();
			QString strNodeID = "";
			QString strLevelID = "";				//��ѹ�ȼ�id
			QString strIntervalID = "";				//���id
			QString strTypeID = m_pTreeModel->data(pCurItem->index(), ITEM_UUID).toString();
			if (RootNode_DeviceType == iNodeType)
			{//�ڵ�
				QStandardItem *pParentItem = pCurItem->parent();
				if (pParentItem)
				{
					strIntervalID = m_pTreeModel->data(pParentItem->index(), ITEM_UUID).toString();
					QStandardItem *pPParentItem = pParentItem->parent();
					if (pPParentItem)
					{
						strLevelID = m_pTreeModel->data(pPParentItem->index(), ITEM_UUID).toString();
						strNodeID = QString("%1/%2/%3").arg(strLevelID)
							.arg(strIntervalID)
							.arg(strTypeID);
					}
				}
			}

			//ɾ���ڵ�
			QStringList lstDelDeivceIDs;			
			foreach(QString strDelID, lstDelDatas)
			{
				QString strTempID = strNodeID + "/" + strDelID;
				QMap<QString, QStandardItem *>::iterator it = m_mapUid2Items.find(strTempID);
				if (it != m_mapUid2Items.end())
				{
					QStandardItem *pDelItem = it.value();
					if (pDelItem)
					{
						lstDelDeivceIDs << m_pTreeModel->data(pDelItem->index(), ITEM_DEVICE).toString();
						//ɾ���ڵ�
						m_mapUid2Items.remove(strTempID);
						QStandardItem *pParnetItem = pDelItem->parent();
						if (pParnetItem)
						{
							pParnetItem->removeRow(pDelItem->row());
						}
					}
				}
			}
		
			if (!strTypeID.isEmpty() && !strIntervalID.isEmpty() && !strLevelID.isEmpty() && lstDelDeivceIDs.size() > 0)
			{//ɾ�����ݿ�ڵ�
				WHEEL_BACK_TO_CORE_SOCKET.robot_delete_devices_fromlist_req(lstDelDeivceIDs, strLevelID, strIntervalID, strTypeID);
			}

			QStringList lstAddEquipmentIDs;
			foreach(QStringList lstSelectData, lstSelectDatas)
			{
				QString strTemp = strNodeID;
				strTemp += "/";
				strTemp += lstSelectData.at(0);
				QMap<QString, QStandardItem *>::iterator it = m_mapUid2Items.find(strTemp);
				if (it != m_mapUid2Items.end())
				{
					continue;
				}
				else
				{
					lstAddEquipmentIDs << lstSelectData.at(0);
				}
			}


			//���½ڵ�  strNodeID
			if (!strLevelID.isEmpty() && !strIntervalID.isEmpty() && !strTypeID.isEmpty() && lstAddEquipmentIDs.size() > 0)
			{//����豸
				WHEEL_BACK_TO_CORE_SOCKET.robot_add_devices_fromlist_req(strLevelID, strIntervalID, strTypeID, lstAddEquipmentIDs);
			}

		}
	}break;
	default:break;
	}
}

void AddEquipmentTreeWgt::AddEquipmentTypeCancelBtnSlot()
{
	m_pAddTypeAndEquipmentDlg->ClearContent();
	m_pAddTypeAndEquipmentBaseWgt->close();
}

void AddEquipmentTreeWgt::EditIntervalOKBtnSlot()
{
	QString strIntervalName = m_pEditIntervalWgt->getLineEditText();
	//qDebug() << "Interval name:" << strIntervalName;
	if (ADD_INTERVAL == m_iCurIntervalType)
	{//��Ӽ��

		int iLevelType = m_pTreeModel->data(m_curModelIndex, ITEM_TYPE).toInt();
		if (RootNode_VoltageLevel == iLevelType)
		{//����ǵ�ѹ�ȼ��ڵ�
			QString strLevelID = m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString();			//��ǰ�ĵ�ѹ�ȼ�id

			//����ȥ��ѹ�ȼ�id�ͼ����
			//���ؼ��id
			WHEEL_BACK_TO_CORE_SOCKET.robot_add_new_interval_req(strLevelID, strIntervalName);
		}
	}
	else if (RENAME_INTERVAL == m_iCurIntervalType)
	{//���������
		int iIntervalType = m_pTreeModel->data(m_curModelIndex, ITEM_TYPE).toInt();
		QStandardItem *pIntervalItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
		if (RootNode_Interval == iIntervalType && pIntervalItem)
		{//����Ǽ���ڵ�
			QStandardItem *pLevelItem = pIntervalItem->parent();
			if (pLevelItem)
			{//�������ݿ�  �����Ǽ��id�ͼ����������
				WHEEL_BACK_TO_CORE_SOCKET.robot_update_interval_req(m_pTreeModel->data(pLevelItem->index(), ITEM_UUID).toString()
					, m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString(), strIntervalName);
				//m_pTreeModel->setData(pIntervalItem->index(), strIntervalName, ITEM_TEXT);
			}
			//�������ڵ�

		}
	}
	else if (PASTE_INTERVAL == m_iCurIntervalType)
	{//ճ�����
		int iLevelType = m_pTreeModel->data(m_curModelIndex, ITEM_TYPE).toInt();
		QStandardItem *pLevelItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
		if (RootNode_VoltageLevel == iLevelType && pLevelItem)
		{//����ǵ�ѹ�ڵ�
			WHEEL_BACK_TO_CORE_SOCKET.robot_copy_paste_interval_req(m_lstCopyIntervalID.at(0), m_lstCopyIntervalID.at(1)
						, m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString(), strIntervalName);
		}
	}
	m_pTreeView->update();
	m_pAddIntervalBaseWgt->close();
}

void AddEquipmentTreeWgt::EditIntervalCancelBtnSlot()
{
	m_pAddIntervalBaseWgt->close();
}


void AddEquipmentTreeWgt::CustomMenuSlot(const QPoint &pos)
{
	if (NULL == m_pTreeMenu)
	{
		m_pTreeMenu = new QMenu(m_pTreeView);
		connect(m_pTreeMenu, SIGNAL(triggered(QAction *)), this, SLOT(TreeMenuActionSlot(QAction *)));
	}
	m_pTreeMenu->clear();

	m_curModelIndex = m_pTreeView->indexAt(pos);
	QStandardItem *pCurItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
	if (NULL == pCurItem) return;
	QString strUUid = m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString();
	int iCurItemType = m_pTreeModel->data(m_curModelIndex, ITEM_TYPE).toInt();
	QString strText = m_pTreeModel->data(m_curModelIndex, ITEM_TEXT).toString();

	if (!strUUid.isEmpty() && iCurItemType != Robot_Company)
	{//���ǵ�һ�ȼ��͵ڶ��ȼ�
		switch (iCurItemType)
		{
		case Robot_Station:
		{//

		}break;
		case RootNode_VoltageLevel:
		{//��ѹ�ȼ�
			QAction *pAddIntervalAction = m_pTreeMenu->addAction("��Ӽ��");
			pAddIntervalAction->setData(ADD_INTERVAL_ACTION);

			if (m_bIsCopyed)
			{
				QAction *pPasteIntervalAction = m_pTreeMenu->addAction("ճ�����");
				pPasteIntervalAction->setData(PASTE_INTERVAL_ACTION);
			}

		}break;
		case RootNode_Interval:
		{//���
			QAction *pRenameIntervalAction = m_pTreeMenu->addAction("������");
			pRenameIntervalAction->setData(RENAME_INTERVAL_ACTION);
			
			QAction *pCopyIntervalAction = m_pTreeMenu->addAction("���Ƽ��");
			pCopyIntervalAction->setData(COPY_INTERVAL_ACTION);

			QAction *pDelIntervalAction = m_pTreeMenu->addAction("ɾ�����");
			pDelIntervalAction->setData(DEL_INTERVAL_ACTION);

			QAction *pAddEquipmentTypeAction = m_pTreeMenu->addAction("����豸����");
			pAddEquipmentTypeAction->setData(ADD_EQUIPMENT_TYPE_ACTION);

			QAction *pAddEquipmentTypeAndEquipmentAction = m_pTreeMenu->addAction("����豸���豸����");
			pAddEquipmentTypeAndEquipmentAction->setData(ADD_EQUIPMENT_AND_TYPE_ACTION);

		}break;
		case RootNode_DeviceType:
		{//����
			QAction *pDelEquipmentTypeAction = m_pTreeMenu->addAction("ɾ���豸����");
			pDelEquipmentTypeAction->setData(DEL_EQUIPMENT_TYPE_ACTION);

			QAction *pAddEquipmentAction = m_pTreeMenu->addAction("����豸");
			pAddEquipmentAction->setData(ADD_EQUIPMENT_ACTION);
		}break;
		case RootNode_Device:
		{//�ڵ�
			QAction *pDelEquipmentAction = m_pTreeMenu->addAction("ɾ���豸");
			pDelEquipmentAction->setData(DEL_EQUIPMENT_ACTION);

			QAction *pRunAction = m_pTreeMenu->addAction("���Ƶ���");
			pRunAction->setData(RUN_POINT_ACTION);
		}break;
		default:break;
		}
	}
	m_pTreeMenu->exec(QCursor::pos());
}



void AddEquipmentTreeWgt::TreeMenuActionSlot(QAction *pAction)
{//�˵��Ĳۺ���
	//m_curModelIndex = m_pTreeView->currentIndex();
	switch (pAction->data().toInt())
	{
	case ADD_INTERVAL_ACTION:
	{//��Ӽ��
		m_iCurIntervalType = ADD_INTERVAL;
		m_pEditIntervalWgt->clearContent();
		m_pAddIntervalBaseWgt->show();
	}break;
	case COPY_INTERVAL_ACTION:
	{//���Ƽ��
		m_lstCopyIntervalID.clear();
		QString strIntervalID = m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString();
		QStandardItem *pCurItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
		if (!strIntervalID.isEmpty() && pCurItem)
		{
			m_bIsCopyed = true;
			//m_strCopyIntervalID = strIntervalID;

			QStandardItem *pParentItem = pCurItem->parent();
			if (pParentItem)
			{
				m_lstCopyIntervalID << m_pTreeModel->data(pParentItem->index(), ITEM_UUID).toString()
					<< m_pTreeModel->data(pCurItem->index(), ITEM_UUID).toString();
			}

		}
	}break;
	case PASTE_INTERVAL_ACTION:
	{//ճ�����
		if (m_bIsCopyed)
		{
			m_bIsCopyed = false;
			QStandardItem *pLevelItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
			if (pLevelItem)
			{
				//////////////////////////////////////////////////////////////////////////
				//���ĸ���ѹ�ȼ��£�ճ���ĸ����
				m_iCurIntervalType = PASTE_INTERVAL;
				m_pEditIntervalWgt->clearContent();
				m_pAddIntervalBaseWgt->show();
			}
		}
	}break;
	case DEL_INTERVAL_ACTION://ɾ�����
	{
		QStandardItem *pIntervalItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
		if (pIntervalItem)
		{
			QString strIntervalID = m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString();
			//��ȡinterval��id�������ݿ���ɾ��
			//////////////////////////////////////////////////////////////////////////
			//�������ݿⷵ��ֵ���Ƿ�ɾ�������ϵĽڵ�

			QStandardItem *parentItem = pIntervalItem->parent(); 
			if (parentItem)
			{
				QString strLevelID = m_pTreeModel->data(parentItem->index(), ITEM_UUID).toString();
				//����ɾ�����
				WHEEL_BACK_TO_CORE_SOCKET.robot_delete_interval_req(strLevelID, strIntervalID);
				//ɾ����������µĽڵ�
				QString strCurID = strLevelID + "/" + strIntervalID;
				RemoveChildItem(pIntervalItem, strCurID);
				m_mapUid2Items.remove(strCurID);
				parentItem->removeRow(pIntervalItem->row());

			}
		}
	}break;
	case DEL_EQUIPMENT_ACTION://ɾ���豸
	{
		QStandardItem *pEquipmentItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
		if (pEquipmentItem)
		{
			QString strLevelID = "";
			QString strIntervalID = "";
			QString strTypeID = "";
			QString strEquipmentID = m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString();
			QString strDeviceID = m_pTreeModel->data(m_curModelIndex, ITEM_DEVICE).toString();
			//��ȡinterval��id�������ݿ���ɾ��
			//////////////////////////////////////////////////////////////////////////
			//�������ݿⷵ��ֵ���Ƿ�ɾ�������ϵĽڵ�

			QStandardItem *parentItem = pEquipmentItem->parent();
			if (parentItem)
			{
				strTypeID = m_pTreeModel->data(parentItem->index(), ITEM_UUID).toString();
				QStandardItem *pIntervalItem = parentItem->parent();
				if (pIntervalItem)
				{
					strIntervalID = m_pTreeModel->data(pIntervalItem->index(), ITEM_UUID).toString();
					QStandardItem *pLevelItem = pIntervalItem->parent();
					if (pLevelItem)
					{
						strLevelID = m_pTreeModel->data(pLevelItem->index(), ITEM_UUID).toString();
					}
				}
				if (!strLevelID.isEmpty() && !strIntervalID.isEmpty() && !strTypeID.isEmpty() && !strEquipmentID.isEmpty())
				{//ɾ���豸�͸������ݿ�
					QString strTemp = QString("%1/%2/%3/%4").arg(strLevelID).arg(strIntervalID).arg(strTypeID).arg(strEquipmentID);
					WHEEL_BACK_TO_CORE_SOCKET.robot_delete_devices_fromlist_req(QStringList() << strDeviceID, strLevelID, strIntervalID, strTypeID);
					m_mapUid2Items.remove(strTemp);
					parentItem->removeRow(m_curModelIndex.row());
				}
			}


		}
	}break;
	case DEL_EQUIPMENT_TYPE_ACTION://ɾ���豸����
	{
		QStandardItem *pTypeItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
		if (pTypeItem)
		{
			QString strTypeID = m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString();
			QString strLevelID = "";
			QString strIntervalID = "";
			//��ȡinterval��id�������ݿ���ɾ��
			//////////////////////////////////////////////////////////////////////////
			//�������ݿⷵ��ֵ���Ƿ�ɾ�������ϵĽڵ�
			QStandardItem *pIntervalItem = pTypeItem->parent();
			if (pIntervalItem)
			{
				strIntervalID = m_pTreeModel->data(pIntervalItem->index(), ITEM_UUID).toString();
				QStandardItem *pLevelItem = pIntervalItem->parent();
				if (pLevelItem)
				{
					strLevelID = m_pTreeModel->data(pLevelItem->index(), ITEM_UUID).toString();
				}
				
				if (!strLevelID.isEmpty() && !strIntervalID.isEmpty() && !strTypeID.isEmpty())
				{
					QString strTemp = QString("%1/%2/%3").arg(strLevelID).arg(strIntervalID).arg(strTypeID);
					RemoveChildItem(pTypeItem, strTemp);
					pIntervalItem->removeRow(pTypeItem->row());
					m_mapUid2Items.remove(strTemp);
					WHEEL_BACK_TO_CORE_SOCKET.robot_delete_device_type_req(strLevelID, strIntervalID, strTypeID);
				}
			}
		}
	}break;
	case RENAME_INTERVAL_ACTION:
	{//�����������
		m_iCurIntervalType = RENAME_INTERVAL;
		m_pEditIntervalWgt->clearContent();
		m_pAddIntervalBaseWgt->show();
	}break;
	case ADD_EQUIPMENT_TYPE_ACTION:
	{//����豸����
		m_pAddTypeAndEquipmentBaseWgt->setTitleContent("����豸����");
		m_pAddTypeAndEquipmentDlg->SetWgtType(ADD_TYPE);
		m_pAddTypeAndEquipmentDlg->ClearContent();
		QList<QStringList> lstAllDatas = m_wheelPointTreeData->GetAllEquipmentTypeDatas();
		QList<QStringList> lstSelectDatas = GetSelectDatas();			//��ȡѡ�е�data
		
		m_pAddTypeAndEquipmentDlg->SetDatas(lstAllDatas, lstSelectDatas);
		m_pAddTypeAndEquipmentBaseWgt->show();
	}break;
	case ADD_EQUIPMENT_AND_TYPE_ACTION:
	{//����豸���豸����
		m_pAddTypeAndEquipmentBaseWgt->setTitleContent("����豸���豸����");
		m_pAddTypeAndEquipmentDlg->SetWgtType(ADD_TYPE_AND_EQUIPMENT);
		m_pAddTypeAndEquipmentDlg->ClearContent();
		QList<QStringList> lstAllDatas = m_wheelPointTreeData->GetAllEquipmentTypeDatas();
		QList<QStringList> lstSelectDatas = GetSelectDatas();					//��ȡѡ�е�data

		m_pAddTypeAndEquipmentDlg->SetDatas(lstAllDatas, lstSelectDatas);
		m_pAddTypeAndEquipmentBaseWgt->show();
	}break;
	case ADD_EQUIPMENT_ACTION:
	{//����豸
		QString strID = m_pTreeModel->data(m_curModelIndex, ITEM_UUID).toString();

		m_pAddTypeAndEquipmentBaseWgt->setTitleContent("����豸");
		m_pAddTypeAndEquipmentDlg->SetWgtType(ADD_EQUIPMENT);
		m_pAddTypeAndEquipmentDlg->ClearContent();
		QList<QStringList> lstAllDatas = m_wheelPointTreeData->GetAllEquipmentDatas(strID);
		QList<QStringList> lstSelectDatas = GetSelectDatas();					//��ȡѡ�е�data

		m_pAddTypeAndEquipmentDlg->SetDatas(lstAllDatas, lstSelectDatas);
		m_pAddTypeAndEquipmentBaseWgt->show();
	}break;
	case RUN_POINT_ACTION:
	{//���ߵ��õ�
		QString strDeviceID = m_pTreeModel->data(m_curModelIndex, ITEM_DEVICE).toString();
		QString strPointId = "";
		bool bIsSuccess = WHEEL_ROBOT_DB.getPointId(strDeviceID, strPointId);
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_gotarget_req(strPointId.toLocal8Bit().constData(), 0);
	}break;
	default:
		break;
	}
}


void AddEquipmentTreeWgt::UpdateTreeNode(QStringList lstUpdateTreeNode, RootNodeType iType)
{
	QList<QStringList> lstEquipments = m_wheelPointTreeData->getCollectTreeDataFromDB(lstUpdateTreeNode, iType);
	LoadEquipmentItem(lstEquipments);
}


QList<QStringList> AddEquipmentTreeWgt::GetSelectDatas()
{
	QList<QStringList> lstSelectDatas;
	QStandardItem *pParentItem = m_pTreeModel->itemFromIndex(m_curModelIndex);
	if (pParentItem)
	{
		for (int iRow = 0; iRow < pParentItem->rowCount(); ++iRow)
		{
			QStandardItem *pChildItem = pParentItem->child(iRow);
			if (pChildItem)
			{
				QStringList lstSelectData;
				lstSelectData << m_pTreeModel->data(pChildItem->index(), ITEM_UUID).toString()
					<< m_pTreeModel->data(pChildItem->index(), ITEM_TEXT).toString() 
					<< m_pTreeModel->data(pChildItem->index(), DEVICE_UID).toString();
				lstSelectDatas.push_back(lstSelectData);
			}
		}
	}
	return lstSelectDatas;
}


QStringList AddEquipmentTreeWgt::GetItemDatas(QStandardItem *pItem)
{
	QStringList lstItemDatas;
	for (int iRow = 0; iRow < pItem->rowCount(); ++iRow)
	{
		QStandardItem *pChildItem = pItem->child(iRow);
		if (pChildItem)
		{
			lstItemDatas << m_pTreeModel->data(pChildItem->index(), ITEM_UUID).toString();
		}
	}
	return lstItemDatas;
}

void AddEquipmentTreeWgt::RemoveChildItem(QStandardItem *pCurItem, QString strCurItemID)
{
	if (pCurItem->rowCount() <= 0)  return;
	for (int iRow = 0; iRow < pCurItem->rowCount(); ++iRow)
	{
		QString strTemp = strCurItemID;
		QStandardItem *pChildItem = pCurItem->child(iRow);
		if (pChildItem)
		{
			strTemp += "/";
			strTemp += m_pTreeModel->data(pChildItem->index(), ITEM_UUID).toString();
			m_mapUid2Items.remove(strTemp);
			RemoveChildItem(pChildItem, strTemp);
		}
	}
}


void AddEquipmentTreeWgt::LoadEquipmentItem(const QList<QStringList> &lstEquipments)
{//�����豸�ڵ�
//    InitModel();
	foreach(QStringList lstEquipment, lstEquipments)
	{
		if (!lstEquipment.at(DEVICE_UID).isEmpty())
		{
			QString strCurEquipmentID = QString("%1/%2/%3/%4").arg(lstEquipment.at(VOLTAGE_LEVEL_UID))
				.arg(lstEquipment.at(INTERVAL_UID))
				.arg(lstEquipment.at(TYPE_UID))
				.arg(lstEquipment.at(EQUIPMENT_UID));
			QMap < QString, QStandardItem * >::const_iterator itCurEquipment = m_mapUid2Items.find(strCurEquipmentID);
			if (itCurEquipment != m_mapUid2Items.end())
			{
				QStandardItem *pEquipmentItem = itCurEquipment.value();
				if (pEquipmentItem)
				{
					m_pTreeModel->setData(pEquipmentItem->index(), QVariant(lstEquipment.at(EQUIPMENT_STATE)), ITEM_STATE);										//��¼�Ƿ�ɼ�
				}
				continue;
			}
		}

		if (!lstEquipment.at(VOLTAGE_LEVEL_UID).isEmpty())
		{
			//����豸��ѹ�ȼ�
			QStandardItem *pLevelItem = NULL;
			QString strLevelID = lstEquipment.at(VOLTAGE_LEVEL_UID);
			QMap < QString, QStandardItem * >::const_iterator itLevel = m_mapUid2Items.find(strLevelID);
			if (itLevel != m_mapUid2Items.end())
			{
				pLevelItem = itLevel.value();
			}
			else
			{
				int index = m_pSecondStandardItem->rowCount();
				pLevelItem = new QStandardItem("");
				m_pSecondStandardItem->setChild(index, pLevelItem);
				m_mapUid2Items.insert(strLevelID, pLevelItem);
				m_pTreeModel->setData(pLevelItem->index(), QVariant(RootNode_VoltageLevel), ITEM_TYPE);								//�ڵ�����
				m_pTreeModel->setData(pLevelItem->index(), QVariant(lstEquipment.at(VOLTAGE_LEVEL_NAME)), ITEM_TEXT);				//�ڵ��ı�
				m_pTreeModel->setData(pLevelItem->index(), QVariant(lstEquipment.at(VOLTAGE_LEVEL_UID)), ITEM_UUID);				//�ڵ�id
			}

			if (!lstEquipment.at(INTERVAL_UID).isEmpty())
			{
				//����豸��� 
				QStandardItem *pIntervalItem = NULL;
				QString strIntervalID = QString("%1/%2").arg(lstEquipment.at(VOLTAGE_LEVEL_UID)).arg(lstEquipment.at(INTERVAL_UID));
				QMap < QString, QStandardItem * >::const_iterator itStation = m_mapUid2Items.find(strIntervalID);
				if (itStation != m_mapUid2Items.end())
				{
					pIntervalItem = itStation.value();
					if (RENAME_INTERVAL == m_iCurIntervalType)
					{
						m_pTreeModel->setData(pIntervalItem->index(), QVariant(lstEquipment.at(INTERVAL_NAME)), ITEM_TEXT);				//�ڵ��ı�
					}
				}
				else
				{
					if (NULL != pLevelItem)
					{
						int index = pLevelItem->rowCount();
						pIntervalItem = new QStandardItem("");
						pLevelItem->setChild(index, pIntervalItem);
						m_mapUid2Items.insert(strIntervalID, pIntervalItem);
						m_pTreeModel->setData(pIntervalItem->index(), QVariant(RootNode_Interval), ITEM_TYPE);								//�ڵ�����
						m_pTreeModel->setData(pIntervalItem->index(), QVariant(lstEquipment.at(INTERVAL_NAME)), ITEM_TEXT);				//�ڵ��ı�
						m_pTreeModel->setData(pIntervalItem->index(), QVariant(lstEquipment.at(INTERVAL_UID)), ITEM_UUID);				//�ڵ�id

					}
				}

				if (!lstEquipment.at(TYPE_UID).isEmpty())
				{
					//����豸����
					QStandardItem *pTypeItem = NULL;
					QString strEquipmentTypeID = QString("%1/%2/%3").arg(lstEquipment.at(VOLTAGE_LEVEL_UID))
														.arg(lstEquipment.at(INTERVAL_UID)).arg(lstEquipment.at(TYPE_UID));
					QMap < QString, QStandardItem * >::const_iterator itType = m_mapUid2Items.find(strEquipmentTypeID);
					if (itType != m_mapUid2Items.end())
					{
						pTypeItem = itType.value();
					}
					else
					{
						if (NULL != pIntervalItem)
						{
							int index = pIntervalItem->rowCount();
							pTypeItem = new QStandardItem("");
							pIntervalItem->setChild(index, pTypeItem);
							m_mapUid2Items.insert(strEquipmentTypeID, pTypeItem);
							m_pTreeModel->setData(pTypeItem->index(), QVariant(RootNode_DeviceType), ITEM_TYPE);								//�ڵ�����
							m_pTreeModel->setData(pTypeItem->index(), QVariant(lstEquipment.at(TYPE_NAME)), ITEM_TEXT);				//�ڵ��ı�
							m_pTreeModel->setData(pTypeItem->index(), QVariant(lstEquipment.at(TYPE_UID)), ITEM_UUID);				//�ڵ�id
						}
					}

					//����豸
					QStandardItem *pEquipmentItem = NULL;
					QString strEquipmentID = QString("%1/%2/%3/%4").arg(lstEquipment.at(VOLTAGE_LEVEL_UID))
																.arg(lstEquipment.at(INTERVAL_UID))
																.arg(lstEquipment.at(TYPE_UID))
																.arg(lstEquipment.at(EQUIPMENT_UID));
					QMap < QString, QStandardItem * >::const_iterator itEquipment = m_mapUid2Items.find(strEquipmentID);
					if (itEquipment != m_mapUid2Items.end())
					{
						pEquipmentItem = itEquipment.value();
					}
					else
					{
						if (NULL != pTypeItem)
						{
							int index = pTypeItem->rowCount();
							pEquipmentItem = new QStandardItem("");
							pTypeItem->setChild(index, pEquipmentItem);
							m_mapUid2Items.insert(strEquipmentID, pEquipmentItem);
							m_pTreeModel->setData(pEquipmentItem->index(), QVariant(RootNode_Device), ITEM_TYPE);								//�ڵ�����
							m_pTreeModel->setData(pEquipmentItem->index(), QVariant(lstEquipment.at(EQUIPMENT_NAME)), ITEM_TEXT);				//�ڵ��ı�
							m_pTreeModel->setData(pEquipmentItem->index(), QVariant(lstEquipment.at(EQUIPMENT_UID)), ITEM_UUID);				//�ڵ�id
							m_pTreeModel->setData(pEquipmentItem->index(), QVariant(lstEquipment.at(DEVICE_UID)), ITEM_DEVICE);					//device id
							//qDebug() << "item state:" << lstEquipment.at(EQUIPMENT_STATE);
							m_pTreeModel->setData(pEquipmentItem->index(), QVariant(lstEquipment.at(EQUIPMENT_STATE)), ITEM_STATE);										//��¼�Ƿ�ɼ�
						}
					}
				}
			}
		}
	}
}

void AddEquipmentTreeWgt::InitAddTypeAndEquipmentDlg()
{
	if (NULL == m_pAddTypeAndEquipmentBaseWgt)
	{
		m_pAddTypeAndEquipmentBaseWgt = new BaseWidget(this, BaseWidgetType::PopupWindow);
		connect(m_pAddTypeAndEquipmentBaseWgt, SIGNAL(signalCloseButtonClicked()), m_pAddTypeAndEquipmentBaseWgt, SLOT(close()));
		m_pAddTypeAndEquipmentBaseWgt->setTitleContent("����豸����");
		//setAttribute(Qt::WA_DeleteOnClose);
		m_pAddTypeAndEquipmentBaseWgt->setFixedSize(QSize(650, 750));
		m_pAddTypeAndEquipmentBaseWgt->setShowCloseButton();
		QHBoxLayout *pLayout = new QHBoxLayout(m_pAddTypeAndEquipmentBaseWgt->getCenterWidget());
		m_pAddTypeAndEquipmentDlg = new AddTypeAndEquipmentDlg(this);
		connect(m_pAddTypeAndEquipmentDlg, SIGNAL(AddEquipmentTypeOkBtnSignal()), this, SLOT(AddEquipmentTypeOkBtnSlot()));
		connect(m_pAddTypeAndEquipmentDlg, SIGNAL(AddEquipmentTypeCancelBtnSignal()), this, SLOT(AddEquipmentTypeCancelBtnSlot()));
		pLayout->addWidget(m_pAddTypeAndEquipmentDlg);
	}
}

void AddEquipmentTreeWgt::InitEditIntervalDlg()
{
	m_pAddIntervalBaseWgt = new BaseWidget(this, PopupWindow);
	connect(m_pAddIntervalBaseWgt, SIGNAL(signalCloseButtonClicked()), m_pAddIntervalBaseWgt, SLOT(close()));
	m_pAddIntervalBaseWgt->setTitleContent("���");
	//setAttribute(Qt::WA_DeleteOnClose);
	m_pAddIntervalBaseWgt->setFixedSize(QSize(300, 200));
	m_pAddIntervalBaseWgt->setShowCloseButton();

	m_pEditIntervalWgt = new InputWidget(InputWidgetType::LineEdit);
	m_pEditIntervalWgt->setTipText(tr("�����"));
	m_pEditIntervalWgt->setFixedWidth(200);
	m_pEditIntervalWgt->setShowValue("-1");

	QPushButton *pOkBtn = new QPushButton(m_pAddIntervalBaseWgt);
	connect(pOkBtn, SIGNAL(clicked()), this, SLOT(EditIntervalOKBtnSlot()));
	pOkBtn->setText("ȷ��");


	QPushButton *pCancelBtn = new QPushButton(m_pAddIntervalBaseWgt);
	connect(pCancelBtn, SIGNAL(clicked()), this, SLOT(EditIntervalCancelBtnSlot()));
	pCancelBtn->setText("ȡ��");

	QHBoxLayout *pBtnLayout = new QHBoxLayout();
	pBtnLayout->addStretch();
	pBtnLayout->addWidget(pOkBtn);
	pBtnLayout->addWidget(pCancelBtn);

	QVBoxLayout *pLayout = new QVBoxLayout(m_pAddIntervalBaseWgt->getCenterWidget());
	pLayout->addWidget(m_pEditIntervalWgt);
	pLayout->addLayout(pBtnLayout);
}

