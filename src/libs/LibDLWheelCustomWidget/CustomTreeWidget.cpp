#include "CustomTreeWidget.h"
#include <QPainter>
#include <QFile>
#include <QHeaderView>
#include <QCoreApplication>
#include <QStack> 
#include <QDebug>
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPointTreeData.h"
#include "DefData.h"


CustomTreeWidget::CustomTreeWidget(QWidget* parent /* = NULL */)
	: QWidget(parent)
	, m_treeWidgetType(ColorRect_CheckBox_With)
	, m_isDevelopTreeWidget(false)
	, m_pTreeView(NULL)
	, m_pTreeDelegate(NULL)
	, m_pTreeMenu(NULL)
	, m_bIsShowMenu(false)
	, m_bIsExistCheckBox(false)
{
	m_wheelPointTreeData = new DLWheelPointTreeData;
	InitItemWidgetType(ColorRect_CheckBox_With);
	initWidget();
	InitModel();

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

CustomTreeWidget::~CustomTreeWidget()
{
	delete m_wheelPointTreeData;
}

void CustomTreeWidget::initWidget()
{
	m_pTreeView = new QTreeView(this);
	connect(m_pTreeView, SIGNAL(customContextMenuRequested(const QPoint &)), this, SLOT(CustomMenuSlot(const QPoint &)));
	m_pTreeView->setHeaderHidden(true);
	m_pTreeView->setFocusPolicy(Qt::NoFocus);
	m_pTreeView->setContextMenuPolicy(Qt::CustomContextMenu);
	m_pTreeView->setEditTriggers(QAbstractItemView::NoEditTriggers);

	m_titleBackWidget = new QWidget;
	m_titleBackWidget->setObjectName("ButtonBackWidget");
	m_titleBackWidget->setFixedHeight(30);

	m_titleLabel = new QLabel;
	m_titleLabel->setText("设备树");
	m_titleLabel->setStyleSheet("font-weight:bold;");

	m_pButtonRefreshTree = new QPushButton("刷新");
	m_pButtonRefreshTree->setFixedSize(QSize(60, 25));
	m_pButtonRefreshTree->setStyleSheet("QPushButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}");
	connect(m_pButtonRefreshTree, &QPushButton::clicked, this, [=] {
		emit signalRefreshTreeWidget();
	});
	

	m_pButtonShrink = new QPushButton();
	m_pButtonShrink->setStyleSheet("QPushButton{border:1px solid rgb(159,168,218);border-radius:3px;}\
									QPushButton:pressed{margin:1px;}");
	m_pButtonShrink->setFixedSize(QSize(16, 16));
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

	m_pButtonRefreshTree->setVisible(false);

	QHBoxLayout* hTitleLayout = new QHBoxLayout(m_titleBackWidget);
	hTitleLayout->addWidget(m_titleLabel);
	hTitleLayout->addStretch();
	hTitleLayout->addWidget(m_pButtonRefreshTree);
	hTitleLayout->addWidget(m_pButtonShrink);
	hTitleLayout->setMargin(5);
	hTitleLayout->setSpacing(15);

	m_searchBackWidget = new QWidget;
	m_searchBackWidget->setObjectName("SearchLineEditWidget");
	m_searchBackWidget->setFixedHeight(30);

	m_searchLineEdit = new QLineEdit;
	m_searchLineEdit->setFixedSize(QSize(230, 20));

	m_pButtonDeviceTreeSearch = new QToolButton;
	m_pButtonDeviceTreeSearch->setText("查询");
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
	hBottomLayout->addWidget(m_searchBackWidget);
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
	connect(m_pTreeModel, SIGNAL(itemChanged(QStandardItem *)), this, SLOT(ItemChangedSlot(QStandardItem *)));
	m_pTreeView->setModel(m_pTreeModel);

	m_pTreeDelegate = new EquipTreeDelegate(m_pTreeView);
	connect(m_pTreeDelegate, SIGNAL(ItemStateChangeSignal(const QModelIndex &)), this, SLOT(ItemStateChangeSlot(const QModelIndex &)));
    //connect(m_pTreeDelegate, SIGNAL(signalDeviceNodeClicked(QString)), this, SLOT(signalDeviceNodeClicked(QString)));
    connect(m_pTreeDelegate, &EquipTreeDelegate::signalDeviceNodeClicked, this, [=](QString uuid)
    {
        emit signalDeviceNodeClicked(uuid);
    });


	m_pTreeDelegate->SetView(m_pTreeView);
	m_pTreeDelegate->SetIsDrawCheckBox(m_bIsExistCheckBox);
	m_pTreeView->setItemDelegate(m_pTreeDelegate);
}

void CustomTreeWidget::InitModel()
{
// 	QTime time;
// 	time.start();

	m_mapUid2Item.clear();
	QString rootItemName, stationName;
	int rootItemLevel, stationLevel;
	m_wheelPointTreeData->getPointTreeFirstRootNode(rootItemName, rootItemLevel);
	m_wheelPointTreeData->getPointTreeSecondRootNode(stationName, stationLevel);

	QStandardItem *pRootItem = new QStandardItem("");
	m_pTreeModel->setItem(0, 0, pRootItem);
	m_pTreeModel->setData(pRootItem->index(), QVariant(rootItemLevel), ITEM_LEVEL);
	m_pTreeModel->setData(pRootItem->index(), QVariant(Robot_Company), ITEM_TYPE);
	m_pTreeModel->setData(pRootItem->index(), QVariant(rootItemName), ITEM_TEXT);
	m_pTreeModel->setData(pRootItem->index(), UnChecked_Status, ITEM_STATE);

	QStandardItem *pStationItem = new QStandardItem("");
	pRootItem->setChild(0, pStationItem);
	m_pTreeModel->setData(pStationItem->index(), QVariant(stationLevel), ITEM_LEVEL);
	m_pTreeModel->setData(pStationItem->index(), QVariant(Robot_Station), ITEM_TYPE);
	m_pTreeModel->setData(pStationItem->index(), QVariant(stationName), ITEM_TEXT);
	//	m_pTreeModel->setData(pStationItem->index(), QVariant(m_treeWidgetType), ITEM_WIDGET_TYPE);
	m_pTreeModel->setData(pStationItem->index(), UnChecked_Status, ITEM_STATE);

	if (NULL != m_wheelPointTreeData)
	{
		QList<WheelTreeSaveStruct> lstTreeItems = m_wheelPointTreeData->getTreeEntiretyData();
		LoadChildItems(pStationItem, lstTreeItems, (Robot_Station + 1));
	}

	m_pTreeView->expand(pRootItem->index());
	m_pTreeView->expand(pStationItem->index());
// 	qDebug() << "====================================================";
// 	qDebug() << "加载时长:" << time.elapsed();
// 	qDebug() << "====================================================";

}

void CustomTreeWidget::LoadChildItems(QStandardItem *pParentItem, QList<WheelTreeSaveStruct> lstChildInfo, int iChildType)
{
	if (NULL == pParentItem || lstChildInfo.size() <= 0 || iChildType > RootNode_Device) return;
	int index = 0;
	foreach(WheelTreeSaveStruct stWheelTreeSaveStruct, lstChildInfo)
	{
		QStandardItem *pTreeItem = new QStandardItem("");
		pParentItem->setChild(index, pTreeItem);
		m_pTreeModel->setData(pTreeItem->index(), QVariant(stWheelTreeSaveStruct.alarm_level), ITEM_LEVEL);
		m_pTreeModel->setData(pTreeItem->index(), QVariant(iChildType), ITEM_TYPE);
		m_pTreeModel->setData(pTreeItem->index(), QVariant(stWheelTreeSaveStruct.node_name), ITEM_TEXT);
		m_pTreeModel->setData(pTreeItem->index(), QVariant(stWheelTreeSaveStruct.node_uuid), ITEM_UUID);
		m_pTreeModel->setData(pTreeItem->index(), UnChecked_Status, ITEM_STATE);

		if (iChildType == RootNode_Device)
		{
			m_mapUid2Item.insert(stWheelTreeSaveStruct.node_uuid, pTreeItem);
		}

		LoadChildItems(pTreeItem, stWheelTreeSaveStruct.iter, (iChildType + 1));

		++index;
	}
}

void CustomTreeWidget::setTreeWidgetType(TreeItemWidgetType type)
{
	m_treeWidgetType = type;
	InitItemWidgetType(m_treeWidgetType);
	if (NULL != m_pTreeDelegate)
	{
		m_pTreeDelegate->SetIsDrawCheckBox(m_bIsExistCheckBox);
	}
}

void CustomTreeWidget::setSearchLineEditVisible(bool isVisible)
{
	m_searchBackWidget->setVisible(isVisible);
}

void CustomTreeWidget::setTitleWidgetVisible(bool isVisible)
{
	m_titleBackWidget->setVisible(isVisible);
}

void CustomTreeWidget::setDevelopTreeWidget()
{
	m_isDevelopTreeWidget = true;
	m_pButtonRefreshTree->setVisible(true);
	setSearchLineEditVisible(false);
	m_titleBackWidget->setFixedHeight(40);
	m_titleBackWidget->setStyleSheet(".QWidget{background:white;border:none;}");
	m_titleLabel->setStyleSheet("QLabel{padding-left:10px;color:rgb(98, 98, 98);font-size:16px;}");
	QString strStyleSheet = m_pTreeView->styleSheet();
	strStyleSheet += QString("border:none;");
	m_pTreeView->setStyleSheet(strStyleSheet);
}

QStringList CustomTreeWidget::getChoosedDeviceIdList()
{
	QStringList lstCheckedItems;
	QMap<QString, QStandardItem *>::const_iterator itItem = m_mapUid2Item.begin();
	while (itItem != m_mapUid2Item.end())
	{
		int iCheckState = m_pTreeModel->data(itItem.value()->index(), ITEM_STATE).toInt();
		if (Checked_Status == iCheckState)
		{
			lstCheckedItems << m_pTreeModel->data(itItem.value()->index(), ITEM_UUID).toString();
		}
		++itItem;
	}
	return lstCheckedItems;
}


void CustomTreeWidget::setIsShrink(bool isShrink)
{
    m_pButtonShrink->setVisible(isShrink);
}


QStandardItem * CustomTreeWidget::getCurrentSelectedItem()
{
	QModelIndex index = m_pTreeView->currentIndex();
	return m_pTreeModel->itemFromIndex(index);
}



//根据查询条件更新树节点的状态
void CustomTreeWidget::refreshTreeItemList(WheelPatrolParameter wheelPatrolPara)
{
	ClearAllItemState();
	if (m_wheelPointTreeData)
	{
		m_wheelPointTreeData->setTreeConditionForSearchData(wheelPatrolPara);
		QMap<QString, WheelPointTreeDataStruct> mapID2Info = m_wheelPointTreeData->getTreeSearchData();
		QMap<QString, WheelPointTreeDataStruct>::const_iterator itInfo = mapID2Info.begin();
		while (itInfo != mapID2Info.end())
		{
			QMap<QString, QStandardItem *>::iterator itItem = m_mapUid2Item.find(itInfo.key());
			if (itItem != m_mapUid2Item.end())
			{
				QStandardItem *pTreeItem = itItem.value();
				if (NULL != pTreeItem)
				{
					m_pTreeModel->setData(pTreeItem->index(), Checked_Status, ITEM_STATE);
				}
			}
			++itInfo;
		}
	}


	//更新父节点的状态
	//添加所有节点再stack中
	QStack<QStandardItem *> stackItems;
	QStandardItem *pRootItem = m_pTreeModel->item(0, 0);
	stackItems.push(pRootItem);
	ChangedItemState(stackItems, pRootItem);
}



void CustomTreeWidget::refreshTreeItemList()
{
	m_wheelPointTreeData->updataTree();
	QString rootItemName, stationName;
	int rootItemLevel, stationLevel;
	m_wheelPointTreeData->getPointTreeFirstRootNode(rootItemName, rootItemLevel);
	m_wheelPointTreeData->getPointTreeSecondRootNode(stationName, stationLevel);
	// 添加电压等级;
	QList<WheelTreeSaveStruct> treeItemDataList = m_wheelPointTreeData->getPointTreeRootNode(QStringList());
}

// void CustomTreeWidget::SetParentItemAlarmLevel(QTreeWidgetItem *pCurItem, DeviceAlarmLevel alarmLevel)
// {
// 	if (pCurItem)
// 	{
// 		QTreeWidgetItem *pParentItem = pCurItem->parent();
// 		if (pParentItem)
// 		{
// 			ItemWidget *pItemWgt = dynamic_cast<ItemWidget *>(m_treeWidget->itemWidget(pParentItem, 0));
// 			if (pItemWgt)
// 			{
// 				pItemWgt->updateCheckBoxState(GetTreeWidgetItemCheckedStatus(pParentItem));
// 				//pItemWgt->updateCheckBoxState(true);
// 				pItemWgt->setAlarmLevel(alarmLevel);
// 				pItemWgt->repaint();
// 			}
// 			SetParentItemAlarmLevel(pParentItem, alarmLevel);
// 		}
// 	}
// }


void CustomTreeWidget::ItemStateChangeSlot(const QModelIndex &index)
{
	QStandardItem *pCurItem = m_pTreeModel->itemFromIndex(index);
	if (pCurItem)
	{
		//更新子节点的状态
		UpdateChildState(pCurItem);
		//更新父类的状态
		UpdateParentState(pCurItem);
	}
}

void CustomTreeWidget::ItemChangedSlot(QStandardItem *pTreeItem)
{

}


void CustomTreeWidget::CustomMenuSlot(const QPoint &pos)
{
	if (!m_bIsShowMenu) return;
	if (NULL == m_pTreeMenu)
	{
		m_pTreeMenu = new QMenu(m_pTreeView);
	}
	m_pTreeMenu->clear();

	QModelIndex index = m_pTreeView->indexAt(pos);
	QStandardItem *pCurItem = m_pTreeModel->itemFromIndex(index);
	if (NULL == pCurItem) return;
	QString strUUid = m_pTreeModel->data(index, ITEM_UUID).toString();
	int iCurItemType = m_pTreeModel->data(index, ITEM_TYPE).toInt();
	QString strText = m_pTreeModel->data(index, ITEM_TEXT).toString();

	if (!strUUid.isEmpty())
	{
		QAction* modifyDevice = m_pTreeMenu->addAction("修改设备");
		connect(modifyDevice, &QAction::triggered, this, [=] {
			emit signalModifyDeviceInfo(strUUid, strText);
		});
		QAction* deleteDevice = m_pTreeMenu->addAction("删除设备");
		connect(deleteDevice, &QAction::triggered, this, [=] {
			onDeleteItem();
		});
	}
	if (iCurItemType != Robot_Company && iCurItemType != Robot_Station)
	{//第一级和第二级不能复制ID
		QAction* copyDeviceID = m_pTreeMenu->addAction("复制设备ID");
		connect(copyDeviceID, &QAction::triggered, this, [=] {
			// 如果设备id为空说明为根节点，需要进行查询;
			QString strDeviceId = strUUid;
			if (strDeviceId.isEmpty())
			{
				strDeviceId = getItemDeviceId(strText, (RootNodeType)iCurItemType);
			}
			QClipboard *clipboard = QApplication::clipboard();
			clipboard->setText(strDeviceId);
		});
	}

	if (!strUUid.isEmpty())
	{
		// 获取具体哪一点;
		QString strDeviceId = getItemDeviceId(strText, (RootNodeType)iCurItemType);
		WheelRobortDeviceParameterStruct paramData = WHEEL_DEVICE_CONFIG.getWheelDeviceParameterData(strDeviceId);
		QString strActionName = QString("移动至%1点").arg(paramData.point_id);

		QAction* moveToPoint = m_pTreeMenu->addAction(strActionName);
 		connect(moveToPoint, &QAction::triggered, this, [=] {
 			// 如果设备id为空说明为根节点，需要进行查询;
 			QString strDeviceId = getItemDeviceId(strText, (RootNodeType)iCurItemType);
 			emit signalMoveToDevicePoint(strDeviceId);
 		});
	}
	m_pTreeMenu->exec(QCursor::pos());
}

void CustomTreeWidget::UpdateTreeItemLevelSlot(QString strUUid, DeviceAlarmLevel iLevel)
{
	QMap<QString, QStandardItem *>::const_iterator itItem =  m_mapUid2Item.find(strUUid);
	if (itItem != m_mapUid2Item.end())
	{
		m_pTreeModel->setData(itItem.value()->index(), iLevel, ITEM_LEVEL);
		UpdateParentLevel(itItem.value());
	}
}

//更新所有子节点状态
void CustomTreeWidget::UpdateChildState(QStandardItem *pParentItem)
{
	if (NULL == pParentItem) return;
	int iState = m_pTreeModel->data(pParentItem->index(), ITEM_STATE).toInt();
	int iRowCount = pParentItem->rowCount();
	for (int iRow = 0; iRow < iRowCount; ++iRow)
	{
		QStandardItem *pChildItem = pParentItem->child(iRow, 0);
		if (pChildItem)
		{
			m_pTreeModel->setData(pChildItem->index(), iState, ITEM_STATE);
			UpdateChildState(pChildItem);
		}
	}
}

//更新父节点的状态
void CustomTreeWidget::UpdateParentState(QStandardItem *pCurItem)
{
	if (NULL == pCurItem || NULL == pCurItem->parent()) return;
	QStandardItem *pParentItem = pCurItem->parent();
	int iRowCount = pParentItem->rowCount();
	int iCheckCount = 0;
	bool bIsPartiallyState = false;
	for (int iRow = 0; iRow < iRowCount; ++iRow)
	{
		QStandardItem *pChildItem = pParentItem->child(iRow, 0);
		if (pChildItem)
		{
			int iState = m_pTreeModel->data(pChildItem->index(), ITEM_STATE).toInt();
			if (iState == Checked_Status)
			{
				++iCheckCount;
			}
			if (iState == PartiallyChecked_Status)
			{
				bIsPartiallyState = true;
			}
		}
	}
	if (0 == iCheckCount && bIsPartiallyState)
	{//没有全部选中的，但是有半选状态的，父节点为半选状态
		m_pTreeModel->setData(pParentItem->index(), PartiallyChecked_Status, ITEM_STATE);
	}
	else if (0 == iCheckCount)
	{//全为不选中
		m_pTreeModel->setData(pParentItem->index(), UnChecked_Status, ITEM_STATE);
	}
	else if (iRowCount == iCheckCount)
	{//全为选中
		m_pTreeModel->setData(pParentItem->index(), Checked_Status, ITEM_STATE);
	}
	else
	{//半选状态
		m_pTreeModel->setData(pParentItem->index(), PartiallyChecked_Status, ITEM_STATE);
	}

	UpdateParentState(pParentItem);
}


//清空所有节点的状态
void CustomTreeWidget::ClearAllItemState()
{
	QStandardItem *pRootItem = m_pTreeModel->item(0, 0);
	if (NULL != pRootItem)
	{
		m_pTreeModel->setData(pRootItem->index(), UnChecked_Status, ITEM_STATE);
		ClearChildState(pRootItem);
	}
}

//清空所有子节点的状态
void CustomTreeWidget::ClearChildState(QStandardItem *pParentItem)
{
	if (NULL == pParentItem) return;
	int iRowCount = pParentItem->rowCount();
	for (int iRow = 0; iRow < iRowCount; ++iRow)
	{
		QStandardItem *pChildItem = pParentItem->child(iRow, 0);
		if (pChildItem)
		{
			m_pTreeModel->setData(pChildItem->index(), UnChecked_Status, ITEM_STATE);
			ClearChildState(pChildItem);
		}
	}
}

void CustomTreeWidget::ChangedItemState(QStack<QStandardItem *> &stackItems, QStandardItem *pParentItem)
{
	if (NULL == pParentItem) return;
	int iRowCount = pParentItem->rowCount();
	for (int iRow = 0; iRow < iRowCount; ++iRow)
	{
		QStandardItem *pChildItem = pParentItem->child(iRow, 0);
		if (pChildItem)
		{
			stackItems.push(pChildItem);
		}
		ChangedItemState(stackItems, pChildItem);
		if (iRow == iRowCount - 1)
		{//遍历到最后一个节点
			int iType = m_pTreeModel->data(pChildItem->index(), ITEM_TYPE).toInt();
			int iCheckCount = 0;
			bool bIsPartially = false;
			QStandardItem *pParentItem = NULL;
			while (1)
			{
				QStandardItem *pPopItem = stackItems.pop();
				if (pPopItem)
				{
					int iCurType = m_pTreeModel->data(pPopItem->index(), ITEM_TYPE).toInt();
					if (iCurType != iType)
					{//父节点
						pParentItem = pPopItem;
						break;
					}
					int iChildStatus = m_pTreeModel->data(pPopItem->index(), ITEM_STATE).toInt();
					if (iChildStatus == Checked_Status)
					{
						++iCheckCount;
					}
					if (iChildStatus == PartiallyChecked_Status)
					{
						bIsPartially = true;
					}
				}
			}
			if (0 == iCheckCount && bIsPartially)
			{
				m_pTreeModel->setData(pParentItem->index(), PartiallyChecked_Status, ITEM_STATE);
			}
			else if (0 == iCheckCount)
			{
				m_pTreeModel->setData(pParentItem->index(), UnChecked_Status, ITEM_STATE);
			}
			else if (iCheckCount == pParentItem->rowCount())
			{
				m_pTreeModel->setData(pParentItem->index(), Checked_Status, ITEM_STATE);
			}
			else
			{
				m_pTreeModel->setData(pParentItem->index(), PartiallyChecked_Status, ITEM_STATE);
			}
			stackItems.push(pParentItem);
		}
	}
}

void CustomTreeWidget::InitItemWidgetType(TreeItemWidgetType type)
{
	switch (type)
	{
	case ColorRect_CheckBox_With:
	{
		m_bIsExistCheckBox = true;
	}
	break;
	case ColorRect_CheckBox_Without:
	{
		m_bIsExistCheckBox = false;
	}
	break;
	case ColorRect_Menu_With:
	{
		m_bIsExistCheckBox = false;
		m_bIsShowMenu = true;
	}
	break;
	case FolderRect:
	{
		m_bIsExistCheckBox = false;
	}
	break;
	case FolderRect_CheckBox_With:
	{
		m_bIsExistCheckBox = true;
	}
	break;
	default:
		break;
	}
}

void CustomTreeWidget::ExpandSelectItem(QStringList lstPath, int index, QStandardItem *pParentItem, QString strUUid /*= ""*/)
{
	if (lstPath.size() <= index || NULL == pParentItem) return;
	int iRowCount = pParentItem->rowCount();
	for (int iRow = 0; iRow < iRowCount; ++iRow)
	{
		QStandardItem *pChildItem = pParentItem->child(iRow, 0);
		if (pChildItem)
		{
			QString strText = m_pTreeModel->data(pChildItem->index(), ITEM_TEXT).toString();
			QString strChildUUid = m_pTreeModel->data(pChildItem->index(), ITEM_UUID).toString();
			int iType = m_pTreeModel->data(pChildItem->index(), ITEM_TYPE).toInt();
			if (RootNode_Device == iType && strChildUUid == strUUid)
			{
				m_pTreeModel->setData(pChildItem->index(), Checked_Status, ITEM_STATE);
				UpdateParentState(pChildItem);
				return;
			}
			if (lstPath.at(index) == strText)
			{
				m_pTreeView->expand(pChildItem->index());
				ExpandSelectItem(lstPath, index + 1, pChildItem, strUUid);
				break;
			}
		}
	}
}

void CustomTreeWidget::UpdateParentLevel(QStandardItem *pChildItem)
{
	if (NULL == pChildItem) return;
	QStandardItem *pParentItem = pChildItem->parent();
	if (pParentItem)
	{
		int iRowCount = pParentItem->rowCount();
		int iMaxLevel = Alarm_NONE;
		int iNoIdentifyCount = 0;
		for (int iRow = 0; iRow < iRowCount; ++iRow)
		{
			QStandardItem *pCurChild = pParentItem->child(iRow, 0);
			if (pCurChild)
			{
				int iCurLevel = m_pTreeModel->data(pCurChild->index(), ITEM_LEVEL).toInt();
				if (iMaxLevel != Alarm_NoIdentifyAbnormal && iMaxLevel < iCurLevel && iCurLevel != Alarm_NoIdentifyAbnormal)
				{
					iMaxLevel = iCurLevel;
				}
				if (iCurLevel == Alarm_NoIdentifyAbnormal)
				{
					++iNoIdentifyCount;
				}
			}
		}
		if (iNoIdentifyCount == iRowCount)
		{
			m_pTreeModel->setData(pParentItem->index(), Alarm_NoIdentifyAbnormal, ITEM_LEVEL);
		}
		else
		{
			m_pTreeModel->setData(pParentItem->index(), iMaxLevel, ITEM_LEVEL);
		}
		UpdateParentLevel(pParentItem);


//		int iParentLevel = m_pTreeModel->data(pParentItem->index(), ITEM_LEVEL).toInt();
// 		if (iParentLevel < iLevel)
// 		{
// 			iParentLevel = iLevel;
// 			m_pTreeModel->setData(pParentItem->index(), iParentLevel, ITEM_LEVEL);
// 			UpdateParentLevel(pParentItem, iLevel);
// 		}
	}
}

QString CustomTreeWidget::getItemDeviceId(QString itemText, RootNodeType rootNodeType)
{
	QString strDeviceId;
	switch (rootNodeType)
	{
	case RootNodeType::RootNode_VoltageLevel:
		strDeviceId = WHEEL_DEVICE_CONFIG.getWheelVoltageLevelIdInt(itemText);
		break;
	case RootNodeType::RootNode_Interval:
		strDeviceId = WHEEL_DEVICE_CONFIG.getWheelEquipmentIntervalQString(itemText);
		break;
	case RootNodeType::RootNode_DeviceType:
		strDeviceId = WHEEL_DEVICE_CONFIG.getWheelDeviceTypeUUidQString(itemText);
		break;
// 	case RootNodeType::RootNode_Device:
// 		strDeviceId = m_itemDeviceId;
// 		break;
	default:
		break;
	}

	return strDeviceId;
}

void CustomTreeWidget::updataTreeData(WheelPatrolParameter m_wheelPatrolPara)
{
	// 更新树控件数据，刷新树控件显示;
	//m_choosedDeviceIdMap.clear();
	refreshTreeItemList(m_wheelPatrolPara);
}

void CustomTreeWidget::addItemToTree(QString strDeviceId, QList<WheelRobortAlarmPathStruct> treeItemData)
{//插入一条节点

	QStandardItem *pRootItem = m_pTreeModel->item(0, 0);
	if (NULL != pRootItem)
	{
		m_pTreeModel->setData(pRootItem->index(), DeviceAlarmLevel(treeItemData[0].alarm_level_id), ITEM_LEVEL);

		QStandardItem *pStationItem = CheckAddItem(pRootItem, treeItemData[1].alarm_path_name, treeItemData[1].alarm_level_id, Robot_Station);
		if (pStationItem)
		{
			QStandardItem *pVoltageLevelItem = CheckAddItem(pStationItem, treeItemData[2].alarm_path_name, treeItemData[2].alarm_level_id, RootNode_VoltageLevel);
			if (pVoltageLevelItem)
			{
				QStandardItem *pIntervalItem = CheckAddItem(pVoltageLevelItem, treeItemData[3].alarm_path_name, treeItemData[3].alarm_level_id, RootNode_Interval);
				if (pIntervalItem)
				{
					QStandardItem *pDeviceTypeItem = CheckAddItem(pIntervalItem, treeItemData[4].alarm_path_name, treeItemData[4].alarm_level_id, RootNode_DeviceType);
					if (pDeviceTypeItem)
					{
						QStandardItem *pChildItem = CheckAddItem(pIntervalItem, treeItemData[5].alarm_path_name, treeItemData[5].alarm_level_id, RootNode_Device, strDeviceId);
					}
				}
			}
		}
	}
}

void CustomTreeWidget::onDeleteItem()
{//删除节点
	QModelIndex index = m_pTreeView->currentIndex();
	int iType = m_pTreeModel->data(index, ITEM_TYPE).toInt();
	QString strText = m_pTreeModel->data(index, ITEM_TEXT).toString();
	QString strDeviceId = "";
	QString strParentId = "";
	switch (iType)
	{
	case RootNode_VoltageLevel:
		strDeviceId = getItemDeviceId(strText, RootNode_VoltageLevel);
		break;
	case RootNode_Interval:
		strDeviceId = getItemDeviceId(strText, RootNode_Interval);
		break;
	case RootNode_DeviceType:
	{
		strDeviceId = getItemDeviceId(strText, RootNode_DeviceType);
		QStandardItem *pParentItem = m_pTreeModel->itemFromIndex(index)->parent();
		if (pParentItem)
		{
			strParentId = getItemDeviceId(m_pTreeModel->data(pParentItem->index(), ITEM_TEXT).toString(), RootNode_Interval);
		}
	}
	break;
	case RootNode_Device:
		strDeviceId = m_pTreeModel->data(index, ITEM_UUID).toString();
		break;
	default:
		break;
	}

	emit signalDeleteTreeItemNode((RootNodeType)iType, strDeviceId, strParentId);
}


void CustomTreeWidget::expandTreeNodeByDevice(QStringList strDevicePath, QString strDeviceId)
{//右侧表格双击操作

	qDebug() << "path:" << strDevicePath << "; UUID:" << strDeviceId;

	QStandardItem *pRootItem = m_pTreeModel->item(0, 0);
	if (pRootItem)
	{
		ExpandSelectItem(strDevicePath, 1, pRootItem, strDeviceId);
	}

//     QTreeWidgetItem* nodeItem = m_stationItem;
//     // 从电压等级开始进行展开;
//     for (int i = 2; i < strDevicePath.count(); i++)
//     {
//         for (int j = 0; j < nodeItem->childCount(); j++)
//         {
//             QTreeWidgetItem* childItem = nodeItem->child(j);
//             ItemWidget* itemWidget = static_cast<ItemWidget*>(m_treeWidget->itemWidget(childItem, 0));
//             if (itemWidget->getItemText() == strDevicePath[i])
//             {
//                 childItem->setExpanded(true);
//                 // 如果展开没有孩子，则当前为设备，将设备勾选上;
//                 if (childItem->childCount() == 0)
//                 {
//                     itemWidget->updateCheckBoxState(true);
//                 }
//                 nodeItem = childItem;
//                 break;
//             }
//         }
//     }   
}

// 
// void CustomTreeWidget::updateAlarmLevel(int currentLevel)
// {
// 	if (currentLevel > m_deviceTypeAlarmLevel)
// 	{
// 		m_deviceTypeAlarmLevel = currentLevel;
// 	}
// 	if (currentLevel > m_intervalAlarmLevel)
// 	{
// 		m_intervalAlarmLevel = currentLevel;
// 	}
// 	if (currentLevel > m_voltageLevelAlarmLevel)
// 	{
// 		m_voltageLevelAlarmLevel = currentLevel;
// 	}
// }

QStandardItem *CustomTreeWidget::CheckAddItem(QStandardItem *pParentItem, const QString &strPath, int iLevel, int iType, QString strUUid)
{
	if (NULL == pParentItem) return NULL;
	QStandardItem *pItem = NULL;
	bool bIsExist = false;
	int iRowCount = pParentItem->rowCount();
	int iRow = 0;
	for (; iRow < iRowCount; ++iRow)
	{
		QStandardItem *pCurItem = pParentItem->child(iRow, 0);
		if (m_pTreeModel->data(pCurItem->index(), ITEM_TEXT).toString() == strPath)
		{
			pItem = pCurItem;
			bIsExist = true;
		}
	}
	if (iRow == iRowCount && !bIsExist)
	{
		pItem = new QStandardItem("");
		pParentItem->setChild(iRowCount, pItem);
		m_pTreeModel->setData(pItem->index(), QVariant(iLevel), ITEM_LEVEL);
		m_pTreeModel->setData(pItem->index(), QVariant(iType), ITEM_TYPE);
		m_pTreeModel->setData(pItem->index(), QVariant(strPath), ITEM_TEXT);
		m_pTreeModel->setData(pItem->index(), QVariant(strUUid), ITEM_UUID);
		m_pTreeModel->setData(pItem->index(), UnChecked_Status, ITEM_STATE);
	}
	return pItem;
}

