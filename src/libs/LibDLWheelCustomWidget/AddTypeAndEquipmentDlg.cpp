#include "AddTypeAndEquipmentDlg.h"
#include "ui_AddTypeAndEquipmentDlg.h"
#include "ListSearch.h"

#define USER_ROLE_ID (Qt::UserRole + 1)
#define USER_ROLE_DEVICE_ID (Qt::UserRole + 2)
AddTypeAndEquipmentDlg::AddTypeAndEquipmentDlg(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::AddTypeAndEquipmentDlg)
	, m_iWgtType(ADD_TYPE)
{
    ui->setupUi(this);

	connect(ui->pushBtnOK, SIGNAL(clicked()), this, SIGNAL(AddEquipmentTypeOkBtnSignal()));
	connect(ui->pushBtnCancel, SIGNAL(clicked()), this, SIGNAL(AddEquipmentTypeCancelBtnSignal()));

	connect(ui->pushBtnAddAll, SIGNAL(clicked()), this, SLOT(AddAllBtnSlot()));
	connect(ui->pushBtnAddOne, SIGNAL(clicked()), this, SLOT(AddOneBtnSlot()));
	connect(ui->pushBtnDelOne, SIGNAL(clicked()), this, SLOT(DelOneBtnSlot()));
	connect(ui->pushBtnDelAll, SIGNAL(clicked()), this, SLOT(DelAllBtnSlot()));

	connect(ui->lstWgtAll, SIGNAL(itemDoubleClicked(QListWidgetItem *)), this, SLOT(AllLstWgtDouClickedSlot(QListWidgetItem *)));
	connect(ui->lstWgtSelect, SIGNAL(itemDoubleClicked(QListWidgetItem *)), this, SLOT(SelectLstWgtDouClickedSlot(QListWidgetItem *)));

	connect(ui->leAllSearch, SIGNAL(textEdited(const QString &)), this, SLOT(SearchAllEditSlot(const QString &)));
	connect(ui->leSelectSearch, SIGNAL(textEdited(const QString &)), this, SLOT(SearchSelectEditSlot(const QString &)));
}

AddTypeAndEquipmentDlg::~AddTypeAndEquipmentDlg()
{
    delete ui;
}

void AddTypeAndEquipmentDlg::ClearContent()
{
	ui->lstWgtAll->clear();
	ui->lstWgtSelect->clear();
	ui->leAllSearch->clear();
	ui->leSelectSearch->clear();
}

void AddTypeAndEquipmentDlg::SetWgtType(int iType)
{
	m_iWgtType = iType;
}

int AddTypeAndEquipmentDlg::GetWgtType()
{
	return m_iWgtType;
}

void AddTypeAndEquipmentDlg::SetDatas(QList<QStringList> lstAllDatas, QList<QStringList> lstSelectDatas)
{
	m_lstAllDatas = lstAllDatas;
	foreach(QStringList lstAllData, lstAllDatas)
	{
		if (lstAllData.size() > 1)
		{
			QListWidgetItem *pItem = new QListWidgetItem(lstAllData.at(1));
			pItem->setData(USER_ROLE_ID, lstAllData.at(0));
			//pItem->setTextAlignment(Qt::AlignCenter);
			ui->lstWgtAll->addItem(pItem);
		}
		
	}

	foreach(QStringList lstSelectData, lstSelectDatas)
	{
		QListWidgetItem *pItem = new QListWidgetItem(lstSelectData.at(1));
		pItem->setData(USER_ROLE_ID, lstSelectData.at(0));
		pItem->setData(USER_ROLE_DEVICE_ID, lstSelectData.at(2));
		ui->lstWgtSelect->addItem(pItem);
	}
}

QList<QStringList> AddTypeAndEquipmentDlg::GetDatas()
{
	QList<QStringList> lstSelectDatas;
	for (int iRow = 0; iRow < ui->lstWgtSelect->count(); ++iRow)
	{
		QStringList lstSelectData;
		QListWidgetItem *pItem = ui->lstWgtSelect->item(iRow);
		if (pItem)
		{
			lstSelectData << pItem->data(USER_ROLE_ID).toString()
				<< pItem->text() << pItem->data(USER_ROLE_DEVICE_ID).toString();
		}
		lstSelectDatas.push_back(lstSelectData);
	}
	return lstSelectDatas;
}

void AddTypeAndEquipmentDlg::AddAllBtnSlot()
{//->>
	ui->lstWgtSelect->clear();
	foreach(QStringList lstSelectData, m_lstAllDatas)
	{
		QListWidgetItem *pItem = new QListWidgetItem(lstSelectData.at(1));
		pItem->setData(USER_ROLE_ID, lstSelectData.at(0));
		ui->lstWgtSelect->addItem(pItem);
	}
}

void AddTypeAndEquipmentDlg::AddOneBtnSlot()
{//->
	QList<QStringList> lstSelectDatas;
	QList<QListWidgetItem *> lstSelectItems = ui->lstWgtAll->selectedItems();
	foreach(QListWidgetItem *pItem, lstSelectItems)
	{
		QStringList lstSelectData;
		lstSelectData << pItem->data(USER_ROLE_ID).toString()
			<< pItem->text();
		lstSelectDatas.push_back(lstSelectData);
	}

	foreach(QStringList lstSelectData, lstSelectDatas)
	{
		int iRow = 0;
		for (; iRow < ui->lstWgtSelect->count(); ++iRow)
		{
			QListWidgetItem *pItem = ui->lstWgtSelect->item(iRow);
			if (pItem)
			{
				QString strID = pItem->data(USER_ROLE_ID).toString();
				if (lstSelectData.at(0) == strID)
				{
					break;
				}
			}
		}

		if (iRow == ui->lstWgtSelect->count())
		{//Ã»ÕÒµ½
			QListWidgetItem *pItem = new QListWidgetItem(lstSelectData.at(1));
			pItem->setData(USER_ROLE_ID, lstSelectData.at(0));
			ui->lstWgtSelect->addItem(pItem);
		}
	}


}

void AddTypeAndEquipmentDlg::DelOneBtnSlot()
{//<-
	QList<QListWidgetItem *> lstSelectItems = ui->lstWgtSelect->selectedItems();
	foreach(QListWidgetItem *pItem, lstSelectItems)
	{
		ui->lstWgtSelect->takeItem(ui->lstWgtSelect->row(pItem));
		delete pItem;
		pItem = NULL;
	}
}

void AddTypeAndEquipmentDlg::DelAllBtnSlot()
{//<<-
	ui->lstWgtSelect->clear();
}

void AddTypeAndEquipmentDlg::AllLstWgtDouClickedSlot(QListWidgetItem *pCurItem)
{
	if (NULL == pCurItem) return;
	QStringList lstSelectData;
	lstSelectData << pCurItem->data(USER_ROLE_ID).toString()
		<< pCurItem->text();

	int iRow = 0;
	for (; iRow < ui->lstWgtSelect->count(); ++iRow)
	{
		QListWidgetItem *pItem = ui->lstWgtSelect->item(iRow);
		if (pItem)
		{
			QString strID = pItem->data(USER_ROLE_ID).toString();
			if (lstSelectData.at(0) == strID)
			{
				return;
			}
		}
	}

	QListWidgetItem *pItem = new QListWidgetItem(lstSelectData.at(1));
	pItem->setData(USER_ROLE_ID, lstSelectData.at(0));
	ui->lstWgtSelect->addItem(pItem);

}

void AddTypeAndEquipmentDlg::SelectLstWgtDouClickedSlot(QListWidgetItem *pCurItem)
{
	if (NULL == pCurItem) return;
	ui->lstWgtSelect->takeItem(ui->lstWgtSelect->row(pCurItem));
	delete pCurItem;
	pCurItem = NULL;
}

void AddTypeAndEquipmentDlg::SearchAllEditSlot(const QString &strText)
{
	ListSearch::SearchItem(ui->lstWgtAll, strText);
}

void AddTypeAndEquipmentDlg::SearchSelectEditSlot(const QString &strText)
{
	ListSearch::SearchItem(ui->lstWgtSelect, strText);
}
