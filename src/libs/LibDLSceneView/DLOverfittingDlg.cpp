#include "DLOverfittingDlg.h"
#include "ui_DLOverfittingDlg.h"

#include "DLCustomScene.h"
#include "DLOperator.h"

DLOverfittingDlg::DLOverfittingDlg(QWidget *parent)
	:QDialog(parent)
    ,ui(new Ui::DLOverfittingDlg)
	, m_pCustomScene(NULL)
{
	ui->setupUi(this);
	setWindowTitle("ÄâºÏ");

	this->setWindowModality(Qt::WindowModal);

	connect(ui->selectLineBtn, SIGNAL(clicked()), this, SLOT(SelectLineBtnSlot()));
	connect(ui->selectPointsBtn, SIGNAL(clicked()), this, SLOT(SelectPointsBtnSlot()));
	connect(ui->startOverfittingBtn, SIGNAL(clicked()), this, SLOT(StartOverfittingBtnSlot()));
	connect(ui->cancelBtn, SIGNAL(clicked()), this, SLOT(CancelBtnSlot()));
}


DLOverfittingDlg::~DLOverfittingDlg()
{

}

void DLOverfittingDlg::SetScene(DLCustomScene *pCustomScene)
{
	m_pCustomScene = pCustomScene;
}

void DLOverfittingDlg::SetSelectLineID(const QString &strLineName)
{
	ui->leLine->setText(strLineName);
}

void DLOverfittingDlg::SetSelectPointIDs(const QString &strPointName)
{
	ui->lePoints->setText(strPointName);
}

void DLOverfittingDlg::ClearContent()
{
	ui->leLine->clear();
	ui->lePoints->clear();
}

void DLOverfittingDlg::SelectLineBtnSlot()
{
	if (NULL != m_pCustomScene)
	{
		m_pCustomScene->ClearItemFocus();
		m_pCustomScene->SetOperateType(DLOperator::TYPE_OVERFITTING_SELECT_LINE);
	}
	hide();
}

void DLOverfittingDlg::SelectPointsBtnSlot()
{
	if (NULL != m_pCustomScene)
	{
		m_pCustomScene->SetOperateType(DLOperator::TYPE_OVERFITTING_SELECT_POINT);
	}
	hide();
}



void DLOverfittingDlg::StartOverfittingBtnSlot()
{
	if (m_pCustomScene)
	{
		m_pCustomScene->StartOverfitting();
	}
	close();
}

void DLOverfittingDlg::CancelBtnSlot()
{
	if (m_pCustomScene)
	{
		m_pCustomScene->CancelOverfitting();
	}
	close();
}
