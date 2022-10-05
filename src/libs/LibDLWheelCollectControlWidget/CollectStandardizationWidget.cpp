#include "CollectStandardizationWidget.h"
#include "ui_CollectStandardizationWidget.h"
#include "PTCalculate.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include <QDir>
#include <QFile>
#include <QMessageBox>
#include <QFileDialog>
#include "LibHCNetCamera/HikCameraPointData.h"

CollectStandardizationWidget::CollectStandardizationWidget(QWidget *parent) :
	QWidget(parent),
	ui(new Ui::CollectStandardizationWidget)
{
	ui->setupUi(this);
	connect(ui->pB_StartCollectPhoto, &QPushButton::clicked, this, &CollectStandardizationWidget::startCollectPhotoSignals);
	connect(ui->pB_CalculateZoomValue, &QPushButton::clicked, this, &CollectStandardizationWidget::CalculateZoomValue);
	connect(ui->pB_CutZoomTxt, &QPushButton::clicked, this, &CollectStandardizationWidget::CutZoomTxt);
	connect(ui->cB_DefaultsTxt, &QCheckBox::stateChanged, this, &CollectStandardizationWidget::DefaultTxt);
}

CollectStandardizationWidget::~CollectStandardizationWidget()
{
	delete ui;
}

void CollectStandardizationWidget::CalculateZoomValue()
{
	//图片路径文件夹路径
	QString strFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/ZoomCaptureImage/image/";
	QDir dir;
	if (!dir.exists(strFilePath))
	{
		dir.mkpath(strFilePath);
	}

	//生成txt目录，需包含具体名字
	QString strFileNamePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/ZoomCaptureImage/zoomValue/";
	if (!dir.exists(strFileNamePath))
	{
		dir.mkpath(strFileNamePath);
	}

	QString txtName = ui->lE_TxtName->text();
	if (txtName.isEmpty())
	{
		txtName = ui->lE_TxtName->placeholderText();
	}
	QString fileName = strFileNamePath + txtName + QString(".txt");

	QFile file;
	if (file.exists(fileName))
	{
		QMessageBox message(QMessageBox::NoIcon, "提示", "该文件已存在，是否替换！", QMessageBox::Yes | QMessageBox::No, NULL);
		if (message.exec() == QMessageBox::No)
		{
			return;
		}
	}
	PTCalculate ptz;
	ptz.CalculatePT(strFilePath.toStdString(), fileName.toStdString());
}

void CollectStandardizationWidget::CutZoomTxt()
{
	QString strFilePath = QFileDialog::getOpenFileName(this, "选择文件", QString("%1/ZoomCaptureImage/zoomValue/").arg(WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath), "text(*.txt)");
	if (strFilePath.isEmpty())
	{
		return;
	}
	HikPointData::getInitance()->readZoomPtzValueFromTxt(strFilePath);
}

void CollectStandardizationWidget::DefaultTxt()
{
//	int i = ui->cB_DefaultsTxt->checkable();
	if (ui->cB_DefaultsTxt->isChecked())
	{
		HikPointData::getInitance()->setIsUseDefaultsZoom(true);
	}
	else
	{
		HikPointData::getInitance()->setIsUseDefaultsZoom(false);
	}	
}