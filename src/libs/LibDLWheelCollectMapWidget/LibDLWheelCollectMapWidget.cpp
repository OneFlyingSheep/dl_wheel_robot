#include <QHBoxLayout>
#include "LibDLWheelCollectMapWidget.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLMapEditorWidget/DLMapEditorWidget.h"

#pragma execution_character_set("utf-8")

DLWheelCollectMapWidget::DLWheelCollectMapWidget(bool isDevelopProgram /* = true */, QWidget *parent /* = NULL */)
    : m_isDevelopProgram(isDevelopProgram)
	, m_pMapEditorWidget(NULL)
{
	initWidget();

	this->setStyleSheet("QPushButton#CommonButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton#CommonButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton#CommonButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}\
						QPushButton#CommonButton:disabled{background:gray;}");		
}

DLWheelCollectMapWidget::~DLWheelCollectMapWidget()
{

}

void DLWheelCollectMapWidget::ClearMap()
{
	if (m_pMapEditorWidget)
	{
		m_pMapEditorWidget->ClearMap();
	}
}

void DLWheelCollectMapWidget::initWidget()
{
	m_pMapEditorWidget = new DLMapEditorWidget;					//编辑+地图
    if (m_isDevelopProgram)
    {
        m_pCollectMapWidget = new BaseWidget;
        m_pCollectMapWidget->setTitleContent("采集地图");

        QHBoxLayout* hMainLayout = new QHBoxLayout(this);
        hMainLayout->addWidget(m_pCollectMapWidget);
        hMainLayout->setSpacing(0);
        hMainLayout->setMargin(10);

        QHBoxLayout *mapEditor_layout = new QHBoxLayout(m_pCollectMapWidget->getCenterWidget());
        mapEditor_layout->addWidget(m_pMapEditorWidget);
        mapEditor_layout->setMargin(0);
    }
    else
    {
        QHBoxLayout *mapEditor_layout = new QHBoxLayout(this);
        mapEditor_layout->addWidget(m_pMapEditorWidget);
        mapEditor_layout->setMargin(0);
    }
	
	
    connect(this, SIGNAL(signalSendCollectPatrolPoint()), m_pMapEditorWidget, SLOT(slot_on_add_landmark()));
    connect(this, SIGNAL(signalSendOpenSmap(QString)), m_pMapEditorWidget, SLOT(slot_on_open_smap(QString)));
	connect(m_pMapEditorWidget, SIGNAL(OpenSmapFileSignal(QString)), this, SIGNAL(OpenSmapFileSignal(QString)));
}