#include "DLWheelMainLineShow.h"
#include <QDateTime>
#include <QCalendarWidget>
#include "DLWheelMainLineGraphicsView.h"

#pragma execution_character_set("utf-8")

DLWheelMainLineShow::DLWheelMainLineShow()
	: m_isInitWidget(false)
{
	this->setStyleSheet("QWidget#TopBackWidget{background:white;}\
							QWidget#CenterBackWidget{background:white;}\
							QWidget#BottomBackWidget{background:rgb(159,168,218);}");
}

void DLWheelMainLineShow::initTopWidget()
{
	m_searchLineEdit = new QLineEdit;
	m_searchLineEdit->setFixedSize(QSize(200, 25));
	m_searchLineEdit->setPlaceholderText("请输入间隔名称");

	m_pButtonSearch = new QToolButton;
	m_pButtonSearch->setStyleSheet("border:1px solid gray;background:lightgray;");
	m_pButtonSearch->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonSearch->setIcon(QIcon(":/Resources/Common/image/Search.png"));
	m_pButtonSearch->setIconSize(QSize(16, 16));
	m_pButtonSearch->setFixedSize(QSize(60, 20));
	m_pButtonSearch->setText("查询");

	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("TopBackWidget");
	m_topBackWidget->setFixedHeight(30);
	QHBoxLayout* hTopLayout = new QHBoxLayout(m_topBackWidget);
	hTopLayout->addWidget(m_searchLineEdit);
	hTopLayout->addWidget(m_pButtonSearch);
	hTopLayout->addStretch();
	hTopLayout->setSpacing(5);
	hTopLayout->setContentsMargins(5, 0, 0, 0);
}

void DLWheelMainLineShow::initCenterWidget()
{
	m_centerWidget = new QWidget;
	m_centerWidget->setObjectName("CenterBackWidget");
	m_centerWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    m_dLWheelMainLineGraphicsView = new DLWheelMainLineGraphicsView;

    QHBoxLayout* hCenterLayout = new QHBoxLayout(m_centerWidget);
    hCenterLayout->addWidget(m_dLWheelMainLineGraphicsView);
    hCenterLayout->setMargin(50);
}

void DLWheelMainLineShow::initBottomWidget()
{
	m_bottomBackWidget = new QWidget;
	m_bottomBackWidget->setObjectName("BottomBackWidget");
	m_bottomBackWidget->setFixedHeight(30);

	QLabel* tipLabel = new QLabel;
	tipLabel->setText("提示:");

	QHBoxLayout* hBottomLayout = new QHBoxLayout(m_bottomBackWidget);
	hBottomLayout->addWidget(tipLabel);
	ColorLabel* colorLabel[6];
	for (int i = 0; i < 6; i++)
	{
		colorLabel[i] = new ColorLabel;
		hBottomLayout->addWidget(colorLabel[i]);
	}
	colorLabel[0]->setText("正常状态");
	colorLabel[0]->setColor(QColor(0, 128, 0));
	colorLabel[1]->setText("预警");
	colorLabel[1]->setColor(Qt::blue);
	colorLabel[2]->setText("一般告警");
	colorLabel[2]->setColor(QColor(255, 255, 0));
	colorLabel[3]->setText("严重警告");
	colorLabel[3]->setColor(QColor(255, 128, 20));
	colorLabel[4]->setText("危急告警");
	colorLabel[4]->setColor(Qt::red);
	colorLabel[5]->setText("未识别异常");
	colorLabel[5]->setColor(Qt::gray);
	hBottomLayout->addStretch();
	hBottomLayout->setSpacing(5);
	hBottomLayout->setContentsMargins(5, 0, 0, 0);
}

void DLWheelMainLineShow::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initTopWidget();
	initCenterWidget();
	initBottomWidget();

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_topBackWidget);
	vMainLayout->addWidget(m_centerWidget);
	vMainLayout->addWidget(m_bottomBackWidget);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(0);
}