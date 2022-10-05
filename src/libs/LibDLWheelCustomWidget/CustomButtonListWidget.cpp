#include "CustomButtonListWidget.h"
#include <QPainter>

CustomButtonListWidget::CustomButtonListWidget(QWidget* parent /* = NULL */, int iButtonDispalyType)
	: QWidget(parent)
	, m_iButtonDisplayType(iButtonDispalyType)
{
	m_pToolButtonGroup = new QButtonGroup(this);
	connect(m_pToolButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &CustomButtonListWidget::signalButtonClicked);
	this->setFixedHeight(30);
	this->setStyleSheet("QToolButton{border:none;}");
}

void CustomButtonListWidget::addWidget(QWidget* childWidget)
{
	m_widgetList.append(childWidget);
}

void CustomButtonListWidget::addToolButton(int buttonId, QString buttonText, QString iconPath, QSize buttonSize /* = QSize(60, 20) */, QSize IconSize /* = QSize(16, 16) */)
{
	QToolButton* pButton = new QToolButton;
	pButton->setText(buttonText);
	pButton->setIcon(QIcon(iconPath));
	pButton->setFixedSize(buttonSize);
	pButton->setIconSize(IconSize);
	pButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pToolButtonGroup->addButton(pButton, buttonId);
	m_widgetList.append(pButton);
}

void CustomButtonListWidget::addWidgetFinished()
{
	QHBoxLayout* hSearchButtonLayout = new QHBoxLayout(this);
	if (RIGHT_START_DISPLAY == m_iButtonDisplayType)
	{
		hSearchButtonLayout->addStretch();
	}
	int i = 0;
	for (; i < m_widgetList.count() - 1; i++)
	{
		hSearchButtonLayout->addWidget(m_widgetList.at(i));

		QLabel* separatorLabel = new QLabel;
		separatorLabel->setFrameShape(QFrame::VLine);
		separatorLabel->setFixedSize(QSize(1, 16));
		separatorLabel->setStyleSheet("border:1px solid white;");
		hSearchButtonLayout->addWidget(separatorLabel);
	}
	hSearchButtonLayout->addWidget(m_widgetList.at(i));
	if (LEFT_START_DISPLAY == m_iButtonDisplayType)
	{
		hSearchButtonLayout->addStretch();
	}
	hSearchButtonLayout->setContentsMargins(3, 0, 0, 0);
	hSearchButtonLayout->setSpacing(2);
}

void CustomButtonListWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), QColor(175, 191, 255));
	painter.setPen(QColor(166, 233, 210));
	painter.drawRect(0, 0, this->width() - 1, this->height() - 1);
}