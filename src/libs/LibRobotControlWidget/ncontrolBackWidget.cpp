

#include "ncontrolBackWidget.h"

NControlBackWidget::NControlBackWidget (QWidget* parent)
{
	m_pButtonText = new QPushButton();
	m_pButtonText->setFixedHeight(30);
	m_pButtonText->setStyleSheet("font-weight:bold;border:1px solid rgb(59,80,206);background:rgb(175,191,255);");
	m_centerWidget = new QWidget;
	QVBoxLayout* vLayout = new QVBoxLayout(this);
	vLayout->addWidget(m_pButtonText);
	vLayout->addWidget(m_centerWidget);
	vLayout->setMargin(0);
	vLayout->setSpacing(0);

	this->setFixedHeight(200);
}

// 设置标题;
void NControlBackWidget::setTitleText(QString text)
{
	m_pButtonText->setText(text);
}

// 设置中心Widget;
void NControlBackWidget::setCenterWidget(QWidget* widget)
{
	QHBoxLayout* hCenterLayout = new QHBoxLayout(m_centerWidget);
	hCenterLayout->addStretch();
	hCenterLayout->addWidget(widget);
	hCenterLayout->addStretch();
	hCenterLayout->setMargin(5);
}
// 绘制边框;
void NControlBackWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), Qt::white);
	painter.setPen(QPen(Qt::lightGray, 1));
	painter.drawRect(QRect(0, 0, this->width() - 1, this->height() - 1));

	return __super::paintEvent(event);
}

