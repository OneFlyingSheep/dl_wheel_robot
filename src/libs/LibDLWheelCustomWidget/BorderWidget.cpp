#include "BorderWidget.h"
#include <QVBoxLayout>
#include <QPainter>

BorderWidget::BorderWidget(QWidget* parent /* = NULL */)
	: QWidget(parent)
{
	initWidget();
	this->setStyleSheet("QWidget#CenterWidget{border:2px solid #59908B;}");
}

void BorderWidget::initWidget()
{
	m_titleLabel = new QLabel;
	m_titleLabel->setAlignment(Qt::AlignCenter);
	m_titleLabel->setStyleSheet("font-size:22px;color:red;font-weight:bold;");

	m_centerWidget = new QWidget;
	m_centerWidget->setObjectName("CenterWidget");

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_titleLabel);
	vMainLayout->addWidget(m_centerWidget);
	vMainLayout->setSpacing(15);
	vMainLayout->setMargin(0);
}

void BorderWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), Qt::white);
}

void BorderWidget::setTitleText(const QString& text)
{
	m_titleLabel->setText(text);
}

QWidget* BorderWidget::getCenterWidget()
{
	return m_centerWidget;
}