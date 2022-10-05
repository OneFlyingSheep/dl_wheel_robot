#include <QPainter>
#include <QDebug>
#include "ConfidenceValueShowWidget.h"

ConfidenceValueShowWidget::ConfidenceValueShowWidget(QWidget* parent /* = NULL */)
    : QWidget(parent)
    , m_value(0)
    , m_isFlash(false)
{
    this->setFixedSize(QSize(45, 23));
    //this->setAttribute(Qt::WA_TranslucentBackground);

    // 置信度低于30开始闪烁;
	m_pFlashTimer = new QTimer(this);
	m_pFlashTimer->setInterval(1000);
	connect(m_pFlashTimer, SIGNAL(timeout()), this, SLOT(TimeOutSlot()));
}

void ConfidenceValueShowWidget::setCurrentValue(float value)
{
	if (m_value == value * 100) return;
    m_value = value * 100;
    if (m_value < 30 && !m_pFlashTimer->isActive())
    {
		m_pFlashTimer->start();
    }
    else if(m_value >= 30 && m_pFlashTimer->isActive())
    {
		m_pFlashTimer->stop();
    }
    update();
}

void ConfidenceValueShowWidget::TimeOutSlot()
{
	m_isFlash = !m_isFlash;
	repaint();
}

void ConfidenceValueShowWidget::paintEvent(QPaintEvent *event)
{
    QColor drawColor;
    if (m_value < 30)
    {
        if (m_isFlash)
        {
            drawColor = Qt::red;
        }
        else
        {
            drawColor = Qt::black;
        }
    }
    else if (m_value < 60)
    {
        drawColor = Qt::magenta;
    }
    else
    {
        drawColor = Qt::green;
    }

    QPainter painter(this);
    painter.setPen(QPen(drawColor, 3));
   // painter.drawRect(QRect(1, 1, this->width() - 3, this->height() - 3));
    QFont font = painter.font();
    if (m_value >= 100)
    {
        m_value = 100;
        font.setPixelSize(16);
    }
    else
    {
        font.setPixelSize(16);
    }
	font.setBold(true);

    painter.setFont(font);
    painter.drawText(this->rect(), Qt::AlignCenter, QString::number(m_value) + "%");
}