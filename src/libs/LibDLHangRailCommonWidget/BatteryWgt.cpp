#include "BatteryWgt.h"
#include <QPainter>

#define BATTER_MARGIN (1)

BatteryWgt::BatteryWgt(QWidget* parent /* = NULL */)
    : QWidget(parent)
    , m_iValue(0)
    , m_bIsFlash(false)
{
    this->setFixedSize(QSize(45, 23));
    this->setAttribute(Qt::WA_TranslucentBackground);

    // 置信度低于30开始闪烁;
	m_pFlashTimer = new QTimer(this);
    m_pFlashTimer->setInterval(600);
	connect(m_pFlashTimer, &QTimer::timeout, this, [=] {
		m_bIsFlash = !m_bIsFlash;
		update();
	});
}

void BatteryWgt::setCurrentValue(float value)
{
    m_iValue = value * 100;
    if (m_iValue < 30)
    {
		m_pFlashTimer->start();
    }
    else
    {
		m_pFlashTimer->stop();
    }
    update();
}

void BatteryWgt::paintEvent(QPaintEvent *event)
{
    QColor drawColor;
    if (m_iValue < 30)
    {
        if (m_bIsFlash)
        {
            drawColor = Qt::red;
        }
        else
        {
            drawColor = Qt::black;
        }
    }
    else if (m_iValue < 60)
    {
        drawColor = Qt::magenta;
    }
    else
    {
        drawColor = Qt::green;
    }


    QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	qreal rHeight = height() - BATTER_MARGIN * 2 - 2;
	//painter.drawRect(1, 1, 16, 16);
	painter.save();
	painter.drawRect(BATTER_MARGIN, BATTER_MARGIN + 2, width() - BATTER_MARGIN * 2, height() - BATTER_MARGIN * 2 - 2);
	//painter.drawRoundedRect(BATTER_MARGIN, BATTER_MARGIN + 2, width() - BATTER_MARGIN * 2, height() - BATTER_MARGIN * 2 - 2, 1, 1);
	painter.restore();
	painter.save();
	painter.setBrush(Qt::black);
	QRect rect(BATTER_MARGIN + 3, BATTER_MARGIN, 5, 2);
	painter.drawRect(rect);
	painter.restore();

	painter.save();
    painter.setPen(QPen(drawColor, 3));
	painter.setBrush(drawColor);

	qreal rValue = (qreal)m_iValue / 100.0;
	qreal rRealHeight = rHeight * rValue;
	QRect realRect(BATTER_MARGIN + 2, BATTER_MARGIN + 2 + (rHeight - rRealHeight) - 1, width() - BATTER_MARGIN * 2 - 4, rRealHeight-1);
	painter.drawRect(realRect);

	painter.restore();



   // painter.drawRect(QRect(1, 1, this->width() - 3, this->height() - 3));
//     QFont font = painter.font();
//     if (m_iValue >= 100)
//     {
//         m_iValue = 100;
//         font.setPixelSize(16);
//     }
//     else
//     {
//         font.setPixelSize(16);
//     }
// 	font.setBold(true);
// 
//     painter.setFont(font);
   // painter.drawText(this->rect(), Qt::AlignCenter, QString::number(m_iValue) + "%");


}

void BatteryWgt::showEvent(QShowEvent *event)
{
	update();
}
