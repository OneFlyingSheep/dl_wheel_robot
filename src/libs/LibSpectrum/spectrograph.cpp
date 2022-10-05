/****************************************************************************
**
** Copyright (C) 2015 The Qt Company Ltd.
** Contact: http://www.qt.io/licensing/
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "spectrograph.h"
#include <QDebug>
#include <QMouseEvent>
#include <QPainter>
#include <QTimerEvent>
#include < QDebug>
#include <QtMath>

const int NullTimerId = -1;
const int NullIndex = -1;
const int BarSelectionInterval = 2000;

Spectrograph::Spectrograph(QWidget *parent)
    :   QWidget(parent)
    ,   m_barSelected(NullIndex)
    ,   m_timerId(NullTimerId)
    ,   m_lowFreq(0.0)
    ,   m_highFreq(0.0)
    ,m_rAmplitudeMax(0.0)
    ,m_rAmplitudeMin(0.0)
{
    setMinimumHeight(100);
}

Spectrograph::~Spectrograph()
{

}

void Spectrograph::setParams(int numBars, qreal lowFreq, qreal highFreq)
{
    Q_ASSERT(numBars > 0);
    Q_ASSERT(highFreq > lowFreq);
    m_bars.resize(numBars);
    m_lowFreq = lowFreq;
    m_highFreq = highFreq;
    updateBars();
}

void Spectrograph::timerEvent(QTimerEvent *event)
{
    Q_ASSERT(event->timerId() == m_timerId);
    Q_UNUSED(event) // suppress warnings in release builds
    killTimer(m_timerId);
    m_timerId = NullTimerId;
    m_barSelected = NullIndex;
    update();
}

void Spectrograph::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event)

    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    const int numBars = m_bars.count();

    // Highlight region of selected bar
    if (m_barSelected != NullIndex && numBars) {
        QRect regionRect = rect();
        regionRect.setLeft(m_barSelected * rect().width() / numBars);
        regionRect.setWidth(rect().width() / numBars);
        QColor regionColor(202, 202, 64);
        painter.setBrush(Qt::DiagCrossPattern);
        painter.fillRect(regionRect, regionColor);
        painter.setBrush(Qt::NoBrush);
    }

    QColor barColor(51, 204, 102);
    QColor clipColor(255, 255, 0);

    // Draw the outline
    const QColor gridColor = barColor.darker();
    QPen gridPen(gridColor);
    painter.setPen(gridPen);
    painter.drawLine(rect().topLeft(), rect().topRight());
    painter.drawLine(rect().topRight(), rect().bottomRight());
    painter.drawLine(rect().bottomRight(), rect().bottomLeft());
    painter.drawLine(rect().bottomLeft(), rect().topLeft());

    QVector<qreal> dashes;
    dashes << 2 << 2;
    gridPen.setDashPattern(dashes);
    painter.setPen(gridPen);

    // Draw vertical lines between bars
//    if (numBars) {
//        const int numHorizontalSections = numBars;
//        QLine line(rect().topLeft(), rect().bottomLeft());
//        for (int i=1; i<numHorizontalSections; ++i) {
//            line.translate(rect().width()/numHorizontalSections, 0);
//            painter.drawLine(line);
//        }
//    }

//     Draw horizontal lines
    const int numVerticalSections = 10;
	//QLine line(rect().topLeft(), rect().topRight());
	QLine line(rect().topLeft(), QPoint(rect().topLeft().x() + 4, rect().topLeft().y()));
    for (int i=1; i<numVerticalSections; ++i) {
        line.translate(0, rect().height()/numVerticalSections);
        painter.drawLine(line);
    }

    barColor = barColor.lighter();
    barColor.setAlphaF(0.75);
    clipColor.setAlphaF(0.75);

    // Draw the bars
    if (numBars) {
        // Calculate width of bars and gaps
        const int widgetWidth = rect().width();
        const int barPlusGapWidth = widgetWidth / numBars;  //一个bar的宽度和间隙的总宽度
        const int barWidth = 0.8 * barPlusGapWidth;		//bar的宽度
        const int gapWidth = barPlusGapWidth - barWidth;		//左右两边间隙的宽度
        const int paddingWidth = widgetWidth - numBars * (barWidth + gapWidth);  //画板边的宽度
        const int leftPaddingWidth = (paddingWidth + gapWidth) / 2;  //左边开始位置
        const int barHeight = rect().height() - 2 * gapWidth;		//画板绘制的高度

        qreal rCell = 1.0;
        if(m_rAmplitudeMax != m_rAmplitudeMin)
        {

            rCell = barHeight / qMax(qAbs(m_rAmplitudeMax), qAbs(m_rAmplitudeMin)) / 2;
        }

		painter.save();
		QPen pen;
		pen.setColor(Qt::yellow);
		painter.setPen(pen);
		QLineF line(QPointF(leftPaddingWidth, rect().height() - gapWidth - 10), QPointF(leftPaddingWidth + numBars*(gapWidth + barWidth), rect().height() - gapWidth - 10));
		painter.drawLine(line);

		painter.drawText(QPointF(leftPaddingWidth + numBars * (gapWidth + barWidth) - 20, rect().height() - gapWidth - 20), "(HZ)");

		painter.restore();

        for (int i=0; i<numBars; ++i) 
		{
            const qreal value = m_bars[i].value;
           // Q_ASSERT(value >= 0.0 && value <= 1.0);
            QRect bar = rect();
            bar.setLeft(rect().left() + leftPaddingWidth + (i * (gapWidth + barWidth)));
            bar.setWidth(barWidth);
            if(value >= 0)
            {
                bar.setTop(rect().top() + barHeight/2 - rCell * value);
                bar.setBottom(rect().top() + barHeight/2);
            }
            else
            {
                bar.setTop(rect().top() + barHeight/2);
                bar.setBottom(rect().top() + barHeight/2 - rCell * value);
            }
           // bar.setTop(rect().top() + gapWidth*2 + (1.0 - value) * barHeight/2);
            //bar.setBottom(rect().bottom() - gapWidth - (1.0 - value) * barHeight/2);

            QColor color = barColor;
            if (m_bars[i].clipped)
                color = clipColor;

            painter.fillRect(bar, color);


			//绘制横坐标
			painter.save();
			QPen pen;
			pen.setColor(Qt::magenta);
			painter.setPen(pen);
			qreal rX = rect().left() + leftPaddingWidth + (i * (gapWidth + barWidth));
			QLineF line(QPointF(rX, rect().height() - gapWidth - 10), QPointF(rX, rect().height() - gapWidth - 20));
			painter.drawLine(line);

			const QPair<qreal, qreal> frequencyRange = barRange(i);
			qreal rVal = frequencyRange.first / 1000;
			painter.drawText(QPointF(rX-3, rect().height() - gapWidth+2), QString::number(rVal)+"K");

			if (i == numBars - 1)
			{
				rX = rect().left() + leftPaddingWidth + (i * (gapWidth + barWidth)) + (gapWidth + barWidth);
				line = QLineF(QPointF(rX, rect().height() - gapWidth - 10), QPointF(rX, rect().height() - gapWidth - 15));
				painter.drawLine(line);
				rVal = frequencyRange.second / 1000;
				painter.drawText(QPointF(rX - 3, rect().height() - gapWidth + 2), QString::number(rVal) + "K");
			}

			painter.restore();
        }



		//绘制纵向坐标轴
		if(m_rAmplitudeMax != m_rAmplitudeMin)
        {
            QLineF line1(QPointF(5, rect().top() + barHeight/2 + gapWidth), QPointF(5, rect().top()));
            QLineF line2(QPointF(5, rect().top() + barHeight/2 + gapWidth), QPointF(5, rect().bottom()));

			painter.save();
            QPen pen;
            pen.setWidth(2);
            pen.setColor(Qt::magenta);
            painter.setPen(pen);
            painter.drawLine(line1);
            painter.drawLine(line2);


            qreal rValue = qMax(qAbs(m_rAmplitudeMax), qAbs(m_rAmplitudeMin));
            painter.drawText(QPointF(8, rect().top() + barHeight/2 + gapWidth), "0");
            painter.drawText(QPointF(8, rect().top() + gapWidth + 10), QString::number((int)rValue)+"(db)");
            painter.drawText(QPointF(8, rect().bottom() - gapWidth), QString::number((int)-rValue));


            const int numVerticalSections = 10;
            QLine line(rect().topLeft(), rect().topRight());
            for (int i=1; i<numVerticalSections; ++i) {
                line.translate(0, rect().height()/numVerticalSections);
                if(i == 5) continue;
                QPointF point = line.p1();
                point.setX(point.x() + 8);
                point.setY(point.y() + 3);
                qreal rCurVal = ((rect().top() + barHeight/2) - line.p1().y()) / rCell;
                painter.drawText(point, QString::number((int)rCurVal));
            }

            painter.restore();
        }
    }
}

void Spectrograph::mousePressEvent(QMouseEvent *event)
{
    const QPoint pos = event->pos();
    const int index = m_bars.count() * (pos.x() - rect().left()) / rect().width();
    selectBar(index);
}

void Spectrograph::reset()
{
    m_spectrum.reset();
    spectrumChanged(m_spectrum);
}

void Spectrograph::spectrumChanged(const FrequencySpectrum &spectrum)
{
    //qDebug() << "Spectrograph spectrum:" << spectrum.count();
    m_spectrum = spectrum;
    updateBars();
}

int Spectrograph::barIndex(qreal frequency) const
{
    Q_ASSERT(frequency >= m_lowFreq && frequency < m_highFreq);
    const qreal bandWidth = (m_highFreq - m_lowFreq) / m_bars.count();
    const int index = (frequency - m_lowFreq) / bandWidth;
    if (index <0 || index >= m_bars.count())
        Q_ASSERT(false);
    return index;
}

QPair<qreal, qreal> Spectrograph::barRange(int index) const
{
    Q_ASSERT(index >= 0 && index < m_bars.count());
    const qreal bandWidth = (m_highFreq - m_lowFreq) / m_bars.count();
    return QPair<qreal, qreal>(index * bandWidth, (index+1) * bandWidth);
}

void Spectrograph::updateBars()
{
    m_bars.fill(Bar());
    FrequencySpectrum::const_iterator i = m_spectrum.begin();
    const FrequencySpectrum::const_iterator end = m_spectrum.end();
    for ( ; i != end; ++i) {
        const FrequencySpectrum::Element e = *i;
        if (e.frequency >= m_lowFreq && e.frequency < m_highFreq) {
            Bar &bar = m_bars[barIndex(e.frequency)];
            bar.value = e.amplitude;//qMax(bar.value, e.amplitude);
            if(i == m_spectrum.begin())
            {
                m_rAmplitudeMax = bar.value;
                m_rAmplitudeMin = bar.value;
            }
            else
            {
                if(m_rAmplitudeMax < bar.value)
                {
                    m_rAmplitudeMax = bar.value;
                }
                if(m_rAmplitudeMin > bar.value)
                {
                    m_rAmplitudeMin = bar.value;
                }
            }
            bar.clipped |= e.clipped;
        }
    }

    update();
}

void Spectrograph::selectBar(int index) {
    const QPair<qreal, qreal> frequencyRange = barRange(index);
    const QString message = QString("%1 - %2 Hz")
                                .arg(frequencyRange.first)
                                .arg(frequencyRange.second);
    emit infoMessage(message, BarSelectionInterval);

    if (NullTimerId != m_timerId)
        killTimer(m_timerId);
    m_timerId = startTimer(BarSelectionInterval);

    m_barSelected = index;
    update();
}


