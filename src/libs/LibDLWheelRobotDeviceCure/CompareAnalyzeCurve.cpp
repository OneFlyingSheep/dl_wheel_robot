#include "CompareAnalyzeCurve.h"
#include <QList>
#include <QHBoxLayout>
#include <QLabel>

CompareAnalyzeCurve::CompareAnalyzeCurve(QWidget *parent)
	: QWidget(parent)
{
	customPlot = new QCustomPlot();

	m_valueLabel = new QLabel(customPlot);
	m_valueLabel->setFixedSize(QSize(50, 20));
	m_valueLabel->setStyleSheet("QLabel{color:green;}");
	m_valueLabel->setVisible(false);

 	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
 	hMainLayout->addWidget(customPlot);
 	hMainLayout->setMargin(0);

	connect(customPlot, SIGNAL(mouseMove(QMouseEvent*)), this, SLOT(my_mouseMe(QMouseEvent*)));
}


CompareAnalyzeCurve::~CompareAnalyzeCurve()
{
}

void CompareAnalyzeCurve::setCompareDataVector(QVector<double> m_enviData, QVector<QString> x_value, QString pointName)
{
	QVector<double> y_value;
	double value_max = 0.0;
	double value_min = 0.0;

	for (int i = 0; i < m_enviData.size(); i++)
	{
		if (i == 0)
		{
			value_max = m_enviData[i];
			value_min = m_enviData[i];
		}
		else
		{
			if (value_max < m_enviData[i])
				value_max = m_enviData[i];
			if (value_min > m_enviData[i])
				value_min = m_enviData[i];
		}
		y_value.append(m_enviData[i]);
	}
	DrawPic(value_min - 2, value_max + 2, y_value, x_value, pointName);
}
void CompareAnalyzeCurve::DataClear()
{
	int plottableCount = customPlot->plottableCount();
	customPlot->clearPlottables();
	customPlot->clearItems();
	customPlot->clearGraphs();
}
void CompareAnalyzeCurve::DrawPic(double value_min, double value_max, QVector<double> y_value, QVector<QString> x_value ,QString pointName)
{
	customPlot->replot();
	QVector<double> x_vi;
	QString m_enviKind = pointName;

	for (int i = 1; i <= x_value.size(); i++)
	{
		x_vi.append(i);
	}
	customPlot->legend->setVisible(true);
	customPlot->legend->setFont(QFont("Helvetica", 10));
	customPlot->legend->setRowSpacing(-8);

	QPen pen;
	pen.setColor("#ff0000");

	customPlot->addGraph();
	customPlot->graph(0)->setPen(pen);
	customPlot->graph(0)->setName(m_enviKind);
	customPlot->graph(0)->setLineStyle(QCPGraph::lsLine);
	customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 7));
	//	customPlot->graph(0)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 7));

	customPlot->graph(0)->setData(x_vi, y_value);
	customPlot->xAxis->setSubTicks(false);

	QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
	for (int i = 1; i <= x_value.size(); i++)
	{
		textTicker->addTick(i, QString("%1").arg(x_value[i - 1]).replace("_", "\n"));
	}
	int value1 = ((value_max - value_min) / 10) * 100;
	double d_value1 = value1 / 100.0;
	int value2 = value_min * 100;
	double d_value2 = value2 / 100.0;
	int value3 = value_max * 100;
	double d_value3 = value3 / 100.0;
	QSharedPointer<QCPAxisTickerText> textTickery(new QCPAxisTickerText);
	for (int i = 0; i < 12; i++)
	{
		textTickery->addTick(d_value2 + i * d_value1, QString("%1").arg(d_value2 + i * d_value1));
	}
	customPlot->yAxis->setTicker(textTickery);
	customPlot->xAxis->setTicker(textTicker);
	customPlot->yAxis->setRange(value_min - d_value1, value_max + d_value1);
	customPlot->xAxis->setRange(0.8, x_value.size() + 0.8);
	customPlot->replot();
}

void CompareAnalyzeCurve::enterEvent(QEvent *event)
{
	if (JudgeVirtual != 2)
	{
		m_valueLabel->setVisible(true);
	}
}

void CompareAnalyzeCurve::leaveEvent(QEvent *event)
{
	if (JudgeVirtual != 2)
	{
		m_valueLabel->setVisible(false);
	}
}

void CompareAnalyzeCurve::my_mouseMe(QMouseEvent *event)
{
	customPlot->setCursor(Qt::CrossCursor);
	if (JudgeVirtual != 2)
	{
		int x_pos = event->pos().x();
		double x = customPlot->xAxis->pixelToCoord(x_pos);
		int y_pos = event->pos().y();
		double y = customPlot->yAxis->pixelToCoord(y_pos);

		m_valueLabel->setText(QString::number(y));
		m_valueLabel->move(event->pos() + QPoint(10, 3));
	}
}

