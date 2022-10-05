#include "QtDeviceCurve.h"
#include <QDebug>
#include <QString>
#include <QTableWidget>
#include <QStandardItemModel>
#include <QTableView>
#include <QList>
#include <LibDLHangRailRobotDBOperation/LibDLHangRailRobotDBOperation.h>
#include <QHBoxLayout>


QtDeviceCurve::QtDeviceCurve(QWidget *parent)
	: QWidget(parent)
{
	customPlot = new QCustomPlot();
	m_valueLabel = new QLabel(customPlot);
	m_valueLabel->setFixedSize(QSize(50, 40));
	m_valueLabel->setStyleSheet("QLabel{color:red;}");
	m_valueLabel->setVisible(false);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(customPlot);
	hMainLayout->setMargin(0);
	connect(customPlot, SIGNAL(mouseMove(QMouseEvent*)), this, SLOT(my_mouseMe(QMouseEvent*)));
}

QtDeviceCurve::~QtDeviceCurve()
{
	delete customPlot;
}

void QtDeviceCurve::AnalysisDataModel(QString Device_ssid)
{
	int row_x, row_y;
	CurveParameter m_curveStruct;

	QStandardItemModel * m_DeviceCurveModel = new QStandardItemModel();
	ROBOTDB_DB.getDeviceCurveData(row_x, row_y, Device_ssid, m_DeviceCurveModel);
	m_transmitValue.clear();
	if (row_x == 99)
	{
		DrawPicEmpty();
		return;
	}
	QVector<double> x_Data_c1;
	QVector<double> y_Data_c1;
	QVector<double> x_Data_c2;
	QVector<double> y_Data_c2;
	QVector<double> x_Data_c3;
	QVector<double> y_Data_c3;
	QVector<double> x_Data_c4;
	QVector<double> y_Data_c4;
	QVector<double> alarm_up;
	QVector<double> alarm_down;
	QVector<double> alarm_x;
	QList<double> y_maxmin;
	QList<double> y_maxmin_v;
	QList<double> compare;
	QList<QString> cureName;
	double value_avg_c1;
	double value_avg_c2;
	double value_avg_c3;
	double value_avg_c4;
	double value_max = 0.0;
	double value_min = 0.0;
	/*int JudgeVirtual = 3;*/
	cureName.clear();

	QString str = m_DeviceCurveModel->index(1, 0).data().toString();
	QStringList list1 = str.split(" ");
	
	if (list1.size() < 3)
		return;
	m_curveStruct.device_type_id = list1[0].toInt();
	m_curveStruct.alarm_up = list1[1].toDouble();
	m_curveStruct.alarm_down = list1[2].toDouble();

	value_max = list1[1].toDouble();
	value_min = list1[2].toDouble();

	JudgeVirtual = m_DeviceCurveModel->index(0, 0).data().toInt();
	if (row_y == SecondCurveJudge)
	{
		cureName.append(m_DeviceCurveModel->index(0, 1).data().toString());
		x_Data_c1.clear();
		y_Data_c1.clear();
		for (int j = 1; j < row_x; j++)
		{
			if ((m_DeviceCurveModel->index(j, 1).data().toString()).isEmpty()){}
			else
			{
				value_avg_c1 = m_DeviceCurveModel->index(j, 1).data().toDouble();
				if (j == 1)
				{
					value_min = value_avg_c1;
				}
				if (value_avg_c1 > value_max)
				{
					value_max = value_avg_c1;
				}
				if (value_avg_c1 < value_min)
				{
					value_min = value_avg_c1;
				}
				x_Data_c1.append(j);
				y_Data_c1.prepend(value_avg_c1);
			}
			alarm_up.append(list1[1].toDouble());
			alarm_down.append(list1[2].toDouble());
			alarm_x.append(j);
		}
		DataClear();
		customPlot->replot();

		m_curveStruct.value_min_1 = value_min;
		m_curveStruct.value_max_1 = value_max;
		m_curveStruct.value_min_2 = 0;
		m_curveStruct.value_max_2 = 0;
		m_curveStruct.row_x_c = row_x;

		if(JudgeVirtual==2)
			DrawPicVirtual(ZeroCurveJudge, m_curveStruct, cureName, x_Data_c1, y_Data_c1);
		else
		{
			cureName.append("告警上限");
			cureName.append("告警下限");
			DrawPic(ZeroCurveJudge, m_curveStruct, cureName, x_Data_c1, y_Data_c1);
			DrawPic(ForthCurveJudge, m_curveStruct, cureName, x_Data_c1, alarm_up);
			DrawPic(FifthCurveJudge, m_curveStruct, cureName, alarm_x, alarm_down);
		}
	}
	if (row_y == ThirdCurveJudge)
	{
		cureName.append(m_DeviceCurveModel->index(0, 1).data().toString());
		cureName.append(m_DeviceCurveModel->index(0, 2).data().toString());
		cureName.append("虚拟设备");
		for (int j = 1; j < row_x; j++)
		{
			value_avg_c1 = m_DeviceCurveModel->index(j, 1).data().toDouble();
			value_avg_c2 = m_DeviceCurveModel->index(j, 2).data().toDouble();
			if (value_avg_c1 >= value_avg_c2)
			{
				value_avg_c3 = value_avg_c1 - value_avg_c2;
			}
			if (value_avg_c1 < value_avg_c2)
			{
				value_avg_c3 = value_avg_c2 - value_avg_c1;
			}
			
			if ((m_DeviceCurveModel->index(j, 1).data().toString()).isEmpty())
			{
				if ((m_DeviceCurveModel->index(j, 2).data().toString()).isEmpty()){}
				else
				{
					y_maxmin.append(value_avg_c2);
					x_Data_c2.append(j);
					y_Data_c2.append(value_avg_c2);
				}
			}
			else
			{
				y_maxmin.append(value_avg_c1);
				x_Data_c1.append(j);
				y_Data_c1.append(value_avg_c1);
				if ((m_DeviceCurveModel->index(j, 2).data().toString()).isEmpty()){}
				else
				{
					y_maxmin.append(value_avg_c1);
					y_maxmin.append(value_avg_c2);
					y_maxmin_v.append(value_avg_c3);
					x_Data_c2.append(j);
					y_Data_c2.append(value_avg_c2);
					x_Data_c3.append(j);
					y_Data_c3.append(value_avg_c3);
				}
			}
			alarm_up.append(list1[1].toDouble());
			alarm_down.append(list1[2].toDouble());
			alarm_x.append(j);
		}
		qSort(y_maxmin.begin(), y_maxmin.end());
		value_min = y_maxmin.first();
		value_max = y_maxmin.last();
		
		m_curveStruct.value_min_1 = value_min;
		m_curveStruct.value_max_1 = value_max;
		if (y_maxmin_v.size() == 0)
		{
			m_curveStruct.value_min_2 = 0.0;
			m_curveStruct.value_max_2 = 0.0;
		}
		else
		{
			qSort(y_maxmin_v.begin(), y_maxmin_v.end());
			m_curveStruct.value_min_2 = y_maxmin_v.first();
			m_curveStruct.value_max_2 = y_maxmin_v.last();
		}
		m_curveStruct.row_x_c = row_x;

		DataClear();
		customPlot->replot();
		if (JudgeVirtual == SecondCurveJudge)
		{
			DrawPicVirtual(ZeroCurveJudge, m_curveStruct, cureName, x_Data_c1, y_Data_c1);
			DrawPicVirtual(FirstCurveJudge, m_curveStruct, cureName, x_Data_c2, y_Data_c2);
			DrawPicVirtual(EighthCurveJudge, m_curveStruct, cureName, x_Data_c3, y_Data_c3);
		}
		else
		{
			cureName.append("告警上限");
			cureName.append("告警下限");
			DrawPic(ZeroCurveJudge, m_curveStruct, cureName, x_Data_c1, y_Data_c1);
			DrawPic(FirstCurveJudge, m_curveStruct, cureName, x_Data_c2, y_Data_c2);
			DrawPic(EighthCurveJudge, m_curveStruct, cureName, x_Data_c3, y_Data_c3);
			DrawPic(SixthCurveJudge, m_curveStruct, cureName, alarm_x, alarm_up);
			DrawPic(SeventhCurveJudge, m_curveStruct, cureName, alarm_x, alarm_down);
		}
	}
	if (row_y == ForthCurveJudge)
	{
		cureName.append(m_DeviceCurveModel->index(0, 1).data().toString());
		cureName.append(m_DeviceCurveModel->index(0, 2).data().toString());
		cureName.append(m_DeviceCurveModel->index(0, 3).data().toString());
		cureName.append("虚拟设备");
		for (int j = 1; j < row_x; j++)
		{
			value_avg_c1 = m_DeviceCurveModel->index(j, 1).data().toDouble();
			value_avg_c2 = m_DeviceCurveModel->index(j, 2).data().toDouble();
			value_avg_c3 = m_DeviceCurveModel->index(j, 3).data().toDouble();
			compare.append(value_avg_c1);
			compare.append(value_avg_c2);
			compare.append(value_avg_c3);
			qSort(compare.begin(), compare.end());
			value_avg_c4 = compare.last() - compare.first();
			compare.clear();
			if ((m_DeviceCurveModel->index(j, 1).data().toString()).isEmpty())
			{
				if ((m_DeviceCurveModel->index(j, 2).data().toString()).isEmpty())
				{
					if ((m_DeviceCurveModel->index(j, 3).data().toString()).isEmpty()){}
					else
					{
						y_maxmin.append(value_avg_c3);
						x_Data_c3.append(j);
						y_Data_c3.append(value_avg_c3);
					}
				}
				else
				{
					y_maxmin.append(value_avg_c2);
					x_Data_c2.append(j);
					y_Data_c2.append(value_avg_c2);
					if ((m_DeviceCurveModel->index(j, 3).data().toString()).isEmpty())
					{
					}
					else
					{
						y_maxmin.append(value_avg_c3);
						x_Data_c3.append(j);
						y_Data_c3.append(value_avg_c3);
					}
				}
			}
			else
			{
				y_maxmin.append(value_avg_c1);
				x_Data_c1.append(j);
				y_Data_c1.append(value_avg_c1);
				if ((m_DeviceCurveModel->index(j, 2).data().toString()).isEmpty())
				{
					if ((m_DeviceCurveModel->index(j, 3).data().toString()).isEmpty()){}
					else
					{
						y_maxmin.append(value_avg_c3);
						x_Data_c3.append(j);
						y_Data_c3.append(value_avg_c3);
					}
				}
				else
				{
					y_maxmin.append(value_avg_c2);
					x_Data_c2.append(j);
					y_Data_c2.append(value_avg_c2);
					if ((m_DeviceCurveModel->index(j, 3).data().toString()).isEmpty())
					{
					}
					else
					{
						y_maxmin.append(value_avg_c3);
						y_maxmin_v.append(value_avg_c4);
						x_Data_c3.append(j);
						y_Data_c3.append(value_avg_c3);
						x_Data_c4.append(j);
						y_Data_c4.append(value_avg_c4);
					}
				}
			}
			alarm_up.append(list1[1].toDouble());
			alarm_down.append(list1[2].toDouble());
			alarm_x.append(j);
		}
		qSort(y_maxmin.begin(), y_maxmin.end());
		value_min = y_maxmin.first();
		value_max = y_maxmin.last();
		DataClear();
		customPlot->replot();

		if (y_maxmin_v.size() == 0)
		{
			m_curveStruct.value_min_2 = 0.0;
			m_curveStruct.value_max_2 = 0.0;
		}
		else
		{
			qSort(y_maxmin_v.begin(), y_maxmin_v.end());
			m_curveStruct.value_min_2 = y_maxmin_v.first();
			m_curveStruct.value_max_2 = y_maxmin_v.last();
		}
		m_curveStruct.value_min_1 = value_min;
		m_curveStruct.value_max_1 = value_max;
		m_curveStruct.row_x_c = row_x;

		if (JudgeVirtual == 2)
		{
			DrawPicVirtual(ZeroCurveJudge, m_curveStruct, cureName, x_Data_c1, y_Data_c1);
			DrawPicVirtual(FirstCurveJudge, m_curveStruct, cureName, x_Data_c2, y_Data_c2);
			DrawPicVirtual(SecondCurveJudge, m_curveStruct, cureName, x_Data_c3, y_Data_c3);
			DrawPicVirtual(NinthCurveJudge, m_curveStruct, cureName, x_Data_c4, y_Data_c4);
		}
		else
		{
			cureName.append("告警上限");
			cureName.append("告警下限");
			DrawPic(ZeroCurveJudge, m_curveStruct, cureName, x_Data_c1, y_Data_c1);
			DrawPic(FirstCurveJudge, m_curveStruct, cureName, x_Data_c2, y_Data_c2);
			DrawPic(SecondCurveJudge, m_curveStruct, cureName, x_Data_c3, y_Data_c3);
			DrawPic(NinthCurveJudge, m_curveStruct, cureName, x_Data_c4, y_Data_c4);
			DrawPic(TenthCurveJudge, m_curveStruct, cureName, alarm_x, alarm_up);
			DrawPic(EleventhCurveJudge, m_curveStruct, cureName, alarm_x, alarm_down);
		}
	}
}
void QtDeviceCurve::DataClear()
{
	int plottableCount = customPlot->plottableCount();
	customPlot->clearPlottables();
	customPlot->clearItems();
	customPlot->clearGraphs();
}
void QtDeviceCurve::DrawPic(int n, CurveParameter m_curveStruct, QList<QString> cureName, QVector<double> x_Data, QVector<double> y_Data)
{
	double value_max = m_curveStruct.value_max_1;
	double value_min = m_curveStruct.value_min_1;
	double value_max2 = m_curveStruct.value_max_2;
	double value_min2 = m_curveStruct.value_min_2;
	double row_x = m_curveStruct.row_x_c;

	int value1 = ((value_max - value_min) / 10.0) * 100;
	double d_value1 = value1 / 100.0;
	int value2 = value_min * 100;
	double d_value2 = value2 / 100.0;
	int value3 = value_max * 100;
	double d_value3 = value3 / 100.0;

	int value1_2 = ((value_max2 - value_min2) / 10.0) * 100;
	double d_value1_2 = value1_2 / 100.0;
	int value2_2 = value_min2 * 100;
	double d_value2_2 = value2_2 / 100.0;
	int value3_2 = value_max2 * 100;
	double d_value3_2 = value3_2 / 100.0;

	customPlot->replot();
	int m = ZeroCurveJudge;
	if (n == EighthCurveJudge)
	{
		m = SecondCurveJudge;
		n = SecondCurveJudge;
		m_transmitValue.append(value_min);
		m_transmitValue.append(value_min2);
		m_transmitValue.append(d_value1);
		m_transmitValue.append(d_value1_2);
		m_transmitValue.append(ThirdCurveJudge);
	}
	if (n == NinthCurveJudge)
	{
		m = ThirdCurveJudge;
		n = ThirdCurveJudge;
		m_transmitValue.append(value_min);
		m_transmitValue.append(value_min2);
		m_transmitValue.append(d_value1);
		m_transmitValue.append(d_value1_2);
		m_transmitValue.append(ThirdCurveJudge);
	}

	customPlot->legend->setVisible(true);
	customPlot->legend->setFont(QFont("Helvetica", EighthCurveJudge));
	customPlot->legend->setRowSpacing(0);

	QPen pen;

	if (n == ZeroCurveJudge) pen.setColor("#9400d3");
	if (n == FirstCurveJudge) pen.setColor("#0000ff");
	if (n == SecondCurveJudge) pen.setColor("#006400");
	if (m == SecondCurveJudge || m == ThirdCurveJudge) pen.setColor("#ff0000");
	if (n == ForthCurveJudge) { pen.setColor("#800000"); n = FirstCurveJudge; m = ForthCurveJudge; }
	if (n == FifthCurveJudge) { pen.setColor("#000000"); n = SecondCurveJudge; m = ForthCurveJudge; }
	if (n == SixthCurveJudge) { pen.setColor("#800000"); n = ThirdCurveJudge; m = ForthCurveJudge; }
	if (n == SeventhCurveJudge) { pen.setColor("#000000"); n = ForthCurveJudge; m = ForthCurveJudge; }
	if (n == TenthCurveJudge) { pen.setColor("#800000"); n = ForthCurveJudge; m = ForthCurveJudge; }
	if (n == EleventhCurveJudge) { pen.setColor("#000000"); n = FifthCurveJudge; m = ForthCurveJudge; }

	customPlot->addGraph();
	customPlot->graph(n)->setPen(pen);
	customPlot->graph(n)->setName(cureName[n]);
	customPlot->graph(n)->setLineStyle(QCPGraph::lsLine);
	if (m == ForthCurveJudge) {}
	else
	{
		if (m == SecondCurveJudge || m == ThirdCurveJudge)
		{
			customPlot->graph(n)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, SeventhCurveJudge));
		}
		else
		{
			customPlot->graph(n)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, SeventhCurveJudge));
		}
	}
	if (m == ThirdCurveJudge)
	{
		for (int i = 0; i < y_Data.size(); i++)
		{
			y_Data[i] = (y_Data[i] - value_min2) / d_value1_2 * d_value1 + value_min;
		}
	}

	customPlot->graph(n)->setData(x_Data, y_Data);
	customPlot->xAxis->setSubTicks(false);
	customPlot->yAxis2->setVisible(true);

	QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
	for (int i = 1; i < row_x; i++)
	{
		textTicker->addTick(i, QString("检测%1").arg(i));
	}

	QSharedPointer<QCPAxisTickerText> textTickery(new QCPAxisTickerText);
	QSharedPointer<QCPAxisTickerText> textTickery2(new QCPAxisTickerText);
	for (int i = 0; i < 12; i++)
	{
		textTickery->addTick(d_value2 +i* d_value1, QString("%1").arg(d_value2 + i * d_value1));
		textTickery2->addTick(d_value2_2 + i * d_value1_2, QString("%1").arg(d_value2_2 + i * d_value1_2));
	}

	customPlot->yAxis->setTicker(textTickery);
	customPlot->xAxis->setTicker(textTicker);
	customPlot->yAxis2->setTicker(textTickery2);
	
	customPlot->yAxis2->setBasePen(QPen(Qt::red, 1));
	customPlot->yAxis2->setTickPen(QPen(Qt::red, 1));
	customPlot->yAxis2->setSubTickPen(QPen(Qt::red, 1));
	customPlot->yAxis2->setTickLabelColor(Qt::red);

	customPlot->yAxis->setRange(value_min - d_value1, value_max + d_value1);
	customPlot->yAxis2->setRange(value_min2 - d_value1_2, value_max2 + d_value1_2);
	customPlot->xAxis->setRange(0.8, row_x);
	customPlot->replot();
}

void QtDeviceCurve::DrawPicVirtual(int n, CurveParameter m_curveStruct, QList<QString> cureName, QVector<double> x_Data, QVector<double> y_Data)
{
	int row_x = m_curveStruct.row_x_c;
	customPlot->replot();
	int m = 0;
	if (n == 8)
	{
		m = 2;
		n = 2;
	}
	if (n == 9)
	{
		m = 3;
		n = 3;
	}
	customPlot->legend->setVisible(true);
	customPlot->legend->setFont(QFont("Helvetica", 8));
	customPlot->legend->setRowSpacing(-8);

	QPen pen;
	if (n == 0)
		pen.setColor("#9400d3");

	if (n == 1)
		pen.setColor("#0000ff");

	if (n == 2)
		pen.setColor("#006400");

	if (n == 3)
		pen.setColor("#ff0000");

	customPlot->addGraph();
	customPlot->graph(n)->setPen(pen);
	customPlot->graph(n)->setName(cureName[n]);
	customPlot->graph(n)->setLineStyle(QCPGraph::lsLine);
	if (m == 2 || m == 3)
	{
		customPlot->graph(n)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, 7));
	}
	else
	{
		customPlot->graph(n)->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 7));
	}
	customPlot->graph(n)->setData(x_Data, y_Data);
	customPlot->xAxis->setSubTicks(false);
	customPlot->yAxis2->setVisible(false);

	QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
	for (int i = 1; i < row_x; i++)
	{
		textTicker->addTick(i, QString("检测%1").arg(i));
	}

	QSharedPointer<QCPAxisTickerText> textTickery(new QCPAxisTickerText);
	textTickery->addTick(0, "异常");
	textTickery->addTick(1, "正常");
	customPlot->yAxis->setTicker(textTickery);
	customPlot->xAxis->setTicker(textTicker);

	customPlot->yAxis->setRange(-1.0, 2.0);
	customPlot->yAxis2->setRange(-1.0, 2.0);
	customPlot->xAxis->setRange(0.8, row_x);
	customPlot->replot();
}

void QtDeviceCurve::DrawPicEmpty()
{
	DataClear();
	customPlot->replot();
	customPlot->addGraph();
	customPlot->graph(0);
	customPlot->graph(0)->setLineStyle(QCPGraph::lsLine);
	
//	customPlot->graph(n)->setData(x_Data, y_Data);
	customPlot->xAxis->setSubTicks(false);
	customPlot->yAxis2->setVisible(false);

	QSharedPointer<QCPAxisTickerText> textTicker(new QCPAxisTickerText);
	for (int i = 1; i < 11; i++)
	{
		textTicker->addTick(i, QString("检测%1").arg(i));
	}

	QSharedPointer<QCPAxisTickerText> textTickery(new QCPAxisTickerText);
	textTickery->addTick(0, "0");
	textTickery->addTick(1, "1");
	textTickery->addTick(2, "2");
	textTickery->addTick(3, "3");
	textTickery->addTick(4, "4");
	textTickery->addTick(5, "5");
	customPlot->yAxis->setTicker(textTickery);
	customPlot->xAxis->setTicker(textTicker);

	customPlot->yAxis->setRange(0, 5.0);
	customPlot->yAxis2->setRange(0, 5.0);
	customPlot->xAxis->setRange(0.8, 10.2);
	customPlot->replot();
}

void QtDeviceCurve::my_mouseMe(QMouseEvent *event)
{
	customPlot->setCursor(Qt::CrossCursor);
	if (JudgeVirtual != 2)
	{
		int x_pos = event->pos().x();
		double x = customPlot->xAxis->pixelToCoord(x_pos);
		int y_pos = event->pos().y();
		double y = customPlot->yAxis->pixelToCoord(y_pos);
		int p = 0;
		double v = 0.0;
		if (m_transmitValue.size())
		{
			p = m_transmitValue[4] + 0.0001;
			v = (y - m_transmitValue[0]) / m_transmitValue[2] * m_transmitValue[3] + m_transmitValue[1];
		}
		if(p==3)
			m_valueLabel->setText(QString("%1\n%2").arg(y).arg(v));
		else
			m_valueLabel->setText(QString("%1").arg(y));
		m_valueLabel->move(event->pos() + QPoint(10, 3));
	}
}

void QtDeviceCurve::enterEvent(QEvent *event)
{
	if (JudgeVirtual != 2)
	{
		m_valueLabel->setVisible(true);
	}
}

void QtDeviceCurve::leaveEvent(QEvent *event)
{
	if (JudgeVirtual != 2)
	{
		m_valueLabel->setVisible(false);
	}
}