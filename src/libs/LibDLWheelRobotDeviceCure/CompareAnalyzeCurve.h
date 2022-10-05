#pragma once

#if _MSC_VER >= 1600  
#pragma execution_character_set("utf-8")  
#endif 

#include <QWidget>
#include "LibQCustomPlot/qcustomplot.h"
#include <QMouseEvent>
class CompareAnalyzeCurve  : public QWidget
{
	Q_OBJECT
public:
	CompareAnalyzeCurve(QWidget *parent = NULL);
	~CompareAnalyzeCurve();

	void setCompareDataVector(QVector<double> m_enviData, QVector<QString> x_value, QString pointName);
	void DataClear();
	void DrawPic(double value_min, double value_max, QVector<double> y_value, QVector<QString> x_value,QString pointName);

	private slots:
	void my_mouseMe(QMouseEvent * event);

private:
	void enterEvent(QEvent *event);
	void leaveEvent(QEvent *event);

	QCustomPlot * customPlot;
	int JudgeVirtual;
	QLabel* m_valueLabel;
};

