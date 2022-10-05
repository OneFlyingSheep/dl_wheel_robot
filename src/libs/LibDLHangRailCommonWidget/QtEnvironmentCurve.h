#pragma once
#include <QtWidgets/QMainWindow>
#include "common/DLHangRailRobotGlobalDef.hpp"
#include "LibQCustomPlot/qcustomplot.h"
#include <QLabel>
class QtEnvironmentCurve : public QWidget
{
	Q_OBJECT
public:
	QtEnvironmentCurve(QWidget *parent = Q_NULLPTR);
	~QtEnvironmentCurve();
	void getDBEnviDataList(int m_envi);
	void DataClear();
	void DrawPic(int m_envi, double value_min, double value_max, QVector<double> y_value, QVector<QString> x_value);

private slots:
	void my_mouseMe(QMouseEvent * event);

private:
	void enterEvent(QEvent *event);
	void leaveEvent(QEvent *event);

	QCustomPlot * customPlot;
	int JudgeVirtual;
	QLabel* m_valueLabel;
};

