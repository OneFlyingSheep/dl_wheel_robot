#pragma once

#include <QtWidgets/QMainWindow>
#include "LibQCustomPlot/qcustomplot.h"


struct CurveParameter
{
	double value_min_1;
	double value_min_2;
	double value_max_1;
	double value_max_2;
	int row_x_c;
	int device_type_id;
	double alarm_up;
	double alarm_down;
};
enum CurveEnumData
{
	ZeroCurveJudge = 0,
	FirstCurveJudge,
	SecondCurveJudge,
	ThirdCurveJudge,
	ForthCurveJudge,
	FifthCurveJudge,
	SixthCurveJudge,
	SeventhCurveJudge,
	EighthCurveJudge,
	NinthCurveJudge,
	TenthCurveJudge,
	EleventhCurveJudge,
};
class QtDeviceCurve : public QWidget
{
	Q_OBJECT

public:
	QtDeviceCurve(QWidget *parent = Q_NULLPTR);
	~QtDeviceCurve();
	void AnalysisDataModel(QString Device_ssid);
	void DataClear();
	void DrawPicEmpty();

private slots:
	void my_mouseMe(QMouseEvent * event);

private:
	void DrawPic(int n, CurveParameter m_curveStruct, QList<QString> cureName, QVector<double> x_Data, QVector<double> y_Data);
	void DrawPicVirtual(int n, CurveParameter m_curveStruct, QList<QString> cureName, QVector<double> x_Data, QVector<double> y_Data);
	
	void enterEvent(QEvent *event);
	void leaveEvent(QEvent *event);

private:
	QCustomPlot *customPlot;
	int JudgeVirtual;
	QLabel* m_valueLabel;
	QList<double> m_transmitValue;
};
