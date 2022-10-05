#ifndef DLSTATUSBAR_ALEXWEI_2018_0514_H
#define DLSTATUSBAR_ALEXWEI_2018_0514_H

#include <QStatusBar>


class QLabel;
class QTimer;
class QProgressBar;
class InputWidget;


class DLStatusBar : public QStatusBar
{
	Q_OBJECT

public:

	enum ChassisConnectState {
		ConnectError = -1,
		Connected = 0,
		disConnected = 1
	};

	DLStatusBar(QWidget *parent = 0);
	~DLStatusBar();
	void paintEvent(QPaintEvent *e);

public:
	void setConnectState(int state);
	void setOperate(const QString &opreation, int buildProgress = 0, bool isBuild = false);

private:
	void initStatusBar();
	void showConncetedInfo();

signals:
	//void signalShowDetailInfo(const robot_status_all1_req&);

public slots :
	void slot_set_statusbar_info();

private:
	QLabel * state_label_;
	QProgressBar *build_map_progressBar_;
	QLabel *runMode_label_;
	QLabel *position_label_;
	QLabel *speed_label_;
	QLabel *battary_label_;
	QLabel *temperature_label_;
	QProgressBar *battary_progressBar_;
	ChassisConnectState connectState_;

private:
	bool isBuild_;
	int buildProgress_;

	// Ä£Ê½Ñ¡Ôñ;
	InputWidget* m_modeChangeWidget;
};



#endif	//20170720