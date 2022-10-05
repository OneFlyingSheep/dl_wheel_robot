#ifndef DLRELOCATEPROCESSER_ALEXWEI_2018_05_25_H
#define DLRELOCATEPROCESSER_ALEXWEI_2018_05_25_H

#include <QThread>
#include <QObject>
#include <QMutex>

class DLCustomScene;

class DLRelocateProcesser : public QThread
{
	Q_OBJECT

public:
	DLRelocateProcesser(DLCustomScene *scene, double x = 0, double y = 0, double angle = 0);
	~DLRelocateProcesser(){}

	void lockThread();
	void unlockThread();
	void stopThread();

protected:
	void run();

signals:
	void signal_relocation_finished(int executeResult);

private slots:
	void slot_confirm_relocation();

private:
	double pos_x_;
	double pos_y_;
	double pos_angle_;
	int old_execute_result_;
	QMutex runningMutex_;
	bool isStop_;

	DLCustomScene *scene_;
};



#endif