#ifndef SMAPBUILDER_ALEXWEI_20180503_H
#define SMAPBUILDER_ALEXWEI_20180503_H

#include <QThread>
#include "startmap.h"

class QTimer;

//struct BuildSmapParam {
//	std::string filename_2d_;
//	std::string filename_smap_;
//	double xmin_;
//	double ymin_;
//	double xmax_;
//	double ymax_;
//	double delta_;
//	double sigma_;
//	double maxUrange_;
//	double maxrange_;
//	double regscore_;
//	double lstep_;
//	double astep_;
//	double kernelSize_;
//
//	double iterations_;
//	double critscore_;
//	double maxMove_;
//	double lsigma_;
//	double ogain_;
//	double lskip_;
//	bool autosize_;
//	bool skipMatching_;
//
//	double srr_;
//	double srt_;
//	double str_;
//	double stt_;
//
//	int particles_;
//	int randseed_;
//
//	double angularUpdate_;
//	double linearUpdate_;
//	double resampleThreshold_;
//
//	double llsamplerange_;
//	double llsamplestep_;
//	double lasamplerange_;
//	double lasamplestep_;
//	double mapscale_;
//};

class SmapBuilder : public QThread
{
	Q_OBJECT
		
public:

	SmapBuilder(QObject *parent = 0);
	~SmapBuilder(){}
	void set_map_param(const BuildSmapParam &param);
	void start_timer();
	void stop_timer();
	void set_file_name(const std::string &file_name);

protected:
	virtual void run();

private slots:
	void slot_get_process();

signals:
	void sig_finished();
	void sig_process(float process);

private:
	std::string file_name_;
	QTimer *timer_;
	BuildSmapParam param_;
};


#endif