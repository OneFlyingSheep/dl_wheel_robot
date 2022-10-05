#ifndef LASERPARAMWIDGET_ALEXWEI_H
#define LASERPARAMWIDGET_ALEXWEI_H

#include <QWidget>
#include "LibSmapBuilder/SmapBuilder.h"



class QLineEdit;
class QLabel;
class QCheckBox;
class QPushButton;
class QGroupBox;

class LaserParamWidget : public QWidget
{
	Q_OBJECT
public:
	LaserParamWidget(QWidget *parent = 0);
	~LaserParamWidget();
	
	void initWidget();
	BuildSmapParam getLaserParam();
	void initLaserParam();

signals:
	void sig_param_ok();

private slots:
	void slot_on_set_param();
	void slot_on_show_more_param(bool state);
	void slot_on_open_input_file();
	void slot_on_open_output_file();

private:
	QGroupBox * more_info_groupBox_;

	QLabel * input_fileName_label_;
	QLabel * output_fileName_label_;
	QLabel * xmin_label_;
	QLabel * ymin_label_;
	QLabel * xmax_label_;
	QLabel * ymax_label_;
	QLabel * delta_label_;
	QLabel * sigma_label_;
	QLabel * maxUrange_label_;
	QLabel * maxrange_label_;
	QLabel * regscore_label_;
	QLabel * lstep_label_;
	QLabel * astep_label_;
	QLabel * kernelSize_label_;
	QLabel * iterations_label_;
	QLabel * critscore_label_;
	QLabel * maxMove_label_;
	QLabel * lsigma_label_;
	QLabel * ogain_label_;
	QLabel * lskip_label_;
	QLabel * autosize_label_;
	QLabel * skipMatching_label_;
	QLabel * srr_label_;
	QLabel * srt_label_;
	QLabel * str_label_;
	QLabel * stt_label_;
	QLabel * particles_label_;
	QLabel * randseed_label_;
	QLabel * angularUpdate_label_;
	QLabel * linearUpdate_label_;
	QLabel * resampleThreshold_label_;
	QLabel * llsamplerange_label_;
	QLabel * llsamplestep_label_;
	QLabel * lasamplerange_label_;
	QLabel * lasamplestep_label_;
	QLabel * mapscale_label_;


	QLineEdit * input_fileName_lineEdit_;
	QLineEdit * output_fileName_lineEdit_;
	QLineEdit * xmin_lineEdit_;
	QLineEdit * ymin_lineEdit_;
	QLineEdit * xmax_lineEdit_;
	QLineEdit * ymax_lineEdit_;
	QLineEdit * delta_lineEdit_;
	QLineEdit * sigma_lineEdit_;
	QLineEdit * maxUrange_lineEdit_;
	QLineEdit * maxrange_lineEdit_;
	QLineEdit * regscore_lineEdit_;
	QLineEdit * lstep_lineEdit_;
	QLineEdit * astep_lineEdit_;
	QLineEdit * kernelSize_lineEdit_;
	QLineEdit * iterations_lineEdit_;
	QLineEdit * critscore_lineEdit_;
	QLineEdit * maxMove_lineEdit_;
	QLineEdit * lsigma_lineEdit_;
	QLineEdit * ogain_lineEdit_;
	QLineEdit * lskip_lineEdit_;
	QCheckBox * autosize_chechBox_;
	QCheckBox * skipMatching_checkBox_;
	QLineEdit * srr_lineEdit_;
	QLineEdit * srt_lineEdit_;
	QLineEdit * str_lineEdit_;
	QLineEdit * stt_lineEdit_;

	QLineEdit * particles_lineEdit_;
	QLineEdit * randseed_lineEdit_;
	QLineEdit * angularUpdate_lineEdit_;
	QLineEdit * linearUpdate_lineEdit_;
	QLineEdit * resampleThreshold_lineEdit_;
	QLineEdit * llsamplerange_lineEdit_;
	QLineEdit * llsamplestep_lineEdit_;
	QLineEdit * lasamplerange_lineEdit_;
	QLineEdit * lasamplestep_lineEdit_;
	QLineEdit * mapscale_lineEdit_;

	QPushButton *set_param_button_;
	QPushButton *open_input_file_button_;
	QPushButton *open_output_file_button_;


private:
	std::string filename_2d_;
	std::string filename_smap_;
	double xmin_;
	double ymin_;
	double xmax_;
	double ymax_;
	double delta_;
	double sigma_;
	double maxUrange_;
	double maxrange_;
	double regscore_;
	double lstep_;
	double astep_;
	double kernelSize_;

	double iterations_;
	double critscore_;
	double maxMove_;
	double lsigma_;
	double ogain_;
	double lskip_;
	bool autosize_;
	bool skipMatching_;

	double srr_;
	double srt_;
	double str_;
	double stt_;

	int particles_;
	int randseed_;

	double angularUpdate_;
	double linearUpdate_;
	double resampleThreshold_;

	double llsamplerange_;
	double llsamplestep_;
	double lasamplerange_;
	double lasamplestep_;
	double mapscale_;
};



#endif//LASERPARAMWIDGET_ALEXWEI_H