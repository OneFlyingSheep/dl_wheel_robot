#include <QtWidgets>
#include "LaserParamWidget.h"


LaserParamWidget::LaserParamWidget(QWidget *parent)
	: QWidget(parent)
{

	filename_2d_ = "raw_map.2d";
	filename_smap_ = "DL.smap";
	xmin_ = -100.;
	ymin_ = -100.;
	xmax_ = 100.;
	ymax_ = 100.;
	delta_ = 0.02;
	sigma_ = 0.05;
	maxUrange_ = 80;
	maxrange_ = 80.;
	regscore_ = 1e4;
	lstep_ = .05;
	astep_ = .05;
	kernelSize_ = 1;
	iterations_ = 5;
	critscore_ = 0.;
	maxMove_ = 1.;
	lsigma_ = .075;
	ogain_ = 3;
	lskip_ = 0;
	autosize_ = false;
	skipMatching_ = false;
	srr_ = 0.01;
	srt_ = 0.2;
	str_ = 0.01;
	stt_ = 0.2;
	particles_ = 30;
	randseed_ = 0;
	angularUpdate_ = 0.05;
	linearUpdate_ = 0.5;
	resampleThreshold_ = 0.5;
	llsamplerange_ = 0.01;
	llsamplestep_ = 0.01;
	lasamplerange_ = 0.005;
	lasamplestep_ = 0.005;
	mapscale_ = 40;

	initWidget();

}


void LaserParamWidget::initWidget()
{
	this->setWindowModality(Qt::ApplicationModal);
	//setWindowFlags(Qt::FramelessWindowHint);
	setWindowTitle(QStringLiteral("构建地图参数设置"));

	input_fileName_label_ = new QLabel(QStringLiteral("2DFilName"));
	output_fileName_label_ = new QLabel(QStringLiteral("SmapFilName"));
	xmin_label_ = new QLabel(QStringLiteral("xmin"));
	ymin_label_ = new QLabel(QStringLiteral("ymin"));
	xmax_label_ = new QLabel(QStringLiteral("xmax"));
	ymax_label_ = new QLabel(QStringLiteral("ymax"));
	delta_label_ = new QLabel(QStringLiteral("delta"));
	mapscale_label_ = new QLabel(QStringLiteral("mapscale"));
	maxUrange_label_ = new QLabel(QStringLiteral("maxUrange"));
	maxrange_label_ = new QLabel(QStringLiteral("maxRange"));
	linearUpdate_label_ = new QLabel(QStringLiteral("linearUpdate"));
	angularUpdate_label_ = new QLabel(QStringLiteral("angularUpdate"));
	resampleThreshold_label_ = new QLabel(QStringLiteral("resampleThreshold"));
	particles_label_ = new QLabel(QStringLiteral("particles"));
	iterations_label_ = new QLabel(QStringLiteral("iterations"));

	input_fileName_lineEdit_ = new QLineEdit;
	output_fileName_lineEdit_ = new QLineEdit;
	xmin_lineEdit_ = new QLineEdit;
	ymin_lineEdit_ = new QLineEdit;
	xmax_lineEdit_ = new QLineEdit;
	ymax_lineEdit_ = new QLineEdit;
	delta_lineEdit_ = new QLineEdit;
	mapscale_lineEdit_ = new QLineEdit;
	maxUrange_lineEdit_ = new QLineEdit;
	maxrange_lineEdit_ = new QLineEdit;
	angularUpdate_lineEdit_ = new QLineEdit;
	linearUpdate_lineEdit_ = new QLineEdit;
	resampleThreshold_lineEdit_ = new QLineEdit;
	particles_lineEdit_ = new QLineEdit;
	iterations_lineEdit_ = new QLineEdit;


	QWidget *widget = new QWidget;
	QGridLayout *widget_layout = new QGridLayout;
	widget_layout->addWidget(xmin_label_, 0, 0);
	widget_layout->addWidget(xmin_lineEdit_, 0, 1);
	widget_layout->addWidget(ymin_label_, 0, 2);
	widget_layout->addWidget(ymin_lineEdit_, 0, 3);
	widget_layout->addWidget(xmax_label_, 0, 4);
	widget_layout->addWidget(xmax_lineEdit_, 0, 5);
	widget_layout->addWidget(ymax_label_, 1, 0);
	widget_layout->addWidget(ymax_lineEdit_, 1, 1);
	widget_layout->addWidget(delta_label_, 1, 2);
	widget_layout->addWidget(delta_lineEdit_, 1, 3);
	widget_layout->addWidget(mapscale_label_, 1, 4);
	widget_layout->addWidget(mapscale_lineEdit_, 1, 5);
	widget_layout->addWidget(maxUrange_label_, 2, 0);
	widget_layout->addWidget(maxUrange_lineEdit_, 2, 1);
	widget_layout->addWidget(maxrange_label_, 2, 2);
	widget_layout->addWidget(maxrange_lineEdit_, 2, 3);
	widget_layout->addWidget(linearUpdate_label_, 2, 4);
	widget_layout->addWidget(linearUpdate_lineEdit_, 2, 5);
	widget_layout->addWidget(angularUpdate_label_, 3, 0);
	widget_layout->addWidget(angularUpdate_lineEdit_, 3, 1);
	widget_layout->addWidget(resampleThreshold_label_, 3, 2);
	widget_layout->addWidget(resampleThreshold_lineEdit_, 3, 3);
	widget_layout->addWidget(particles_label_, 3, 4);
	widget_layout->addWidget(particles_lineEdit_, 3, 5);
	widget_layout->addWidget(iterations_label_, 4, 0);
	widget_layout->addWidget(iterations_lineEdit_, 4, 1);
	widget_layout->addWidget(input_fileName_label_, 4, 2);
	widget_layout->addWidget(input_fileName_lineEdit_, 4, 3);
	widget_layout->addWidget(output_fileName_label_, 4, 4);
	widget_layout->addWidget(output_fileName_lineEdit_, 4, 5);
	widget->setLayout(widget_layout);

	/////////////////////////////////////////////////
	QPushButton *more_info_button = new QPushButton(QStringLiteral("更多"));
	more_info_button->setCheckable(true);
	more_info_button->setChecked(Qt::Unchecked);

	more_info_groupBox_ = new QGroupBox;
	QGridLayout *groupBox_layout = new QGridLayout;

	sigma_label_ = new QLabel(QStringLiteral("sigma"));
	regscore_label_ = new QLabel(QStringLiteral("regscore"));
	lstep_label_ = new QLabel(QStringLiteral("lstep"));
	astep_label_ = new QLabel(QStringLiteral("astep"));
	kernelSize_label_ = new QLabel(QStringLiteral("kernelSize"));
	critscore_label_ = new QLabel(QStringLiteral("critscore"));
	maxMove_label_ = new QLabel(QStringLiteral("maxMove"));
	lsigma_label_ = new QLabel(QStringLiteral("lsigma"));
	ogain_label_ = new QLabel(QStringLiteral("ogain"));
	lskip_label_ = new QLabel(QStringLiteral("lskip"));
	autosize_label_ = new QLabel(QStringLiteral("autosize"));
	skipMatching_label_ = new QLabel(QStringLiteral("skipMatching"));
	srr_label_ = new QLabel(QStringLiteral("srr"));
	srt_label_ = new QLabel(QStringLiteral("srt"));
	str_label_ = new QLabel(QStringLiteral("str"));
	stt_label_ = new QLabel(QStringLiteral("stt"));
	randseed_label_ = new QLabel(QStringLiteral("randseed"));
	llsamplerange_label_ = new QLabel(QStringLiteral("llsamplerange"));
	llsamplestep_label_ = new QLabel(QStringLiteral("llsamplestep"));
	lasamplerange_label_ = new QLabel(QStringLiteral("lasamplerange"));
	lasamplestep_label_ = new QLabel(QStringLiteral("lasamplestep"));

	sigma_lineEdit_ = new QLineEdit;
	regscore_lineEdit_ = new QLineEdit;
	lstep_lineEdit_ = new QLineEdit;
	astep_lineEdit_ = new QLineEdit;
	kernelSize_lineEdit_ = new QLineEdit;
	critscore_lineEdit_ = new QLineEdit;
	maxMove_lineEdit_ = new QLineEdit;
	lsigma_lineEdit_ = new QLineEdit;
	ogain_lineEdit_ = new QLineEdit;
	lskip_lineEdit_ = new QLineEdit;
	autosize_chechBox_ = new QCheckBox;
	skipMatching_checkBox_ = new QCheckBox;
	srr_lineEdit_ = new QLineEdit;
	srt_lineEdit_ = new QLineEdit;
	str_lineEdit_ = new QLineEdit;
	stt_lineEdit_ = new QLineEdit;
	randseed_lineEdit_ = new QLineEdit;
	llsamplerange_lineEdit_ = new QLineEdit;
	llsamplestep_lineEdit_ = new QLineEdit;
	lasamplerange_lineEdit_ = new QLineEdit;
	lasamplestep_lineEdit_ = new QLineEdit;

	groupBox_layout->addWidget(sigma_label_, 2, 0);
	groupBox_layout->addWidget(sigma_lineEdit_, 2, 1);
	groupBox_layout->addWidget(regscore_label_, 3, 0);
	groupBox_layout->addWidget(regscore_lineEdit_, 3, 1);
	groupBox_layout->addWidget(lstep_label_, 3, 2);
	groupBox_layout->addWidget(lstep_lineEdit_, 3, 3);
	groupBox_layout->addWidget(kernelSize_label_, 4, 0);
	groupBox_layout->addWidget(kernelSize_lineEdit_, 4, 1);
	groupBox_layout->addWidget(critscore_label_, 4, 4);
	groupBox_layout->addWidget(critscore_lineEdit_, 4, 5);
	groupBox_layout->addWidget(maxMove_label_, 5, 0);
	groupBox_layout->addWidget(maxMove_lineEdit_, 5, 1);
	groupBox_layout->addWidget(lsigma_label_, 5, 2);
	groupBox_layout->addWidget(lsigma_lineEdit_, 5, 3);
	groupBox_layout->addWidget(ogain_label_, 5, 4);
	groupBox_layout->addWidget(ogain_lineEdit_, 5, 5);
	groupBox_layout->addWidget(lskip_label_, 6, 0);
	groupBox_layout->addWidget(lskip_lineEdit_, 6, 1);
	groupBox_layout->addWidget(autosize_label_, 6, 2);
	groupBox_layout->addWidget(autosize_chechBox_, 6, 3);
	groupBox_layout->addWidget(skipMatching_label_, 6, 4);
	groupBox_layout->addWidget(skipMatching_checkBox_, 6, 5);
	groupBox_layout->addWidget(srr_label_, 7, 0);
	groupBox_layout->addWidget(srr_lineEdit_, 7, 1);
	groupBox_layout->addWidget(srt_label_, 7, 2);
	groupBox_layout->addWidget(srt_lineEdit_, 7, 3);
	groupBox_layout->addWidget(str_label_, 7, 4);
	groupBox_layout->addWidget(str_lineEdit_, 7, 5);
	groupBox_layout->addWidget(stt_label_, 8, 0);
	groupBox_layout->addWidget(stt_lineEdit_, 8, 1);
	groupBox_layout->addWidget(randseed_label_, 8, 4);
	groupBox_layout->addWidget(randseed_lineEdit_, 8, 5);
	groupBox_layout->addWidget(llsamplerange_label_, 10, 0);
	groupBox_layout->addWidget(llsamplerange_lineEdit_, 10, 1);
	groupBox_layout->addWidget(llsamplestep_label_, 10, 2);
	groupBox_layout->addWidget(llsamplestep_lineEdit_, 10, 3);
	groupBox_layout->addWidget(lasamplerange_label_, 10, 4);
	groupBox_layout->addWidget(lasamplerange_lineEdit_, 10, 5);
	groupBox_layout->addWidget(lasamplestep_label_, 11, 0);
	groupBox_layout->addWidget(lasamplestep_lineEdit_, 11, 1);

	more_info_groupBox_->setLayout(groupBox_layout);
	
	QVBoxLayout *info_layout = new QVBoxLayout;
	info_layout->addWidget(more_info_button);
	info_layout->addWidget(more_info_groupBox_);
	info_layout->setSpacing(0);
	more_info_groupBox_->hide();

	QHBoxLayout *oprerate_layout = new QHBoxLayout;
	set_param_button_ = new QPushButton(QStringLiteral("建图"));
	open_input_file_button_ = new QPushButton(QStringLiteral("导入2D地图"));
	open_output_file_button_ = new QPushButton(QStringLiteral("输出smap地图"));
	oprerate_layout->addWidget(open_input_file_button_);
	oprerate_layout->addWidget(open_output_file_button_);
	oprerate_layout->addWidget(set_param_button_);

	QVBoxLayout *main_layout = new QVBoxLayout;
	main_layout->addWidget(widget);
	main_layout->addLayout(info_layout);
	main_layout->addLayout(oprerate_layout);
	
	initLaserParam();
	this->setLayout(main_layout);
	this->setFixedSize(700, 250);
	connect(open_input_file_button_, SIGNAL(clicked()), this, SLOT(slot_on_open_input_file()));
	connect(open_output_file_button_, SIGNAL(clicked()), this, SLOT(slot_on_open_output_file()));
	connect(set_param_button_, SIGNAL(clicked()), this, SLOT(slot_on_set_param()));
	connect(more_info_button, SIGNAL(clicked(bool)), this, SLOT(slot_on_show_more_param(bool)));
}


void LaserParamWidget::initLaserParam()
{
	input_fileName_lineEdit_->setText(QStringLiteral(""));
	output_fileName_lineEdit_->setText(QStringLiteral(""));
	xmin_lineEdit_->setText(QString::number(-100));
	ymin_lineEdit_->setText(QString::number(-100));
	xmax_lineEdit_->setText(QString::number(100));
	ymax_lineEdit_->setText(QString::number(100));
	delta_lineEdit_->setText(QString::number(0.02));
	sigma_lineEdit_->setText(QString::number(0.05));
	maxUrange_lineEdit_->setText(QString::number(80));
	maxrange_lineEdit_->setText(QString::number(80));
	regscore_lineEdit_->setText(QString::number(1e4));
	lstep_lineEdit_->setText(QString::number(.05));
	astep_lineEdit_->setText(QString::number(.05));
	kernelSize_lineEdit_->setText(QString::number(1));
	iterations_lineEdit_->setText(QString::number(5));
	critscore_lineEdit_->setText(QString::number(0.));
	maxMove_lineEdit_->setText(QString::number(1.));
	lsigma_lineEdit_->setText(QString::number(0.075));
	ogain_lineEdit_->setText(QString::number(3));
	lskip_lineEdit_->setText(QString::number(0));
	autosize_chechBox_->setCheckState(Qt::Unchecked);
	skipMatching_checkBox_->setCheckState(Qt::Unchecked);
	srr_lineEdit_->setText(QString::number(0.01));
	srt_lineEdit_->setText(QString::number(0.2));
	str_lineEdit_->setText(QString::number(0.01));
	stt_lineEdit_->setText(QString::number(0.2));
	particles_lineEdit_->setText(QString::number(30));
	randseed_lineEdit_->setText(QString::number(0));
	angularUpdate_lineEdit_->setText(QString::number(0.05));
	linearUpdate_lineEdit_->setText(QString::number(0.5));
	resampleThreshold_lineEdit_->setText(QString::number(0.5));
	llsamplerange_lineEdit_->setText(QString::number(0.01));
	llsamplestep_lineEdit_->setText(QString::number(0.01));
	lasamplerange_lineEdit_->setText(QString::number(0.005));
	lasamplestep_lineEdit_->setText(QString::number(0.005));
	mapscale_lineEdit_->setText(QString::number(40.));
}


BuildSmapParam LaserParamWidget::getLaserParam()
{
	BuildSmapParam laserParam;
	laserParam.filename_2d_ = input_fileName_lineEdit_->text().toLocal8Bit().constData();
	laserParam.filename_smap_ = output_fileName_lineEdit_->text().toLocal8Bit().constData();
	laserParam.xmin_ = xmin_lineEdit_->text().toDouble();
	laserParam.ymin_ = ymin_lineEdit_->text().toDouble();
	laserParam.xmax_ = xmax_lineEdit_->text().toDouble();
	laserParam.ymax_ = ymax_lineEdit_->text().toDouble();
	laserParam.delta_ = delta_lineEdit_->text().toDouble();
	laserParam.sigma_ = sigma_lineEdit_->text().toDouble();
	laserParam.maxUrange_ = maxUrange_lineEdit_->text().toDouble();
	laserParam.maxrange_ = maxrange_lineEdit_->text().toDouble();
	laserParam.regscore_ = regscore_lineEdit_->text().toDouble();
	laserParam.lstep_ = lstep_lineEdit_->text().toDouble();
	laserParam.astep_ = astep_lineEdit_->text().toDouble();
	laserParam.kernelSize_ = kernelSize_lineEdit_->text().toDouble();
	laserParam.iterations_ = iterations_lineEdit_->text().toDouble();
	laserParam.critscore_ = critscore_lineEdit_->text().toDouble();
	laserParam.maxMove_ = maxMove_lineEdit_->text().toDouble();
	laserParam.lsigma_ = lsigma_lineEdit_->text().toDouble();
	laserParam.ogain_ = ogain_lineEdit_->text().toDouble();
	laserParam.lskip_ = lskip_lineEdit_->text().toDouble();
	if (autosize_chechBox_->checkState() == Qt::Unchecked) {
		laserParam.autosize_ = false;
	}
	else {
		laserParam.autosize_ = true;
	}
	if (skipMatching_checkBox_->checkState() == Qt::Unchecked) {
		laserParam.skipMatching_ = false;
	}
	else {
		laserParam.skipMatching_ = true;
	}
	laserParam.srr_ = srr_lineEdit_->text().toDouble();
	laserParam.srt_ = srt_lineEdit_->text().toDouble();
	laserParam.str_ = str_lineEdit_->text().toDouble();
	laserParam.stt_ = stt_lineEdit_->text().toDouble();
	laserParam.particles_ = particles_lineEdit_->text().toDouble();
	laserParam.randseed_ = randseed_lineEdit_->text().toDouble();
	laserParam.angularUpdate_ = angularUpdate_lineEdit_->text().toDouble();
	laserParam.linearUpdate_ = linearUpdate_lineEdit_->text().toDouble();
	laserParam.resampleThreshold_ = resampleThreshold_lineEdit_->text().toDouble();
	laserParam.llsamplerange_ = llsamplerange_lineEdit_->text().toDouble();
	laserParam.llsamplestep_ = llsamplestep_lineEdit_->text().toDouble();
	laserParam.lasamplerange_ = lasamplerange_lineEdit_->text().toDouble();
	laserParam.lasamplestep_ = lasamplestep_lineEdit_->text().toDouble();
	laserParam.mapscale_ = mapscale_lineEdit_->text().toDouble();

	return laserParam;
}


void LaserParamWidget::slot_on_set_param()
{
	this->close();
	emit sig_param_ok();
}


void LaserParamWidget::slot_on_show_more_param(bool state)
{
	if (state) {
		more_info_groupBox_->show();
		this->setFixedSize(700, 500);
	}
	else {
		more_info_groupBox_->hide();
		this->setFixedSize(700, 250);
	}
}


LaserParamWidget::~LaserParamWidget()
{
	
}


void LaserParamWidget::slot_on_open_input_file()
{
	QString fileName = QFileDialog::getOpenFileName(this, QStringLiteral("选择输入2D地图"), QStringLiteral("D:\RCF_Server\2dmap"), QStringLiteral("Maps (*.2d)"));
	if (fileName.isEmpty() || fileName.isNull()) {
		return;
	}
	input_fileName_lineEdit_->setText(fileName);
}


void LaserParamWidget::slot_on_open_output_file()
{
	QString fileName = QFileDialog::getExistingDirectory(this, QStringLiteral("选择输出Smap地图"), QStringLiteral("D:\\RCF_Server\\smap"));
	if (fileName.isEmpty() || fileName.isNull()) {
		return;
	}

	QStringList path_list = input_fileName_lineEdit_->text().split("/");
	QStringList file_list;
	QString file_name = path_list.last();
	if (path_list.size() < 2) {
		fileName += "/DL.smap";
	}
	else {
		file_list = file_name.split(".");
		if (file_list.size() < 2) {
			fileName += "/DL.smap";
		}
		else {
			fileName = fileName + "/" + file_list.at(0) + ".smap";
		}
	}
	output_fileName_lineEdit_->setText(fileName);
}





