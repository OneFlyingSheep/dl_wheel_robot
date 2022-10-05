#include "SmapBuilder.h"
#include <QTimer>

SmapBuilder::SmapBuilder(QObject* parent)
{
	timer_ = new QTimer(this);
	connect(timer_, SIGNAL(timeout()), this, SLOT(slot_get_process()));
}


void SmapBuilder::run()
{
	Startmap(param_);
	emit sig_finished();
}


void SmapBuilder::set_file_name(const std::string &file_name)
{
	file_name_ = file_name;
}


void SmapBuilder::slot_get_process()
{
	float process = getPercent();
	emit sig_process(process);
}


void SmapBuilder::set_map_param(const BuildSmapParam &param)
{
	param_ = param;

}

void SmapBuilder::start_timer()
{
	timer_->start(1000);
}


void SmapBuilder::stop_timer()
{
	timer_->stop();
}




