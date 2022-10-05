#include <QtWidgets>
#include "DLStatusBar.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"

DLStatusBar::DLStatusBar(QWidget *parent)
	: QStatusBar(parent)
{

	buildProgress_ = 0;
	isBuild_ = false;
	connectState_ = disConnected;
	initStatusBar();

}


void DLStatusBar::paintEvent(QPaintEvent *)
{
	QPainter painter(this);
	painter.setPen(QPen(Qt::lightGray, 2));
	painter.drawRect(this->rect());
}


void DLStatusBar::initStatusBar()
{

	this->setSizeGripEnabled(true);
	state_label_ = new QLabel(("选择"));
	build_map_progressBar_ = new QProgressBar;
	build_map_progressBar_->hide();
	runMode_label_ = new QLabel;
	position_label_ = new QLabel;
	speed_label_ = new QLabel;
	battary_label_ = new QLabel(("电量: "));
	temperature_label_ = new QLabel;
	battary_progressBar_ = new QProgressBar;

	// 模式选择;
	QStringList modeList = QStringList() << ("自动模式") << ("手柄模式") << ("键盘模式") << ("紧急定位");
	m_modeChangeWidget = new InputWidget(InputWidgetType::ComboBox);
	m_modeChangeWidget->setTipText("模式选择:");
	m_modeChangeWidget->setComboBoxContent(modeList);

	connect(m_modeChangeWidget, &InputWidget::signalComboBoxIndexChanged, this, [=](int index) {
		WHEEL_BACK_TO_CORE_SOCKET.robot_config_mode_req(WheelRobotSwitchRunningStatus(index));
	});

	this->addWidget(state_label_, 10);
	this->addWidget(build_map_progressBar_, 10);
	this->addWidget(runMode_label_, 2);
	this->addWidget(position_label_, 2);
	this->addWidget(speed_label_, 4);
	this->addWidget(battary_label_);
	this->addWidget(battary_progressBar_);
	this->addWidget(temperature_label_, 8);

	showConncetedInfo();

	QTimer *timer = new QTimer;
	timer->start(1000);
	connect(timer, SIGNAL(timeout()), this, SLOT(slot_set_statusbar_info()));
}


void DLStatusBar::setOperate(const QString &opreation, int buildProgress, bool isBuild)
{
	buildProgress_ = buildProgress;
	isBuild_ = isBuild;
	if (isBuild) {
		state_label_->setText(opreation);
		build_map_progressBar_->setValue(buildProgress);
		build_map_progressBar_->show();
	}
	else {
		build_map_progressBar_->hide();
		state_label_->setText(opreation);
	}
	showConncetedInfo();
}


void DLStatusBar::slot_set_statusbar_info()
{
	showConncetedInfo();

	if (connectState_ == disConnected) {
		if (isBuild_) {
			build_map_progressBar_->setFormat(QString("当前进度为：%1%").arg(QString::number(buildProgress_, 'f', 1)));
			build_map_progressBar_->setValue(buildProgress_);
		}
	}
	else if (connectState_ == ConnectError) {
		if (!isBuild_) {
			state_label_->setText(("连接错误"));
		}
		else {
			build_map_progressBar_->setFormat(QString("当前进度为：%1%").arg(QString::number(buildProgress_, 'f', 1)));
			build_map_progressBar_->setValue(buildProgress_);
		}
	}
	else {
		int runMode = 0;
		double position_x = 0;
		double position_y = 0;
		double position_dir = 0;
		double speed_x = 0;
		double speed_y = 0;
		double speed_w = 0;
		double battery_level = 0;
		double temperature = 0;

		QString QPosition = ("位置: (") + QString::number(position_x) + " ," + QString::number(position_y) + (")m 方向:") + QString::number(position_dir);
		QString QSpeed = ("速度: (") + QString::number(speed_x) + " ," + QString::number(speed_y) + ") m/s  " + QString::number(speed_w) + " rad/s";
		QString Qtemperature = ("温度: ") + QString::number(temperature) + ("°");

		if (runMode == 0) {
			runMode_label_->setText(("自动模式"));
		}
		else if (runMode == 1) {
			runMode_label_->setText(("后台模式"));
		}
		else if (runMode == 2) {
			runMode_label_->setText(("手柄模式"));
		}
		else {
			runMode_label_->setText(("紧急模式"));
		}
		position_label_->setText(QPosition);
		speed_label_->setText(QSpeed);
		temperature_label_->setText(Qtemperature);
		battary_progressBar_->setValue((int)(battery_level * 100));
	}
}


void DLStatusBar::setConnectState(int state)
{

	connectState_ = ChassisConnectState(state);

}


void DLStatusBar::showConncetedInfo()
{

	if (connectState_ == Connected) {
		state_label_->hide();
		build_map_progressBar_->hide();
		runMode_label_->show();
		position_label_->show();
		speed_label_->show();
		battary_label_->show();
		temperature_label_->show();
		battary_progressBar_->show();
	}
	else {
		state_label_->show();
		if (isBuild_) {
			build_map_progressBar_->show();
		}

		runMode_label_->hide();
		position_label_->hide();
		speed_label_->hide();
		battary_label_->hide();
		temperature_label_->hide();
		battary_progressBar_->hide();
	}


}

DLStatusBar::~DLStatusBar()
{

}

