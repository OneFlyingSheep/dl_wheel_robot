#include "DLWheelRobotSet.h"
#include "LibDLWheelCustomWidget/BorderWidget.h"
#include "LibDLHangRailCommonWidget/InputWidget.h"
#include "LibDLHangRailCommonWidget/SwitchControl.h"
#include <QHBoxLayout>
#include <QDesktopWidget>
#include <QApplication>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include <QRegExp>

#pragma execution_character_set("utf-8")

DLWheelRobotSet::DLWheelRobotSet(QWidget* parent /* = NULL */)
	: QWidget(parent)
	, m_isInitWidget(false)
{
	this->setStyleSheet("QWidget#BackWidget{background:white;}\
							QLabel#TitleLabel{font-weight:bold;font-size:25px;background:rgb(0, 110, 110)}\
							QPushButton#CommonButton{color:white;font-weight:bold;font-size:14px;background:rgb(0, 110, 110);border-radius:3px;}\
							QLabel#DotSplitter{border-top:2px dashed rgb(0, 110, 110)}\
							QLabel#SolidSplitter{border:2px solid lightgray;}");
}

void DLWheelRobotSet::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initLeftWidget();
	initRightWidget();

	QLabel* splitterLable = new QLabel;
	splitterLable->setFixedWidth(2);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_leftBackWidget);
	hMainLayout->addWidget(splitterLable);
	hMainLayout->addWidget(m_rightBackWidget);
	hMainLayout->setSpacing(0);
	hMainLayout->setMargin(0);
}

void DLWheelRobotSet::initRobotAlarmSet()
{
	m_alarmSetWidget = new BorderWidget;
	m_alarmSetWidget->setTitleText("������ͨ���жϼ��澯����");
	m_alarmSetWidget->setFixedWidth(600);

	m_alarmExecuteMechanism = new InputWidget(InputWidgetType::WheelComboBox_Green);
	m_alarmExecuteMechanism->setTipText("�澯��ִ�л���:");
    m_alarmExecuteMechanism->setComboBoxContent(QStringList() << "���س��" << "����ԭ��");
    
	m_abortExecuteMechanism = new InputWidget(InputWidgetType::WheelComboBox_Green);
	m_abortExecuteMechanism->setTipText("�жϺ�ִ�л���:");
    m_abortExecuteMechanism->setComboBoxContent(QStringList() << "���س��" << "����ԭ��");

	QLabel* dotSplitterLabel = new QLabel;
	dotSplitterLabel->setFixedHeight(2);
	dotSplitterLabel->setObjectName("DotSplitter");

	m_robotMoveSpeed = new InputWidget(InputWidgetType::WheelLineEdit_Green);
	m_robotMoveSpeed->setTipText("��������������ٶ�");
    m_robotMoveSpeed->setTipLabelWidth(180);
    m_robotMoveSpeed->setFixedWidth(290);

	m_radarAlarmDistance = new InputWidget(InputWidgetType::WheelLineEdit_Green);
	m_radarAlarmDistance->setTipText("�״ﱨ������");
    m_radarAlarmDistance->setTipLabelWidth(180);
    m_radarAlarmDistance->setFixedWidth(290);

    m_batteryUpperlimit = new InputWidget(InputWidgetType::WheelLineEdit_Green);
    m_batteryUpperlimit->setTipText("�͵�ѹ����");
    m_batteryUpperlimit->setTipLabelWidth(180);
    m_batteryUpperlimit->setFixedWidth(290);

    QPushButton* pButtonConfigBattery = new QPushButton("����");
    pButtonConfigBattery->setFixedSize(QSize(60, 28));
    pButtonConfigBattery->setObjectName("CommonButton");
    connect(pButtonConfigBattery, &QPushButton::clicked, this, [=] {
        WHEEL_BACK_TO_CORE_SOCKET.set_battery_low_voltage(m_batteryUpperlimit->getLineEditText().toFloat());
        bool isValueRight = WHEEL_BACK_TO_CORE_SOCKET.robot_config_battery_threshold_req(99.0, m_batteryUpperlimit->getLineEditText().toFloat());
        if (!isValueRight)
        {
            DLMessageBox::showDLMessageBox(NULL, "����", "����д��ȷ����ֵ", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });

    QHBoxLayout* hBatteryAlarmLayout = new QHBoxLayout;
    hBatteryAlarmLayout->addWidget(m_batteryUpperlimit);
    hBatteryAlarmLayout->addWidget(pButtonConfigBattery);
    hBatteryAlarmLayout->addStretch();
    hBatteryAlarmLayout->setSpacing(20);
    hBatteryAlarmLayout->setMargin(0);

	QLabel* solidSplitterLabel = new QLabel;
	solidSplitterLabel->setFixedHeight(1);
	solidSplitterLabel->setObjectName("SolidSplitter");

	QPushButton* pButtonSave = new QPushButton("����");
	pButtonSave->setObjectName("CommonButton");
	pButtonSave->setFixedSize(QSize(80, 35));

    // ������ͨ���жϼ��澯����;
    connect(pButtonSave, &QPushButton::clicked, this, [=] {
        int alarmExecuteMechanismIndex = m_alarmExecuteMechanism->getComboBoxCurrentIndex();
        WHEEL_BACK_TO_CORE_SOCKET.robot_config_warning_oper_req(RobotOperationAfterWarning(alarmExecuteMechanismIndex));

        int m_abortExecuteMechanismIndex = m_abortExecuteMechanism->getComboBoxCurrentIndex();
        WHEEL_BACK_TO_CORE_SOCKET.robot_config_disconnect_oper_req(RobotOperationAfterDisconnect(m_abortExecuteMechanismIndex));

        WHEEL_BACK_TO_CORE_SOCKET.set_battery_low_voltage(m_batteryUpperlimit->getLineEditText().toFloat());
        bool isValueRight = WHEEL_BACK_TO_CORE_SOCKET.robot_config_battery_threshold_req(99.9, m_batteryUpperlimit->getLineEditText().toFloat());
        if (!isValueRight)
        {
            DLMessageBox::showDLMessageBox(NULL, "����", "����д��ȷ����ֵ", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });
	
	QPushButton* pButtonReset = new QPushButton("����");
	pButtonReset->setObjectName("CommonButton");
	pButtonReset->setFixedSize(QSize(80, 35));
    connect(pButtonReset, &QPushButton::clicked, this, [=] {
        m_alarmExecuteMechanism->setComboBoxCurrentIndex(0);
        m_abortExecuteMechanism->setComboBoxCurrentIndex(0);

        m_robotMoveSpeed->setLineEditText("");
        m_radarAlarmDistance->setLineEditText("");
        m_batteryUpperlimit->setLineEditText("");
    });

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(pButtonSave);
	hButtonLayout->addWidget(pButtonReset);
	hButtonLayout->setSpacing(10);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vLayout = new QVBoxLayout(m_alarmSetWidget->getCenterWidget());
	vLayout->addWidget(m_alarmExecuteMechanism);
	vLayout->addWidget(m_abortExecuteMechanism);
	vLayout->addWidget(dotSplitterLabel);
	vLayout->addWidget(m_robotMoveSpeed);
	vLayout->addWidget(m_radarAlarmDistance);
	vLayout->addLayout(hBatteryAlarmLayout);
	vLayout->addWidget(solidSplitterLabel);
	vLayout->addLayout(hButtonLayout);
	vLayout->setSpacing(20);
	vLayout->setContentsMargins(50, 10, 30, 30);
}

void DLWheelRobotSet::initRobotPtzControl()
{
	m_ptzControlWidget = new BorderWidget;
	m_ptzControlWidget->setTitleText("��̨����");
	m_ptzControlWidget->setFixedWidth(600);

	QLabel* titleLabel = new QLabel("��̨��ʼλ������");
	titleLabel->setStyleSheet("color:red;font-size:18px;font-weight:bold;");
	titleLabel->setAlignment(Qt::AlignCenter);

	m_ptzXValye = new InputWidget(InputWidgetType::WheelLineEdit_Green);
	m_ptzXValye->setTipText("X");
	m_ptzXValye->setFixedWidth(160);
    m_ptzXValye->setLineEditValidator(new QRegExpValidator(QRegExp("[0-9]+$")));

	m_ptzYValue = new InputWidget(InputWidgetType::WheelLineEdit_Green);
	m_ptzYValue->setTipText("Y");
	m_ptzYValue->setFixedWidth(160);
    m_ptzYValue->setLineEditValidator(new QRegExpValidator(QRegExp("[0-9]+$")));

	QLabel* dotSplitterLabel = new QLabel;
	dotSplitterLabel->setFixedHeight(2);
	dotSplitterLabel->setObjectName("DotSplitter");

	m_ptzHorizontalOffset = new InputWidget(InputWidgetType::WheelLineEdit_Green);
	m_ptzHorizontalOffset->setTipText("��̨ˮƽƫ����");
    m_ptzHorizontalOffset->setLineEditValidator(new QRegExpValidator(QRegExp("[0-9]+$")));
    m_ptzHorizontalOffset->setTipLabelWidth(160);
    m_ptzHorizontalOffset->setFixedWidth(275);

	m_ptzVerticalOffset = new InputWidget(InputWidgetType::WheelLineEdit_Green);
	m_ptzVerticalOffset->setTipText("��̨��ֱƫ����");
    m_ptzVerticalOffset->setLineEditValidator(new QRegExpValidator(QRegExp("[0-9]+$")));
    m_ptzVerticalOffset->setTipLabelWidth(160);
    m_ptzVerticalOffset->setFixedWidth(275);

    m_wheelDiameter = new InputWidget(InputWidgetType::WheelLineEdit_Green);
    m_wheelDiameter->setTipText("����ֱ��");
    m_wheelDiameter->setLineEditValidator(new QRegExpValidator(QRegExp("[0-9]+$")));
    m_wheelDiameter->setTipLabelWidth(160);
    m_wheelDiameter->setFixedWidth(275);

    m_wheelToCeneterDistance = new InputWidget(InputWidgetType::WheelLineEdit_Green);
    m_wheelToCeneterDistance->setTipText("���ӵ������ľ���");
    m_wheelToCeneterDistance->setLineEditValidator(new QRegExpValidator(QRegExp("[0-9]+$")));
    m_wheelToCeneterDistance->setTipLabelWidth(160);
    m_wheelToCeneterDistance->setFixedWidth(275);

	QLabel* solidSplitterLabel = new QLabel;
	solidSplitterLabel->setFixedHeight(1);
	solidSplitterLabel->setObjectName("SolidSplitter");

	QPushButton* pButtonSave = new QPushButton("����");
	pButtonSave->setObjectName("CommonButton");
	pButtonSave->setFixedSize(QSize(80, 35));
    connect(pButtonSave, &QPushButton::clicked, this, [=] {
        WHEEL_BACK_TO_CORE_SOCKET.robot_config_ptz_init_req(m_ptzXValye->getLineEditText().toInt(), m_ptzYValue->getLineEditText().toInt(),
                                                            m_ptzHorizontalOffset->getLineEditText().toInt(), m_ptzVerticalOffset->getLineEditText().toInt());
    });

	QPushButton* pButtonReset = new QPushButton("����");
	pButtonReset->setObjectName("CommonButton");
	pButtonReset->setFixedSize(QSize(80, 35));
    connect(pButtonReset, &QPushButton::clicked, this, [=] {
        m_ptzXValye->setLineEditText("");
        m_ptzYValue->setLineEditText("");
        m_ptzHorizontalOffset->setLineEditText("");
        m_ptzVerticalOffset->setLineEditText("");
        m_wheelDiameter->setLineEditText("");
        m_wheelToCeneterDistance->setLineEditText("");
    });

	QHBoxLayout* hLineEditLayout = new QHBoxLayout;
	hLineEditLayout->addWidget(m_ptzXValye);
	hLineEditLayout->addStretch();
	hLineEditLayout->addWidget(m_ptzYValue);
	hLineEditLayout->setSpacing(0);
	hLineEditLayout->setMargin(0);

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(pButtonSave);
	hButtonLayout->addWidget(pButtonReset);
	hButtonLayout->setSpacing(10);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vLayout = new QVBoxLayout(m_ptzControlWidget->getCenterWidget());
	vLayout->addWidget(titleLabel);
	vLayout->addLayout(hLineEditLayout);
	vLayout->addWidget(dotSplitterLabel);
	vLayout->addWidget(m_ptzHorizontalOffset);
	vLayout->addWidget(m_ptzVerticalOffset);
    vLayout->addWidget(m_wheelDiameter);
    vLayout->addWidget(m_wheelToCeneterDistance);
	vLayout->addWidget(solidSplitterLabel);
	vLayout->addLayout(hButtonLayout);
	vLayout->setSpacing(20);
	vLayout->setContentsMargins(50, 10, 30, 30);
}

void DLWheelRobotSet::initLeftWidget()
{
	initRobotAlarmSet();
	initRobotPtzControl();

	QLabel* robotParamSetTitleLabel = new QLabel("�����˲�������");
	robotParamSetTitleLabel->setObjectName("TitleLabel");
	robotParamSetTitleLabel->setFixedHeight(60);
	robotParamSetTitleLabel->setAlignment(Qt::AlignCenter);

	QDesktopWidget * desktopWidget = QApplication::desktop();
	m_leftBackWidget = new QWidget;
	m_leftBackWidget->setObjectName("BackWidget");
	m_leftBackWidget->setFixedWidth(desktopWidget->screenGeometry().width() / 2);

	QVBoxLayout* vLeftLayout = new QVBoxLayout;
	vLeftLayout->addWidget(m_alarmSetWidget);
	vLeftLayout->addWidget(m_ptzControlWidget);
	vLeftLayout->addStretch();
	vLeftLayout->setSpacing(20);
	vLeftLayout->setMargin(0);

	QHBoxLayout* hLeftLayout = new QHBoxLayout;
	hLeftLayout->addStretch();
	hLeftLayout->addLayout(vLeftLayout);
	hLeftLayout->addStretch();
	hLeftLayout->setMargin(0);

	QVBoxLayout* vLayout = new QVBoxLayout(m_leftBackWidget);
	vLayout->addWidget(robotParamSetTitleLabel);
	vLayout->addLayout(hLeftLayout);
	vLayout->setSpacing(10);
	vLayout->setContentsMargins(0, 0, 0, 20);
}

void DLWheelRobotSet::initRightWidget()
{
	QLabel* robotControlTitleLabel = new QLabel("�����˿���");
	robotControlTitleLabel->setObjectName("TitleLabel");
	robotControlTitleLabel->setFixedHeight(60);
	robotControlTitleLabel->setAlignment(Qt::AlignCenter);

	m_controlMode = new InputWidget(InputWidgetType::WheelComboBox_Green);
	m_controlMode->setTipText("����ģʽ:");

	QVBoxLayout* vControlLayout = new QVBoxLayout;
	vControlLayout->addWidget(m_controlMode);

	SwitchWidget* switchWidget[7];
	for (int i = 0; i < 7; i++)
	{
		switchWidget[i] = new SwitchWidget(true);
		vControlLayout->addWidget(switchWidget[i]);
	}

	vControlLayout->addStretch();
	vControlLayout->setSpacing(50);
	vControlLayout->setContentsMargins(0, 40, 0, 0);

	switchWidget[0]->setSwitchName("���⹦��");
    connect(switchWidget[0], &SwitchWidget::toggled, this, [=](bool isCheck) {
        emit signalInfraredSwitch(isCheck);
    });

	switchWidget[1]->setSwitchName("�ɼ��⹦��");
    connect(switchWidget[0], &SwitchWidget::toggled, this, [=](bool isCheck) {
        emit signalInfraredSwitch(isCheck);
    });

	switchWidget[2]->setSwitchName("��ˢ״̬");
    connect(switchWidget[2], &SwitchWidget::toggled, this, [=](bool isCheck) {
        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_wiper_req(isCheck);
    });

	switchWidget[3]->setSwitchName("���Ϲ���");

	switchWidget[4]->setSwitchName("����״̬");
    connect(switchWidget[4], &SwitchWidget::toggled, this, [=](bool isCheck) {
        WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_light_req(isCheck);
    });

	switchWidget[5]->setSwitchName("��緿");
    connect(switchWidget[5], &SwitchWidget::toggled, this, [=](bool isCheck) {
        WHEEL_BACK_TO_CORE_SOCKET.robot_temporary_door_req(isCheck);
    });
	switchWidget[6]->setSwitchName("������״̬");

	QHBoxLayout* hControlLayout = new QHBoxLayout;
	hControlLayout->addStretch();
	hControlLayout->addLayout(vControlLayout);
	hControlLayout->addStretch();
	hControlLayout->setSpacing(0);
	hControlLayout->setMargin(0);

	QDesktopWidget * desktopWidget = QApplication::desktop();
	m_rightBackWidget = new QWidget;
	m_rightBackWidget->setFixedWidth(desktopWidget->screenGeometry().width() / 2);
	m_rightBackWidget->setObjectName("BackWidget");

	QVBoxLayout* vRightLayout = new QVBoxLayout(m_rightBackWidget);
	vRightLayout->addWidget(robotControlTitleLabel);
	vRightLayout->addLayout(hControlLayout);
	vRightLayout->addStretch();
	vRightLayout->setSpacing(5);
	vRightLayout->setMargin(0);
}