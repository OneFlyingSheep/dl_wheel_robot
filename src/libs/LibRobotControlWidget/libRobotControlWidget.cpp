
/// 机器人控制界面程序

#include "libRobotControlWidget.h"
//#pragma comment(linker,"/subsystem:windows /entry:mainCRTStartup")

LibRobotControlWidget::LibRobotControlWidget(QWidget *parent)
	: QWidget(parent), m_systemInformationWidget(NULL), m_chargingRoomInformationWidget(NULL),
	m_fireHydrantInformationWidget(NULL), m_inspectionRobotConClassWidget(NULL), m_fireFightingRobotInfoWidget(NULL)
{
	connect(&m_updataTimer, SIGNAL(timeout()), this, SLOT(slotUpdataDataInfo()));
	m_updataTimer.start(1000);

	/// 获取当前ip
//	QString nCurrIPNumber = getCurrIPNumber();

#if 0
	/// 获取当前城市天气情况
	manager = new QNetworkAccessManager(this);  //新建QNetworkAccessManager对象
	connect(manager, SIGNAL(finished(QNetworkReply*)), this, SLOT(replyFinished(QNetworkReply*)));//关联信号和槽

	QString local_city = "南京";
	char quest_array[256] = "http://wthrcdn.etouch.cn/weather_mini?city=";
	QNetworkRequest quest;
	sprintf(quest_array, "%s%s", quest_array, local_city.toUtf8().data());
	quest.setUrl(QUrl(quest_array));
	quest.setHeader(QNetworkRequest::UserAgentHeader, "RT-Thread ART");
	/*发送get网络请求*/
	manager->get(quest);
#endif 
	/// 双光云台信息
	m_focusValue = -1;
	m_panValue   = -1;

}

LibRobotControlWidget::~LibRobotControlWidget()
{
}

/// 计时器结束后修改界面相关信息
void LibRobotControlWidget::slotUpdataDataInfo()
{
	if(NULL != m_chargingRoomInformationWidget)
		m_chargingRoomInformationWidget->updateParameInformation();
	if(NULL != m_fireHydrantInformationWidget)
		m_fireHydrantInformationWidget->updateParameInformation();
	if(NULL != m_inspectionRobotConClassWidget)
		m_inspectionRobotConClassWidget->updateParameInformation();
	if(NULL != m_fireFightingRobotInfoWidget)
		m_fireFightingRobotInfoWidget->updateParameInformation();
}

/// 获取当前机器的IP地址
QString LibRobotControlWidget::getCurrIPNumber()
{
	QString mIpAddress;
	return mIpAddress;
}

#if 0
/// 根据反馈信息获取当前城市的天气、温度和风力
void LibRobotControlWidget::replyFinished(QNetworkReply* reply)
{
	QString all = reply->readAll();
	QJsonParseError err;
	QJsonDocument json_recv = QJsonDocument::fromJson(all.toUtf8(), &err);//解析json对象
	if (!json_recv.isNull())
	{
		QJsonObject object = json_recv.object();

		if (object.contains("data"))
		{
			QJsonValue value = object.value("data");  // 获取指定 key 对应的 value
			if (value.isObject())
			{
				QJsonObject object_data = value.toObject();
				if (object_data.contains("forecast"))
				{
					QJsonValue value = object_data.value("forecast");
					if (value.isArray())
					{
						QJsonObject today_weather = value.toArray().at(0).toObject();
						weather_type = today_weather.value("type").toString();

						QString low = today_weather.value("low").toString();
						QString high = today_weather.value("high").toString();
						wendu = low.mid(low.length() - 3, 4) + "~" + high.mid(high.length() - 3, 4);
						QString strength = today_weather.value("fengli").toString();
						strength.remove(0, 8);
						strength.remove(strength.length() - 2, 2);
						fengli = today_weather.value("fengxiang").toString() + strength;
					}
				}
			}
		}
	}
	else
	{
		qDebug() << "json_recv is NULL or is not a object !!";
	}
	reply->deleteLater(); //销毁请求对象

	QString nUpdateTitle = "机器人控制  城市:南京   天气:" + weather_type + "      温度:" + wendu + "      风力:" + fengli;
	this->setWindowTitle(nUpdateTitle);

	/// 根据不同的天气,显示不同的背景图
	QString filePath = QCoreApplication::applicationDirPath();
	QString nImageFileName = filePath + "//test_img//" + weather_type + ".png";
	
	QFileInfo nfileInfo(nImageFileName);
	if(!nfileInfo.exists())
	{
		return;
	}
#if 0
	QPalette palette;
	QPixmap pixmap(nImageFileName);
	palette.setBrush(QPalette::Window, QBrush(pixmap));
	this->setPalette(palette);
#else
	//QString nStyleSheetCMD = "background-image:url(:" + nImageFileName + ");";
	//this->setStyleSheet(nStyleSheetCMD);

	QImage image1;
	image1.load(nImageFileName);
	QImage image2 = image1.scaled(1400, 770);

	QPalette   palette;
	palette.setBrush(QPalette::Window, QBrush(image2));
	this->setPalette(palette);
	this->setAutoFillBackground(true);
#endif
}
#endif

void LibRobotControlWidget::initWidget()
{
	QGridLayout* nThisLayout = new QGridLayout(this);

	QLabel* labelSpacerLeft = new QLabel;
	//labelSpacerLeft->setStyleSheet("background-color:#FFFFFF;");
	labelSpacerLeft->setFixedWidth(300);
	nThisLayout->addWidget(labelSpacerLeft, 1, 0);

	/// 系统信息:天气信息、平台网络状态、平台本地磁盘空间
	QWidget* nSystemInfoGBox = new QWidget;
	nSystemInfoGBox->setFixedHeight(200);
	nSystemInfoGBox->setFixedWidth(600);

	m_systemInformationWidget = new SystemInformation;

	QGridLayout* nSystemLayout = new QGridLayout;
	nSystemLayout->addWidget(m_systemInformationWidget, 0, 0);

	nSystemInfoGBox->setLayout(nSystemLayout);

	nThisLayout->addWidget(nSystemInfoGBox, 0, 1);


	/// 充电房状态(充电状态)
	QWidget* nChargingRoomInfoGBox = new QWidget;
	nChargingRoomInfoGBox->setFixedHeight(200);

	m_chargingRoomInformationWidget = new ChargingRoomInformation;
	QGridLayout* nChargingRoomInfoLayout = new QGridLayout;
	nChargingRoomInfoLayout->addWidget(m_chargingRoomInformationWidget, 0, 0);

	nChargingRoomInfoGBox->setLayout(nChargingRoomInfoLayout);

	nThisLayout->addWidget(nChargingRoomInfoGBox, 0, 2);


	/// 智能消防栓状态:消防栓门开关阀门、消防栓门压力状态、消防栓门开关控制
	QWidget* nFireHydrantInfoGBox = new QWidget;
	nFireHydrantInfoGBox->setFixedHeight(200);
	nFireHydrantInfoGBox->setFixedWidth(600);

	m_fireHydrantInformationWidget = new FireHydrantInformation;
	QGridLayout* nFireHydrantInfoLayout = new QGridLayout;
	nFireHydrantInfoLayout->addWidget(m_fireHydrantInformationWidget, 0, 0);

	nFireHydrantInfoGBox->setLayout(nFireHydrantInfoLayout);

	nThisLayout->addWidget(nFireHydrantInfoGBox, 1, 1);

	///查打一体机器人状态显示以及控制
	QWidget* nInspectionRobotInfo = new QWidget;
	nInspectionRobotInfo->setFixedWidth(600);

	m_inspectionRobotConClassWidget = new InspectionRobotControlClass;
	QGridLayout* nInspectionRobotConClassInfoLayout = new QGridLayout;
	nInspectionRobotConClassInfoLayout->addWidget(m_inspectionRobotConClassWidget, 0, 0);

	nInspectionRobotInfo->setLayout(nInspectionRobotConClassInfoLayout);

	nThisLayout->addWidget(nInspectionRobotInfo, 2, 1);


	/// 消防机器人信息
	QWidget* nFireFightingRobotInfo = new QWidget;

	m_fireFightingRobotInfoWidget = new FirefightingRobotInformation;
	QGridLayout* nfireFightingRobotInfoLayout = new QGridLayout;
	nfireFightingRobotInfoLayout->addWidget(m_fireFightingRobotInfoWidget, 0, 0);

	nFireFightingRobotInfo->setLayout(nfireFightingRobotInfoLayout);

	nThisLayout->addWidget(nFireFightingRobotInfo, 2, 2);

	QLabel* labelSpacerRight = new QLabel;
	//labelSpacerRight->setStyleSheet("background-color:#FFFFFF;");
	labelSpacerRight->setFixedWidth(300);
	nThisLayout->addWidget(labelSpacerRight, 1, 3);

	/// 
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotInfraredFocusSignal.connect(boost::bind(&LibRobotControlWidget::slotSetInfraredFocus, this, _1));


	/// 消防罐云台

	/// 灭火云台

	/// 
	this->setLayout(nThisLayout);
	this->setStyleSheet("background-color:#FFFFFF;");
	this->setStyleSheet("QWidget{background-color:#FFFFFF;}");
}

/// 系统信息相关槽函数
void LibRobotControlWidget::slotDiskSpaceforSystem(int pSpaceRemarning, int pSpaceTotal)	///< 当前系统磁盘空间
{
	if (NULL != m_systemInformationWidget)
	{
		//m_systemInformationWidget->setDiskSpaceInformation(pSpaceRemarning, pSpaceTotal);
	}
}
void LibRobotControlWidget::slotSignalStrengthforSystem(int pSignalStrength)	///< 信号强度
{
	if (NULL != m_systemInformationWidget)
	{
		//m_systemInformationWidget->setSignalStrengthInformation(pSignalStrength);
	}
}


/// 消防栓信息相关槽函数
void LibRobotControlWidget::slotValveSwitchforFireHydrant(bool pSwitch)	///< 阀门开关
{
	if (NULL != m_fireHydrantInformationWidget)
	{
		m_fireHydrantInformationWidget->setValveSwitchInformation(pSwitch);
	}
}

void LibRobotControlWidget::slotValvePressureforFireHydrant(int Pressure)	///< 阀门压力...压强
{
	if (NULL != m_fireHydrantInformationWidget)
	{
		m_fireHydrantInformationWidget->setValvePressureInformation(Pressure);
	}
}

/// 充电房信息相关槽函数
void LibRobotControlWidget::slotSignalStrengthforChargingRoom(int nSignalStrength)	///< 充电房信号强度
{
	if (NULL != m_chargingRoomInformationWidget)
	{
		m_chargingRoomInformationWidget->setSignalStrength(nSignalStrength);
	}
}
void LibRobotControlWidget::slotChargingSwitchStateforChargingRoom(bool nChargingSwitchState)	///< 充电开关状态
{
	if (NULL != m_chargingRoomInformationWidget)
	{
		m_chargingRoomInformationWidget->setChargingSwitchState(nChargingSwitchState);
	}
}
void LibRobotControlWidget::slotWorkingStateforChargingRoom(int nWorkingState)
{
	if (NULL != m_chargingRoomInformationWidget)
	{
		m_chargingRoomInformationWidget->setWorkingStateInformation(nWorkingState);
	}
}


/// 查打一体消防机器人相关槽函数
void LibRobotControlWidget::slotSetInfraredFocus(int focus)	///< 云台位置槽函数
{
	if(NULL != m_inspectionRobotConClassWidget)
		m_inspectionRobotConClassWidget->setInfraredFocus(focus);
}	
void LibRobotControlWidget::slotSetInfraredQuantityOfElectricity(int value)///< 电量
{
	if (NULL != m_inspectionRobotConClassWidget)
		m_inspectionRobotConClassWidget->setQuantityOfExectricity(value);
}

/// 获取消防一体机器人电量
/// 
