#include "DLWheelPatrolPointPosSet.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotStationConfig.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelTaskAdministrationData.h"
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"
#include <QUuid>
#include <QFileDialog>
#include "LibDLWheelCustomWidget/DefData.h"

#pragma execution_character_set("utf-8")

#define TABLE_PER_PAGE_COUNT 15

PointPosSetAddWindow::PointPosSetAddWindow(QWidget* parent /* = NULL */)
    : QWidget(parent)
    , m_isModifyData(false)
{
    initWidget();

	this->setWindowModality(Qt::ApplicationModal);
	this->setAttribute(Qt::WA_DeleteOnClose);
	this->setWindowFlags(Qt::FramelessWindowHint | Qt::Tool);
	this->setStyleSheet("QWidget#CenterWidget{background:rgb(240, 240, 240);border:2px solid rgb(166, 233, 210);}");
	this->setFixedSize(QSize(600, 450));
}

void PointPosSetAddWindow::setData(WheelPatrolPointSet data)
{
    m_isModifyData = true;
    m_windowData = data;

    m_voltageLevelWidget->setComboBoxCurrentContent(data.voltahe_level_name);
    m_intervalNameWidget->setComboBoxCurrentContent(data.equipment_interval_name) ;
    m_deviceAreaWidget->setComboBoxCurrentContent(data.device_area_name);
    m_deviceTypeWidget->setComboBoxCurrentContent(data.device_type_name);
    m_deviceSubTypeWidget->setComboBoxCurrentContent(data.sub_device_type);
    m_pointNameWidget->setComboBoxCurrentContent(data.device_point_type_name);

    m_titleLabel->setText("��λ�����޸�");
}

void PointPosSetAddWindow::initCenterWidget()
{
	m_centerWidget = new QWidget;
	m_centerWidget->setObjectName("CenterWidget");

	QLabel* titleLabel = new QLabel("��λ�������");
	titleLabel->setStyleSheet("background:qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(220, 240, 240, 200),\
										stop:0.3 rgba(220, 240, 240, 255), stop:1 rgba(200, 230, 230, 255));\
										color:rgb(40, 120, 100);font-weight:bold;padding-left:3px;");
	titleLabel->setFixedHeight(30);

	m_pointPosPath = new InputWidget(InputWidgetType::WheelLineEdit);
	m_pointPosPath->setTipText("��λ·��");
	m_pointPosPath->setLineEditWidth(370);
	m_pointPosPath->setFixedWidth(460);

	m_deviceType = new InputWidget(InputWidgetType::WheelLineEdit);
	m_deviceType->setTipText("�豸����");
	m_deviceType->setLineEditWidth(370);
	m_deviceType->setFixedWidth(460);

	m_deviceArea = new InputWidget(InputWidgetType::WheelLineEdit);
	m_deviceArea->setTipText("�豸����");
	m_deviceArea->setLineEditWidth(370);
	m_deviceArea->setFixedWidth(460);

	m_connectObject = new InputWidget(InputWidgetType::WheelComboBox);
	m_connectObject->setTipText("���Ӷ���");
	m_connectObject->setTipLabelWidth(85);
	m_connectObject->setFixedWidth(210);

	m_protocolParam = new InputWidget(InputWidgetType::WheelLineEdit);
	m_protocolParam->setTipText("��Լ����");

	m_pointPosChoose = new InputWidget(InputWidgetType::WheelLineEdit);
	m_pointPosChoose->setTipText("��λѡ��");

	m_pButtonSearch = new QToolButton;
	m_pButtonSearch->setIcon(QIcon(":/Resources/Common/image/Search.png"));
	m_pButtonSearch->setIconSize(QSize(16, 16));
	m_pButtonSearch->setFixedSize(QSize(30, 30));
	m_pButtonSearch->setStyleSheet("border:none;");

	QHBoxLayout* hPointPosChooseLayout = new QHBoxLayout;
	hPointPosChooseLayout->addWidget(m_pointPosChoose);
	hPointPosChooseLayout->addWidget(m_pButtonSearch);
	hPointPosChooseLayout->setMargin(0);
	hPointPosChooseLayout->setSpacing(2);

	m_pointPosCollect = new InputWidget(InputWidgetType::WheelLineEdit);
	m_pointPosCollect->setTipText("������λ��");

	m_pointPosSign = new InputWidget(InputWidgetType::WheelLineEdit);
	m_pointPosSign->setTipText("��λ��ʶ");

	m_identifyType = new InputWidget(InputWidgetType::WheelLineEdit);
	m_identifyType->setTipText("ʶ������");

	m_meterType = new InputWidget(InputWidgetType::WheelLineEdit);
	m_meterType->setTipText("�������");

	m_feverType = new InputWidget(InputWidgetType::WheelLineEdit);
	m_feverType->setTipText("��������");

	m_deviceAppearenceCheckType = new InputWidget(InputWidgetType::WheelLineEdit);
	m_deviceAppearenceCheckType->setTipText("�豸��ۼ��");

	QLabel* checkBoxLabel = new QLabel;
	checkBoxLabel->setText("��������:");
	checkBoxLabel->setFixedWidth(85);

	m_checkBoxInfrared = new QCheckBox;
	m_checkBoxInfrared->setText("�������");

	m_checkBoxPicture = new QCheckBox;
	m_checkBoxPicture->setText("����ͼƬ");

	m_checkBoxAudio = new QCheckBox;
	m_checkBoxAudio->setText("������Ƶ");

	QHBoxLayout* hCheckBoxLayout = new QHBoxLayout;
	hCheckBoxLayout->addWidget(checkBoxLabel);
	hCheckBoxLayout->addWidget(m_checkBoxInfrared);
	hCheckBoxLayout->addWidget(m_checkBoxPicture);
	hCheckBoxLayout->addWidget(m_checkBoxAudio);
	hCheckBoxLayout->setMargin(0);

	m_pButtonCommit = new QToolButton;
	m_pButtonCommit->setText("�ύ");
	m_pButtonCommit->setFixedSize(QSize(70, 25));
//    connect(m_pButtonCommit, &QToolButton::clicked, this, &PointPosSetAddWindow::signalCommitButtonClicked);


	QHBoxLayout* hCommitButtonLayout = new QHBoxLayout;
	hCommitButtonLayout->addStretch();
	hCommitButtonLayout->addWidget(m_pButtonCommit);
	hCommitButtonLayout->addStretch();
	hCommitButtonLayout->setMargin(0);

	QGridLayout* gInputWidgetLayout = new QGridLayout;
	gInputWidgetLayout->addWidget(m_pointPosPath, 0, 0, 1, 2);
	gInputWidgetLayout->addWidget(m_deviceType, 1, 0, 1, 2);
	gInputWidgetLayout->addWidget(m_deviceArea, 2, 0, 1, 2);
	gInputWidgetLayout->addWidget(m_connectObject, 3, 0, 1, 1);
	gInputWidgetLayout->addWidget(m_protocolParam, 3, 1, 1, 1);
	gInputWidgetLayout->addLayout(hPointPosChooseLayout, 4, 0, 1, 1);
	gInputWidgetLayout->addWidget(m_pointPosCollect, 5, 0, 1, 1);
	gInputWidgetLayout->addWidget(m_pointPosSign, 5, 1, 1, 1);
	gInputWidgetLayout->addWidget(m_identifyType, 6, 0, 1, 1);
	gInputWidgetLayout->addWidget(m_meterType, 6, 1, 1, 1);
	gInputWidgetLayout->addWidget(m_feverType, 7, 0, 1, 1);
	gInputWidgetLayout->addWidget(m_deviceAppearenceCheckType, 7, 1, 1, 1);
	gInputWidgetLayout->addLayout(hCheckBoxLayout, 8, 0, 1, 2);
	gInputWidgetLayout->setSpacing(15);

	QHBoxLayout* hInputWidgetLayout = new QHBoxLayout;
	hInputWidgetLayout->addStretch();
	hInputWidgetLayout->addLayout(gInputWidgetLayout);
	hInputWidgetLayout->addStretch();
	hInputWidgetLayout->setMargin(0);

	QVBoxLayout* vCenterLayout = new QVBoxLayout(m_centerWidget);
	vCenterLayout->addWidget(titleLabel);
	vCenterLayout->addLayout(hInputWidgetLayout);
	vCenterLayout->addSpacing(30);
	vCenterLayout->addLayout(hCommitButtonLayout);
	vCenterLayout->setSpacing(10);
	vCenterLayout->setContentsMargins(2, 2, 2, 20);
}

void PointPosSetAddWindow::initNewCenterWidget()
{
    m_centerWidget = new QWidget;
    m_centerWidget->setObjectName("CenterWidget");

    m_titleLabel = new QLabel("��λ�������");
    m_titleLabel->setStyleSheet("background:qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 rgba(220, 240, 240, 200),\
										stop:0.3 rgba(220, 240, 240, 255), stop:1 rgba(200, 230, 230, 255));\
										color:rgb(40, 120, 100);font-weight:bold;padding-left:3px;");
    m_titleLabel->setFixedHeight(30);

    m_stationNameWidget = new InputWidget(InputWidgetType::WheelComboBox);
    m_stationNameWidget->setTipText("���վ����");
    m_stationNameWidget->setComboBoxWidth(270);
    m_stationNameWidget->setFixedWidth(350);

    m_voltageLevelWidget = new InputWidget(InputWidgetType::WheelComboBox);
    m_voltageLevelWidget->setTipText("��ѹ�ȼ�");
    m_voltageLevelWidget->setComboBoxWidth(270);
    m_voltageLevelWidget->setFixedWidth(350);
    m_voltageLevelWidget->getComboBoxWidget()->setEditable(true);

    connect(m_voltageLevelWidget, &InputWidget::signalComboBoxIndexChanged, this, [=] {
        QString voltageLevelId = m_voltageLevelMap.key(m_voltageLevelWidget->getComboBoxCurrentContent());
        m_intervalNameMap.clear();
        std::map<QString, QString> intervalNameMap = WHEEL_DEVICE_CONFIG.getWheelRobortEquipmentIntervalFromVoltageLevelId(voltageLevelId);
        QStringList intervalNameList;
        std::map<QString, QString>::iterator intervalNameIter;
        for (intervalNameIter = intervalNameMap.begin(); intervalNameIter != intervalNameMap.end(); intervalNameIter++)
        {
            m_intervalNameMap.insert(intervalNameIter->first, intervalNameIter->second);
            intervalNameList.append(intervalNameIter->second);
        }
        m_intervalNameWidget->setComboBoxContent(intervalNameList);
    });

    m_intervalNameWidget = new InputWidget(InputWidgetType::WheelComboBox);
    m_intervalNameWidget->setTipText("�������");
    m_intervalNameWidget->setComboBoxWidth(270);
    m_intervalNameWidget->setFixedWidth(350);
    m_intervalNameWidget->getComboBoxWidget()->setEditable(true);

    m_deviceAreaWidget = new InputWidget(InputWidgetType::WheelComboBox);
    m_deviceAreaWidget->setTipText("�豸����");
    m_deviceAreaWidget->setComboBoxWidth(270);
    m_deviceAreaWidget->setFixedWidth(350);
    m_deviceAreaWidget->getComboBoxWidget()->setEditable(true);

    m_deviceTypeWidget = new InputWidget(InputWidgetType::WheelComboBox);
    m_deviceTypeWidget->setTipText("�豸����");
    m_deviceTypeWidget->setComboBoxWidth(270);
    m_deviceTypeWidget->setFixedWidth(350);

    connect(m_deviceTypeWidget, &InputWidget::signalComboBoxIndexChanged, this, [=] {
        QString deviceTypeId = m_deviceTypeMap.key(m_deviceTypeWidget->getComboBoxCurrentContent());
        // �豸����;
        m_deviceChildTypeMap.clear();
        std::map<QString, QString> deviceChildTypeMap = WHEEL_DEVICE_CONFIG.getWheelSubDeviceNameFromDeviceType(deviceTypeId);
        QStringList deviceChildTypeList;
        std::map<QString, QString>::iterator deviceChildTypeIter;
        for (deviceChildTypeIter = deviceChildTypeMap.begin(); deviceChildTypeIter != deviceChildTypeMap.end(); deviceChildTypeIter++)
        {
            m_deviceChildTypeMap.insert(deviceChildTypeIter->first, deviceChildTypeIter->second);
            deviceChildTypeList.append(deviceChildTypeIter->second);
        }
        m_deviceSubTypeWidget->setComboBoxContent(deviceChildTypeList);
    });

    m_deviceSubTypeWidget = new InputWidget(InputWidgetType::WheelComboBox);
    m_deviceSubTypeWidget->setTipText("�豸����");
    m_deviceSubTypeWidget->setComboBoxWidth(270);
    m_deviceSubTypeWidget->setFixedWidth(350);

    connect(m_deviceSubTypeWidget, &InputWidget::signalComboBoxIndexChanged, this, [=] {
        QString deviceChildTypeId = m_deviceChildTypeMap.key(m_deviceSubTypeWidget->getComboBoxCurrentContent());
        m_devicePointPosMap.clear();
        std::map<QString, WheelDevicePointNameStruct> devicePointPosMap = WHEEL_DEVICE_CONFIG.getWheelDevicePointNameFromSubDeviceType(deviceChildTypeId);
        QStringList devicePointPosList;
        std::map<QString, WheelDevicePointNameStruct>::iterator devicePointPosIter;
        for (devicePointPosIter = devicePointPosMap.begin(); devicePointPosIter != devicePointPosMap.end(); devicePointPosIter++)
        {
            m_devicePointPosMap.insert(devicePointPosIter->first, devicePointPosIter->second);
            devicePointPosList.append(devicePointPosIter->second.device_point_type_name);
        }
        m_pointNameWidget->setComboBoxContent(devicePointPosList);
    });

    m_pointNameWidget = new InputWidget(InputWidgetType::WheelComboBox);
    m_pointNameWidget->setTipText("��λ����");
    m_pointNameWidget->setComboBoxWidth(270);
    m_pointNameWidget->setFixedWidth(350);

    m_pButtonCommit = new QToolButton;
    m_pButtonCommit->setText("�ύ");
    m_pButtonCommit->setFixedSize(QSize(70, 25));
    
    connect(m_pButtonCommit, &QToolButton::clicked, this, [=] {
        WheelPatrolPointSet data;
        if (m_isModifyData)
        {
            data.device_uuid = m_windowData.device_uuid;
        }
        else
        {
            data.device_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
        }
        
        data.station_name = m_stationNameWidget->getComboBoxCurrentContent();
        data.voltahe_level_name = m_voltageLevelWidget->getComboBoxCurrentContent();
        data.equipment_interval_name = m_intervalNameWidget->getComboBoxCurrentContent();
        data.device_area_name = m_deviceAreaWidget->getComboBoxCurrentContent();
        data.device_point_type_name = m_pointNameWidget->getComboBoxCurrentContent();

        if (data.voltahe_level_name.isEmpty() || data.equipment_interval_name.isEmpty() || data.device_area_name.isEmpty())
        {
            DLMessageBox::showDLMessageBox(NULL, "��ʾ", "������Ϊ�յ��ֶ�", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return;
        }
        emit signalCommitButtonClicked(data);
    });

    QHBoxLayout* hCommitButtonLayout = new QHBoxLayout;
    hCommitButtonLayout->addStretch();
    hCommitButtonLayout->addWidget(m_pButtonCommit);
    hCommitButtonLayout->addStretch();
    hCommitButtonLayout->setContentsMargins(0, 30, 0, 0);

    QVBoxLayout* vInputWidgetLayout = new QVBoxLayout;
    vInputWidgetLayout->addWidget(m_stationNameWidget);
    vInputWidgetLayout->addWidget(m_voltageLevelWidget);
    vInputWidgetLayout->addWidget(m_intervalNameWidget);
    vInputWidgetLayout->addWidget(m_deviceAreaWidget);
    vInputWidgetLayout->addWidget(m_deviceTypeWidget);
    vInputWidgetLayout->addWidget(m_deviceSubTypeWidget);
    vInputWidgetLayout->addWidget(m_pointNameWidget);
    vInputWidgetLayout->addLayout(hCommitButtonLayout);
    vInputWidgetLayout->setSpacing(15);

    QHBoxLayout* hInputWidgetLayout = new QHBoxLayout;
    hInputWidgetLayout->addStretch();
    hInputWidgetLayout->addLayout(vInputWidgetLayout);
    hInputWidgetLayout->addStretch();
    hInputWidgetLayout->setMargin(0);

    QVBoxLayout* vCenterLayout = new QVBoxLayout(m_centerWidget);
    vCenterLayout->addWidget(m_titleLabel);
    vCenterLayout->addLayout(hInputWidgetLayout);
    vCenterLayout->addSpacing(30);
    vCenterLayout->addLayout(hCommitButtonLayout);
    vCenterLayout->setSpacing(10);
    vCenterLayout->setContentsMargins(2, 2, 2, 20);
}

void PointPosSetAddWindow::initComboBoxData()
{
    // վ����;
    QString strStationName = "";
    std::map<int, WheelStationConfigStruct> stationNameData = WHEEL_STATION_CONFIG.getWheelStationConfigData();
    QStringList stationNameList;
    std::map<int, WheelStationConfigStruct>::iterator stationNameIter;
    for (stationNameIter = stationNameData.begin(); stationNameIter != stationNameData.end(); stationNameIter++)
    {
        m_stationNameMap.insert(stationNameIter->first, stationNameIter->second);
        stationNameList.append(stationNameIter->second.station_name);
    }
    m_stationNameWidget->setComboBoxContent(stationNameList);

    // ��ѹ�ȼ�;
    m_voltageLevelMap.clear();
    std::map<QString, QString> voltageLevelMap = WHEEL_DEVICE_CONFIG.getWheelVoltageLevelDataMap();
    QStringList voltageLevelList;
    std::map<QString, QString>::iterator voltageLevelIter;
    for (voltageLevelIter = voltageLevelMap.begin(); voltageLevelIter != voltageLevelMap.end(); voltageLevelIter++)
    {
        m_voltageLevelMap.insert(voltageLevelIter->first, voltageLevelIter->second);
        voltageLevelList.append(voltageLevelIter->second);
    }
    m_voltageLevelWidget->setComboBoxContent(voltageLevelList);

    // �������;
    m_intervalNameMap.clear();
    if (voltageLevelList.size())
    {
        std::map<QString, QString> intervalNameMap = WHEEL_DEVICE_CONFIG.getWheelRobortEquipmentIntervalFromVoltageLevelId(m_voltageLevelMap.key(voltageLevelList.at(0)));
        QStringList intervalNameList;
        std::map<QString, QString>::iterator intervalNameIter;
        for (intervalNameIter = intervalNameMap.begin(); intervalNameIter != intervalNameMap.end(); intervalNameIter++)
        {
            m_intervalNameMap.insert(intervalNameIter->first, intervalNameIter->second);
            intervalNameList.append(intervalNameIter->second);
        }
        m_intervalNameWidget->setComboBoxContent(intervalNameList);
    }
    else
    {
        m_intervalNameWidget->setComboBoxContent(QStringList() << "");
    }

    // �豸����;
    m_areaNameMap.clear();
    std::map<QString, QString> areaNameMap = WHEEL_DEVICE_CONFIG.getWheelDeviceAreaDataMap();
    QStringList areaNameList;
    std::map<QString, QString>::iterator areaNameIter;
    for (areaNameIter = areaNameMap.begin(); areaNameIter != areaNameMap.end(); areaNameIter++)
    {
        m_areaNameMap.insert(areaNameIter->first, areaNameIter->second);
        areaNameList.append(areaNameIter->second);
    }
    m_deviceAreaWidget->setComboBoxContent(areaNameList);
    
    // �豸����;
    m_deviceTypeMap.clear();
    std::map<QString, QString> deviceTypeMap = WHEEL_DEVICE_CONFIG.getWheelDeviceTypeDataMap();
    QStringList deviceTypeList;
    std::map<QString, QString>::iterator deviceTypeIter;
    for (deviceTypeIter = deviceTypeMap.begin(); deviceTypeIter != deviceTypeMap.end(); deviceTypeIter++)
    {
        m_deviceTypeMap.insert(deviceTypeIter->first, deviceTypeIter->second);
        deviceTypeList.append(deviceTypeIter->second);
    }
    m_deviceTypeWidget->setComboBoxContent(deviceTypeList);
  
    // �豸����;
    m_deviceChildTypeMap.clear();
    if (deviceTypeList.size())
    {
        std::map<QString, QString> deviceChildTypeMap = WHEEL_DEVICE_CONFIG.getWheelSubDeviceNameFromDeviceType(m_deviceTypeMap.key(deviceTypeList.at(0)));
        QStringList deviceChildTypeList;
        std::map<QString, QString>::iterator deviceChildTypeIter;
        for (deviceChildTypeIter = deviceChildTypeMap.begin(); deviceChildTypeIter != deviceChildTypeMap.end(); deviceChildTypeIter++)
        {
            m_deviceChildTypeMap.insert(deviceChildTypeIter->first, deviceChildTypeIter->second);
            deviceChildTypeList.append(deviceChildTypeIter->second);
        }
        m_deviceSubTypeWidget->setComboBoxContent(deviceChildTypeList);
       

        // �豸��λ;
        m_devicePointPosMap.clear();
        if (deviceChildTypeList.size())
        {
            std::map<QString, WheelDevicePointNameStruct> devicePointPosMap = WHEEL_DEVICE_CONFIG.getWheelDevicePointNameFromSubDeviceType(m_deviceChildTypeMap.key(deviceChildTypeList.at(0)));
            QStringList devicePointPosList;
            std::map<QString, WheelDevicePointNameStruct>::iterator devicePointPosIter;
            for (devicePointPosIter = devicePointPosMap.begin(); devicePointPosIter != devicePointPosMap.end(); devicePointPosIter++)
            {
                m_devicePointPosMap.insert(devicePointPosIter->first, devicePointPosIter->second);
                devicePointPosList.append(devicePointPosIter->second.device_point_type_name);
            }
            m_pointNameWidget->setComboBoxContent(devicePointPosList);
        }
    }
}

void PointPosSetAddWindow::initWidget()
{
    initNewCenterWidget();
    initComboBoxData();

	QIcon icon = style()->standardIcon(QStyle::SP_TitleBarCloseButton);
	m_pButtonClose = new QToolButton;
	m_pButtonClose->setIcon(icon);
	m_pButtonClose->setIconSize(QSize(16, 16));
	m_pButtonClose->setStyleSheet("border:1px solid rgb(159,168,218);border-radius:3px;background:rgb(220, 240, 230);");
	m_pButtonClose->setFixedSize(QSize(18, 18));
	connect(m_pButtonClose, &QToolButton::clicked, this, [=] {
		this->close();
	});

	QHBoxLayout* hTopLayout = new QHBoxLayout;
	hTopLayout->addStretch();
	hTopLayout->addWidget(m_pButtonClose);
	hTopLayout->setMargin(0);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addLayout(hTopLayout);
	vMainLayout->addWidget(m_centerWidget);
	vMainLayout->setSpacing(10);
	vMainLayout->setMargin(10);
}

void PointPosSetAddWindow::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), QColor(175, 191, 255));
	painter.setPen(QColor(166, 233, 210));
	painter.drawRect(QRect(0, 0, this->width() - 1, this->height() - 1));
}


DLWheelPatrolPointPosSet::DLWheelPatrolPointPosSet(QWidget* parent /* = NULL */)
	: QWidget(parent)
	, m_isInitWidget(false)
    , m_currentPageIndex(1)
{
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotDeletePatrolPointSetStatus.connect(boost::bind(&DLWheelPatrolPointPosSet::signalDeletePatrolPointCallBack, this, _1, _2));
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotStartUsingStatus.connect(boost::bind(&DLWheelPatrolPointPosSet::signalIsStartUsingPointCallBack, this, _1, _2));

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotPatrolPointAddStatus.connect(boost::bind(&DLWheelPatrolPointPosSet::signalAddPatrolPointCallBack, this, _1, _2));
    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotPatrolPointUpdataStatus.connect(boost::bind(&DLWheelPatrolPointPosSet::signalUpdatePatrolPointCallBack, this, _1, _2));
    
    
    connect(this, &DLWheelPatrolPointPosSet::signalDeletePatrolPointCallBack, this, [=](bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            initTableData();
            DLMessageBox::showDLMessageBox(NULL, "��ʾ", "ɾ���ɹ�", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "����", "ɾ��ʧ��:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });

    connect(this, &DLWheelPatrolPointPosSet::signalIsStartUsingPointCallBack, this, [=](bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            initTableData();
            DLMessageBox::showDLMessageBox(NULL, "��ʾ", "�����ɹ�", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "����", "����ʧ��:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });
    
    connect(this, &DLWheelPatrolPointPosSet::signalAddPatrolPointCallBack, this, [=](bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            refreshComboBoxData();
            initTableData();
            DLMessageBox::showDLMessageBox(NULL, "��ʾ", "��ӳɹ�", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "����", "���ʧ��:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });

    connect(this, &DLWheelPatrolPointPosSet::signalUpdatePatrolPointCallBack, this, [=](bool isSuccess, QString strMsg) {
        if (isSuccess)
        {
            refreshComboBoxData();
            initTableData();
            DLMessageBox::showDLMessageBox(NULL, "��ʾ", "�޸ĳɹ�", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "����", "�޸�ʧ��:" + strMsg, MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
    });
}

void DLWheelPatrolPointPosSet::initButtonWidget()
{
	m_buttonBackWidget = new CustomButtonListWidget;
	connect(m_buttonBackWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelPatrolPointPosSet::onButtonClicked);
	m_buttonBackWidget->addToolButton(0, "��ѯ", ":/Resources/Common/image/Search.png");
	m_buttonBackWidget->addToolButton(1, "���", ":/Resources/Common/image/Button_Add.png");
	m_buttonBackWidget->addToolButton(2, "�޸�", ":/Resources/Common/image/ModifyButton.png");
	m_buttonBackWidget->addToolButton(3, "ɾ��", ":/Resources/Common/image/Button_Delete.png");
	m_buttonBackWidget->addToolButton(4, "����", ":/Resources/Common/image/EnabelButton.png");
	m_buttonBackWidget->addToolButton(5, "����", ":/Resources/Common/image/ForbidButton.png");
	m_buttonBackWidget->addToolButton(6, "����", ":/Resources/Common/image/Reset.png");
	m_buttonBackWidget->addToolButton(7, "����", ":/Resources/Common/image/ExportButton.png");

	m_buttonBackWidget->addWidgetFinished();
}

void DLWheelPatrolPointPosSet::initTopBoxWidget()
{
    initCheckBoxWidget();
	initButtonWidget();

	m_topBoxBackWidget = new QWidget;
	m_topBoxBackWidget->setObjectName("TopBackWidget");

	// ����״̬;
	m_enableStateComboBox = new InputWidget(InputWidgetType::WheelComboBox);
	m_enableStateComboBox->setTipText("����״̬");
	m_enableStateComboBox->setFixedWidth(220);
    m_enableStateComboBox->setComboBoxContent(QStringList() << "����" << "����");

	QHBoxLayout* hComboBoxLayout = new QHBoxLayout;
	hComboBoxLayout->addWidget(m_enableStateComboBox);
	hComboBoxLayout->addStretch();

	QVBoxLayout* vTopLayout = new QVBoxLayout(m_topBoxBackWidget);
	vTopLayout->addWidget(m_checkBoxBackWidget);
	vTopLayout->addLayout(hComboBoxLayout);
	vTopLayout->addWidget(m_buttonBackWidget);
	vTopLayout->setSpacing(20);
	vTopLayout->setMargin(6);
}

void DLWheelPatrolPointPosSet::initCheckBoxWidget()
{
    m_checkBoxBackWidget = new QWidget;

    QList<WheelTaskAdminCheckBoxStruct> checkBoxDataList = WHEEL_TASK_ADMINISTRATION.getTaskAdminCheckBoxData(WHEEL_PATROL_POINT_SET);
    QVBoxLayout* vCheckBoxLayout = new QVBoxLayout(m_checkBoxBackWidget);
    for (int i = 0; i < checkBoxDataList.count(); i++)
    {
        CheckBoxWidget* checkBoxWidget = new CheckBoxWidget;
        checkBoxWidget->setCheckBoxType(checkBoxDataList[i].CheckBoxTypeEnum);
        checkBoxWidget->addCheckBoxWidget(checkBoxDataList[i].typeName, checkBoxDataList[i].PackageCheckBoxStru);

        m_checkBoxWidgetList.append(checkBoxWidget);
        vCheckBoxLayout->addWidget(checkBoxWidget);
    }
    vCheckBoxLayout->setSpacing(20);
    vCheckBoxLayout->setMargin(0);
}

void DLWheelPatrolPointPosSet::initDeviceTreeWidget()
{
	m_deviceTreeWidget = new CustomTreeWidget;
	m_deviceTreeWidget->setTreeWidgetType(TreeItemWidgetType::FolderRect);
	m_deviceTreeWidget->refreshTreeItemList();
}

void DLWheelPatrolPointPosSet::initTableWidget()
{
	m_customTableWidget = new CustomTableWidget(11);
    m_customTableWidget->setHorizontalHeaderLabels(QStringList() << "���" << "���վ����" << "��ѹ�ȼ�" << "�������" << "�豸����" << "�豸����" << "�豸����" << "��λ����" << "ʶ������" << "�������"  << "����״̬");
    
    connect(m_customTableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableData();
    });
    connect(m_customTableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableData();
    });
}

void DLWheelPatrolPointPosSet::initTableData()
{
    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
    m_customTableWidget->setHeaderCheckBoxState(false);

    m_tableDataList = WHEEL_DEVICE_CONFIG.getPatrolPointSetData(m_currentPageIndex, TABLE_PER_PAGE_COUNT, m_wheelPatrolPara);
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    int totalPage, totalCount;
    WHEEL_DEVICE_CONFIG.getPatrolPointSetDataCount(totalPage, totalCount, TABLE_PER_PAGE_COUNT, m_wheelPatrolPara);
    m_customTableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

    // վ����;
    QString strStationName = "";
    std::map<int, WheelStationConfigStruct> data =  WHEEL_STATION_CONFIG.getWheelStationConfigData();
    if (data.size() > 0)
    {
        strStationName = data[1].station_name;
    }
    
    for (int i = 0; i < m_tableDataList.count(); i++)
    {
        tableWidget->insertRow(i);
        QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
        checkBoxItem->setCheckState(Qt::Unchecked);
        checkBoxItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, checkBoxItem);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 1, idItem);

        QTableWidgetItem* stationNameItem = new QTableWidgetItem(strStationName);
        tableWidget->setItem(i, 2, stationNameItem);
        stationNameItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* voltageLevelItem = new QTableWidgetItem(m_tableDataList[i].voltahe_level_name);
        voltageLevelItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, voltageLevelItem);

        QTableWidgetItem* intervalNameItem = new QTableWidgetItem(m_tableDataList[i].equipment_interval_name);
        intervalNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, intervalNameItem);

        QTableWidgetItem* deviceAreaItem = new QTableWidgetItem(m_tableDataList[i].device_area_name);
        deviceAreaItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, deviceAreaItem);
       
        QTableWidgetItem* deviceTypeItem = new QTableWidgetItem(m_tableDataList[i].device_type_name);
        deviceTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 6, deviceTypeItem);

        QTableWidgetItem* pointPosTagItem = new QTableWidgetItem(m_tableDataList[i].sub_device_type);
        pointPosTagItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 7, pointPosTagItem);

        QTableWidgetItem* pointPosNameItem = new QTableWidgetItem(m_tableDataList[i].device_point_type_name);
        pointPosNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 8, pointPosNameItem);

        QTableWidgetItem* recognitionTypeItem = new QTableWidgetItem(m_tableDataList[i].recognition_type_name);
        recognitionTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 9, recognitionTypeItem);

        QTableWidgetItem* meterTypeNameItem = new QTableWidgetItem(m_tableDataList[i].meter_type_name);
        meterTypeNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 10, meterTypeNameItem);

        QTableWidgetItem* startUseStateItem = new QTableWidgetItem(m_tableDataList[i].start_using);
        startUseStateItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 11, startUseStateItem);

        tableWidget->setRowHeight(i, 40);
    }
}

QStringList DLWheelPatrolPointPosSet::getChoosedDeviceIdList()
{
    QStringList strDeviceIdList;
    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
    for (int i = 0; i < tableWidget->rowCount(); i++)
    {
        QTableWidgetItem* checkItem = tableWidget->item(i, 0);
        if (checkItem != NULL && checkItem->checkState() == Qt::Checked)
        {
            strDeviceIdList.append(m_tableDataList[i].device_uuid);
        }
    }

    return strDeviceIdList;
}

void DLWheelPatrolPointPosSet::deleteChoosedDevice()
{
    QStringList strDeviceIdList = getChoosedDeviceIdList();
    
    if (strDeviceIdList.isEmpty())
    {
        DLMessageBox::showDLMessageBox(NULL, "��ʾ", "��ǰδѡ���κ���", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
    }
    else
    {
        WHEEL_BACK_TO_CORE_SOCKET.robot_delete_patrol_point_set_req(strDeviceIdList);
    }    
}

void DLWheelPatrolPointPosSet::isStartUseDevice(WheelRootStartUsing start_using)
{
    QStringList strDeviceIdList = getChoosedDeviceIdList();

    if (strDeviceIdList.isEmpty())
    {
        DLMessageBox::showDLMessageBox(NULL, "��ʾ", "��ǰδѡ���κ���", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
    }
    else
    {
        WHEEL_BACK_TO_CORE_SOCKET.robot_start_using_status_req(strDeviceIdList, start_using);
    }
}

void DLWheelPatrolPointPosSet::updatePointPos()
{
    QTableWidgetItem* item = m_customTableWidget->getTableWidget()->currentItem();
    if ( item != NULL)
    {
        int row = item->row();
        WheelPatrolPointSet data = m_tableDataList[row];

        PointPosSetAddWindow* pointPosSetWindow = new PointPosSetAddWindow;
        pointPosSetWindow->setData(data);
        connect(pointPosSetWindow, &PointPosSetAddWindow::signalCommitButtonClicked, this, [=](WheelPatrolPointSet data) {
            pointPosSetWindow->hide();
            WHEEL_BACK_TO_CORE_SOCKET.robot_patrol_point_updata_req(data);
        });
        pointPosSetWindow->show();
    }
    else
    {
        DLMessageBox::showDLMessageBox(NULL, "��ʾ", "���ڱ����ѡ��һ���豸", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
    } 
}

void DLWheelPatrolPointPosSet::refreshComboBoxData()
{
    WHEEL_DEVICE_CONFIG.loadWheelVoltageLevelData();
    WHEEL_DEVICE_CONFIG.loadWheelEquipmentIntervalData();
    WHEEL_DEVICE_CONFIG.loadWheelDeviceAreaData();
}

void DLWheelPatrolPointPosSet::searchButtonClicked()
{
    WheelPatrolParameter wheelPatrolPara;

    for (int i = 0; i < m_checkBoxWidgetList.count(); i++)
    {
        QStringList strCheckedIdList = m_checkBoxWidgetList[i]->getCheckedIdList();
        WheelCheckBoxTypeEnum currentType = m_checkBoxWidgetList[i]->getCheckBoxType();
        switch (currentType)
        {
        case WHEEL_DEVICE_AREA_NAME:
            m_wheelPatrolPara.m_device_area_uuid = strCheckedIdList;
            break;
        case WHEEL_DEVICE_TYPE:
            m_wheelPatrolPara.m_device_type_uuid = strCheckedIdList;
            break;
        case WHEEL_RECOGNITION_TYPE:
            m_wheelPatrolPara.m_recognition_type_uuid = strCheckedIdList;
            break;
        case WHEEL_METER_TYPE:
            m_wheelPatrolPara.m_meter_type_id = strCheckedIdList;
            break;
        // case WHEEL_DEVICE_APPEARANCE_TYPE:
        //	wheelPatrolPara.m_device_area_uuid = strCheckedIdList;
        //	break;
        default:
            break;
        }
    }

    wheelPatrolPara.m_startStatus = m_enableStateComboBox->getComboBoxCurrentContent();

    initTableData();
}

void DLWheelPatrolPointPosSet::exportTable()
{
    QList<QStringList> excelData;
    excelData.append(QStringList() << "���վ����" << "��ѹ�ȼ�" << "�������" << "�豸����" << "�豸����" << "�豸����" << "��λ����" << "ʶ������" << "�������" << "����״̬");

    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
    for (int i = 0; i < tableWidget->rowCount(); i++)
    {
        QStringList strDataList;
        if (tableWidget->item(i, 0)->checkState() == Qt::Checked)
        {
            for (int j = 2; j < tableWidget->columnCount(); j++)
            {
                strDataList.append(tableWidget->item(i, j)->text());
            }

            excelData.append(strDataList);
        }
    }

    if (excelData.count() == 1)
    {
        DLMessageBox::showDLMessageBox(NULL, "��ʾ", "δѡ�񵼳���", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        return;
    }

    QString dir = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("excel(*.xlsx)"));
    if (dir.isEmpty())
    {
        return;
    }

    SearchRecordCreateExcel excel;
    excel.CreateNewExcelForList(excelData, dir, EXCEL_PATROL_POINT_SET);
}

void DLWheelPatrolPointPosSet::resetButtonClicked()
{
    for (int i = 0; i < m_checkBoxWidgetList.count(); i++)
    {
        m_checkBoxWidgetList[i]->resetCheckBox();
    }
}

void DLWheelPatrolPointPosSet::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initTopBoxWidget();
	initDeviceTreeWidget();
	initTableWidget();
    initTableData();

	QHBoxLayout* hCenterLayout = new QHBoxLayout;
	hCenterLayout->addWidget(m_deviceTreeWidget);
	hCenterLayout->addWidget(m_customTableWidget);
	hCenterLayout->setSpacing(5);
	hCenterLayout->setMargin(0);

	QVBoxLayout* vLayout = new QVBoxLayout(this);
	vLayout->addWidget(m_topBoxBackWidget);
	vLayout->addLayout(hCenterLayout);
	vLayout->setSpacing(0);
	vLayout->setMargin(2);
}

void DLWheelPatrolPointPosSet::onButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
    {
        searchButtonClicked();
    }
        break;
    case 1:
    {
        PointPosSetAddWindow* pointPosSetWindow = new PointPosSetAddWindow;
        connect(pointPosSetWindow, &PointPosSetAddWindow::signalCommitButtonClicked, this, [=](WheelPatrolPointSet data) {
            pointPosSetWindow->hide();
            WHEEL_BACK_TO_CORE_SOCKET.robot_patrol_point_add_req(data);
        });
        pointPosSetWindow->show();
    }
        break;
    case 2:
    {
        updatePointPos();
    }
        break;
    case 3:
    {
        deleteChoosedDevice();
    }
        break;
    case 4:
    {
        isStartUseDevice(open_using);
    }
        break;
    case 5:
    {
        isStartUseDevice(forbid_using);
    }
        break;
    case 6:
    {
        resetButtonClicked();
    }
        break;
    case 7:
    {
        exportTable();
    }
        break;
    default:
        break;
    }
}