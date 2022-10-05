#include "DLWheelTaskCheck.h"
#include <QHBoxLayout>
#include <QStyle>
#include <QPainter>
#include <QButtonGroup>
#include <QUuid>
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include "RCFImageDownload.h"
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include "LibDLWheelRobotConfigData/DLWheelRobotCoreConfig.h"

#pragma execution_character_set("utf-8")

#define TITLE_WIDGET_HEIGHT 150

DLWheelTaskCheck::DLWheelTaskCheck(QWidget* parent)
    : QWidget(parent)
    , m_isSingleTask(true)
    , m_taskCheckDataIndex(0)
    , m_taskCheckDataCount(0)
    , m_currentCheckedIndex(0)
    , m_RCFImageDownload(NULL)
{
	initWidget();
	this->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
	this->setFixedSize(QSize(1000, 750));
	this->setWindowModality(Qt::ApplicationModal);

	this->setStyleSheet("QWidget#TitleWidget{border:1px solid gray;}\
							QWidget#LastPageButtonBackWidget,QWidget#NextPageBackWidget\
							{background:rgb(220, 235, 235);border:1px solid rgba(63,81,181, 150);border-bottom:none;}\
							QWidget#RightBackWidget{background:white;}\
							QWidget#CenterWidget{background:white;border:1px solid rgba(63,81,181, 150);border-top:none;}\
                            QWidget#TopBackWidget{background:transparent;}\
                            QComboBox{background:white;border:1px solid gray;}");

    WHEEL_BACK_TO_CORE_SOCKET.wheelRobotPartrolResultVerifyStatus.connect(boost::bind(&DLWheelTaskCheck::signalSaveTaskCheckData, this, _1, _2, _3));
    
    connect(this, &DLWheelTaskCheck::signalSaveTaskCheckData, this, [=](int checkType, bool isSuccess, QString strMsg) {
        // 如果保存成功，关闭窗口，保存失败提示;
        TaskCheckType sendCheckType = TaskCheckType(checkType);
        if (m_taskCheckType != sendCheckType)
        {
            return;
        }
        m_saveTimeoutTimer.stop();
        if (isSuccess)
        {
            if (m_isSingleTask)
            {
                this->hide();
                emit signalRefreshTable();
            }
            else
            {
                m_pButtonNextPage->setDisabled(false);
                if (m_taskCheckDataIndex > m_currentCheckedIndex)
                {
                    m_currentCheckedIndex = m_taskCheckDataIndex;
                }
                m_dataList[m_taskCheckDataIndex] = m_data;
            }
        }
        else
        {
            DLMessageBox::showDLMessageBox(NULL, "错误", "保存失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        }
        this->setDisabled(false);
        m_pButtonSave->setText("保存");
    });
}

DLWheelTaskCheck::~DLWheelTaskCheck()
{
    if (m_RCFImageDownload != NULL)
    {
        m_RCFImageDownload->stopDownloadImage();
    }
}

void DLWheelTaskCheck::setData(DeviceAlarmSearchStruct data)
{
    m_data = data;

    m_resultShow->setText(data.inspect_result);

    m_pointInfoLineEdit->setText(m_data.device_point_type_name);
    m_timeLabel->setText(m_data.inspect_time);
    m_timeLabel->setScaledContents(true);

    m_alarmTypeLabel->setText(m_data.save_type_name);
    m_alarmTypeLabel->setScaledContents(true);

    m_visibleImageWidget->setImage("");
    m_fraredImageWidget->setImage("");

	//QFile file("d:/1.txt");
	//if (!file.open(QIODevice::ReadOnly | QIODevice::Text))//打开指定文件
	//{
	//}

	//QString _task_uuid;
	//QString _device_uuid;

	//QTextStream txtInput(&file);
	//QString lineStr;
	//int index = 0;
	//while (!txtInput.atEnd())
	//{
	//	if (index == 0)
	//		_task_uuid = txtInput.readLine();  //读取数据

	//	if (index == 1) {
	//		_device_uuid = txtInput.readLine();  //读取数据
	//		break;
	//	}
	//	index++;
	//}

	//file.close();

	//QString rootPath = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath;
	//QString imgPath = rootPath + "/task/" + _task_uuid + "/" + _device_uuid + "/num_target_" + _device_uuid + ".jpg";

	//QString rawImgPath = rootPath + "/task/" + _task_uuid + "/" + _device_uuid + "/" + _device_uuid + ".jpg";

	QString rootPath = WHEEL_ROBOT_CORE_CONFIG.getCfg().rootPath;
	QString imgPath = rootPath + "/task/" + data.task_uuid + "/" + data.device_uuid + "/num_target_" + data.device_uuid + ".jpg";

	QString rawImgPath = rootPath + "/task/" + data.task_uuid + "/" + data.device_uuid + "/" + data.device_uuid + ".jpg";

	ROS_ERROR("imgPath=======%s", imgPath.toStdString().c_str());
	ROS_ERROR("rawImgPath=======%s", rawImgPath.toStdString().c_str());

	QFile _file;
	if (_file.exists(imgPath))
	{
		m_visibleImageWidget->setImage(imgPath);
		ROS_ERROR("DLWheelTaskCheck=======%s", rawImgPath.toStdString().c_str());
	}
	else {
		m_visibleImageWidget->setImage(rawImgPath);
	}

    // 如果table进行了刷新，释放之前下载图片的线程，重新开辟线程进行新的下载;
    //if (m_RCFImageDownload != NULL)
    //{
    //    m_RCFImageDownload->stopDownloadImage();
    //    disconnect(m_RCFImageDownload, &RCFImageDownload::signalDowmloadFinished, this, &DLWheelTaskCheck::onRcfDownloadImageFinished);
    //    m_RCFImageDownload->deleteLater();
    //}

    //m_RCFImageDownload = new RCFImageDownload(this);
    //connect(m_RCFImageDownload, &RCFImageDownload::signalDowmloadFinished, this, &DLWheelTaskCheck::onRcfDownloadImageFinished);
    //m_RCFImageDownload->setImageInfoList(QStringList() << m_data.task_uuid, QStringList() << m_data.device_uuid);
    //m_RCFImageDownload->start();

    // 设置告警阈值;
//     QString threshold_filename;
//     bool iDBRet;
//     WheelRobotMeterType meterTypeId = WHEEL_DEVICE_CONFIG.getMeterEnumWithDeviceUUid(data.device_uuid);
//     iDBRet = WHEEL_ROBOT_DB.getThresholdFileName(data.device_uuid, meterTypeId, threshold_filename);
//     if (iDBRet)
//     {
//         m_alarmThresholdLabel;
//     }

    // 当前任务是否已经审核过;
    if (m_data.is_dealed)
    {
        if (m_data.deal_status_id == NORMAL_STATUS)
        {
            m_checkBoxNormal->setChecked(true);
            m_identifyComboBox->setDisabled(true);
        }
        else
        {
            m_checkBoxAbnormal->setChecked(true);
            m_identifyComboBox->setDisabled(false);
        }
        
        if (m_data.deal_result.isEmpty())
        {
            m_identifyRight->setChecked(true);
            m_realValueLineEdit->clear();
            m_realValueLineEdit->setDisabled(true);
        }
        else
        {
            m_identifyError->setChecked(true);
            m_realValueLineEdit->setText(m_data.deal_result);
            m_realValueLineEdit->setDisabled(false);
        }

        m_identifyComboBox->setCurrentText(m_data.deal_cause);
        m_alarmLavelComBoBox->setCurrentText(m_data.alarm_level_name);

        m_pButtonNextPage->setDisabled(false);
    }
    else
    {
        m_checkBoxNormal->setChecked(true);
        m_identifyComboBox->setDisabled(true);

        m_identifyRight->setChecked(true);
        m_realValueLineEdit->setDisabled(true);

        m_identifyComboBox->setCurrentIndex(0);
        //m_alarmLavelComBoBox->setCurrentIndex(0);
        m_alarmLavelComBoBox->setCurrentText(m_data.alarm_level_name);
        m_realValueLineEdit->clear();
        m_pButtonNextPage->setDisabled(true);
    }

    int alarmFrequency = WHEEL_PATROL_RESULT.alarmCountSearch(m_data.deal_uuid);
    m_alarmFrequencyLabel->setText(QString::number(alarmFrequency));

    thresholdAnalysis(data.device_uuid);
}

void DLWheelTaskCheck::thresholdAnalysis(QString deviceUUid)
{
    QStringList jsonValue;
    WHEEL_ROBOT_DB.getThresholdValue(deviceUUid, jsonValue);
    if (jsonValue.size() == 0 || jsonValue.size() < 5)
    {
        return;
    }

    QList<QStringList> tableValue;
    for (int iJson = 0; iJson < jsonValue.size(); iJson++)
    {
        QStringList strList;
        QJsonParseError parseJsonErr;
        QJsonDocument document = QJsonDocument::fromJson(jsonValue[iJson].toUtf8(), &parseJsonErr);
        if (!(parseJsonErr.error == QJsonParseError::NoError))
        {
            return;
        }

        QJsonArray jsonArray = QJsonValue(document.array()).toArray();
        for (int i = 0; i < jsonArray.size(); i++)
        {
            QJsonObject valBody = QJsonValue(jsonArray.at(i)).toObject();
            int type = valBody["type"].toInt();
            if (type == 22 || type == 5 || type == 6 || type == 12 || type == 13)
            {
                QJsonValue valNext = QJsonArray(QJsonValue(valBody.value("val")).toArray()).at(0);
                QString value = valNext.toString();
                strList.append(value);
            }

            if (type == 11)
            {
                strList.append("");
                strList.append("不等于");
            }
            if (type == 14)
            {
                strList.append("");
                strList.append("且");
            }
            if (type == 15)
            {
                strList.append("");
                strList.append("或");
            }

//             if (type == 11 || type == 14 || type == 15)
//             {
//                 QJsonValue valNext = QJsonArray(QJsonValue(valBody.value("val")).toArray()).at(0);
//                 QString value = valNext.toString();
//                 strList.append("");
//                 strList.append(value);
//             }
        }

        QStringList listJson;
        QString showString;
        if (strList.size() % 2 != 0)
        {
            return;
        }

        for (int i = 0; i < strList.size(); i++)
        {
            showString = showString + QString(" %1%2").arg(strList[i]).arg(strList[i + 1]);
            i++;
        }
        switch (iJson)
        {
        case 0:
            listJson.append("正常");
            break;
        case 1:
            listJson.append("预警");
            break;
        case 2:
            listJson.append("一般告警");
            break;
        case 3:
            listJson.append("严重告警");
            break;
        case 4:
            listJson.append("危急告警");
            break;
        default:
            break;
        }
        listJson.append(showString);
        tableValue.append(listJson);
    }
    m_ThresholdValueWidget->setTableValue(tableValue);
    //m_alarmThresholdLabel->setText(showString);
}

void DLWheelTaskCheck::setDataList(QList<DeviceAlarmSearchStruct> dataList)
{
    setIsSingleTask(false);
    m_taskCheckDataIndex = 0;
    m_dataList = dataList;
    setData(m_dataList.first());
    m_taskCheckDataCount = m_dataList.count();
}

void DLWheelTaskCheck::setIsSingleTask(bool isSingleTask)
{
    m_isSingleTask = isSingleTask;

    if (isSingleTask)
    {
        m_currentTaskCheckIndexLabel->setVisible(false);
        m_pButtonLastPage->setVisible(false);
        m_pButtonNextPage->setVisible(false);
    }
    else
    {
        m_currentTaskCheckIndexLabel->setVisible(true);
        m_pButtonLastPage->setVisible(true);
        m_pButtonNextPage->setVisible(true);
        m_pButtonLastPage->setDisabled(true);
        m_pButtonNextPage->setDisabled(true);
    }
}

void DLWheelTaskCheck::setTaskCheckType(TaskCheckType taskCheckType)
{
    m_taskCheckType = taskCheckType;
}

void DLWheelTaskCheck::onSaveTaskCheckData()
{
    // 结果;
    if (m_checkBoxNormal->isChecked())
    {
        m_data.deal_status_id = NORMAL_STATUS;
        m_data.deal_cause = "";
    }
    else
    {
        m_data.deal_status_id = UNUSUAL_STATUS;
        m_data.deal_cause = m_identifyComboBox->currentText();
    }

    // 告警等级;
    m_data.alarm_level_name = m_alarmLavelComBoBox->currentText();

    // 识别结果;
    if (m_identifyRight->isChecked())
    {
        m_data.deal_result = "";
    }
    else
    {
        m_data.deal_result = m_realValueLineEdit->text();
    }

    if (!m_data.is_dealed)
    {
        m_data.deal_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
    }

    WHEEL_BACK_TO_CORE_SOCKET.robot_partrol_result_verify_req(m_data, (int)m_taskCheckType);

    m_pButtonSave->setText("保存中");
    this->setDisabled(true);

    m_saveTimeoutTimer.start();
}

void DLWheelTaskCheck::onRcfDownloadImageFinished(int result, int imageIndex, QString strImagePath)
{
    if (result >= 0)
    {
		ROS_INFO("task audit path:strImagePath:%s ", strImagePath.toStdString());
        WheelJudgeTakePhoto type = WHEEL_DEVICE_CONFIG.getWheelChooseRecForDeviceUUidBool(m_data.device_uuid);
        if (type == VisibleLightJudge)
        {
            m_visibleImageWidget->setImage(strImagePath);
        }
        else if (type == InfraredLightJudge)
        {
			strImagePath = strImagePath.replace("-temp", "_result");
            m_fraredImageWidget->setImage(strImagePath);
        }
    }
//     else
//     {
//         m_imageWidgetList[imageIndex]->setImage(":/Resources/Common/image/ImageLoadFailed.png");
//     }
}

void DLWheelTaskCheck::initTopWidget()
{
	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("TopBackWidget");

	QLabel* titleLabel = new QLabel;
	titleLabel->setText("任务审核");
	titleLabel->setStyleSheet("color:rgb(63,81,181);font-weight:bold;");

	QIcon icon = style()->standardIcon(QStyle::SP_TitleBarCloseButton);
	m_pButtonClose = new QToolButton;
	m_pButtonClose->setIcon(icon);
	m_pButtonClose->setIconSize(QSize(12, 12));
	m_pButtonClose->setFixedSize(QSize(14, 14));
	m_pButtonClose->setStyleSheet("border:none;background:rgb(220, 235, 235);");
	connect(m_pButtonClose, &QToolButton::clicked, this, [=] {
        if (!m_isSingleTask)
        {
            emit signalRefreshTable();
            m_currentTaskCheckIndexLabel->setText("1");
        }
		hide();
	});

	m_pButtonSave = new QToolButton;
	m_pButtonSave->setText("保存");
	m_pButtonSave->setFixedSize(QSize(70, 25));
	m_pButtonSave->setStyleSheet("border-radius:2px;border:1px solid gray;background:rgb(240, 240, 240);");
    connect(m_pButtonSave, &QToolButton::clicked, this, &DLWheelTaskCheck::onSaveTaskCheckData);

	m_pButtonCancel = new QToolButton;
	m_pButtonCancel->setText("取消");
	m_pButtonCancel->setFixedSize(QSize(70, 25));
	m_pButtonCancel->setStyleSheet("border-radius:2px;border:1px solid gray;background:rgb(240, 240, 240);");
	connect(m_pButtonCancel, &QToolButton::clicked, this, [=] {
        if (!m_isSingleTask)
        {
            emit signalRefreshTable();
            m_currentTaskCheckIndexLabel->setText("1");
        }
		hide();
	});

	QHBoxLayout* hTitleLayout = new QHBoxLayout;
	hTitleLayout->addWidget(titleLabel);
	hTitleLayout->addStretch();
	hTitleLayout->addWidget(m_pButtonClose);
	hTitleLayout->setMargin(5);

	// 上一页;
    m_currentTaskCheckIndexLabel = new QLabel("1");
	QWidget* lastPageButtonBackWidget = new QWidget;
	lastPageButtonBackWidget->setObjectName("LastPageButtonBackWidget");
	lastPageButtonBackWidget->setFixedHeight(30);
	m_pButtonLastPage = new QToolButton;
	m_pButtonLastPage->setText("上一页");
	m_pButtonLastPage->setStyleSheet("border:none;");
	connect(m_pButtonLastPage, &QToolButton::clicked, this, [=] {
        m_taskCheckDataIndex--;
        if (m_taskCheckDataIndex == 0)
        {
            m_pButtonLastPage->setDisabled(true);
        }
        m_currentTaskCheckIndexLabel->setText(QString::number(m_taskCheckDataIndex + 1));
        setData(m_dataList[m_taskCheckDataIndex]);
	});

	QHBoxLayout* hButtonLayout = new QHBoxLayout(lastPageButtonBackWidget);
	hButtonLayout->addWidget(m_pButtonLastPage);
	hButtonLayout->addStretch();
    hButtonLayout->addWidget(m_currentTaskCheckIndexLabel);
	hButtonLayout->setContentsMargins(0, 0, 10, 0);

	// 下一页;
	QLabel* identifyResultLabel = new QLabel("识别结果");
	identifyResultLabel->setStyleSheet("font-weight:bold;");

	m_pButtonNextPage = new QToolButton;
	m_pButtonNextPage->setText("下一页");
	m_pButtonNextPage->setStyleSheet("border:none");
	connect(m_pButtonNextPage, &QToolButton::clicked, this, [=] {
        m_taskCheckDataIndex++;
        if (m_taskCheckDataIndex >= m_taskCheckDataCount)
        {
            m_taskCheckDataIndex = m_taskCheckDataCount - 1;
            m_pButtonNextPage->setDisabled(true);
            return;
        }
        m_pButtonLastPage->setDisabled(false);
        m_currentTaskCheckIndexLabel->setText(QString::number(m_taskCheckDataIndex + 1));
        setData(m_dataList[m_taskCheckDataIndex]);
	});

	QWidget* nextPageBackWidget = new QWidget;
	nextPageBackWidget->setFixedSize(QSize(402, 30));
	nextPageBackWidget->setObjectName("NextPageBackWidget");

	QHBoxLayout* hTopBackLayout = new QHBoxLayout(nextPageBackWidget);
	hTopBackLayout->addWidget(identifyResultLabel);
	hTopBackLayout->addStretch();
	hTopBackLayout->addWidget(m_pButtonNextPage);
	hTopBackLayout->setContentsMargins(5, 0, 5, 0);

	QHBoxLayout* hPageTurnLayout = new QHBoxLayout;
	hPageTurnLayout->addWidget(lastPageButtonBackWidget);
	hPageTurnLayout->addWidget(nextPageBackWidget);
	hPageTurnLayout->setSpacing(0);
	hPageTurnLayout->setMargin(0);

	QVBoxLayout* vTopLayout = new QVBoxLayout(m_topBackWidget);
	vTopLayout->addLayout(hTitleLayout);
	vTopLayout->addLayout(hPageTurnLayout);
	vTopLayout->setSpacing(0);
	vTopLayout->setMargin(0);
}

void DLWheelTaskCheck::initLastPageWidget()
{
	
}

void DLWheelTaskCheck::initVisibleImageWidget()
{
	m_visibleImageWidget = new TitleWidget;
	m_visibleImageWidget->setObjectName("TitleWidget");
	m_visibleImageWidget->setFixedHeight(TITLE_WIDGET_HEIGHT);
	m_visibleImageWidget->setTitleText("可见光图片");
    m_visibleImageWidget->setImageWidget();
}

void DLWheelTaskCheck::initFraredImageWidget()
{
	m_fraredImageWidget = new TitleWidget(true);
	m_fraredImageWidget->setFixedHeight(TITLE_WIDGET_HEIGHT);
	m_fraredImageWidget->setTitleText("红外图片");
    m_fraredImageWidget->setImageWidget();
}

void DLWheelTaskCheck::initAudioFileWidget()
{
	m_audioImageWidget = new TitleWidget;
	m_audioImageWidget->setFixedHeight(TITLE_WIDGET_HEIGHT);
	m_audioImageWidget->setTitleText("音频文件");
    m_audioImageWidget->setImageWidget();
}

void DLWheelTaskCheck::initThresholdValueWidget()
{
	m_ThresholdValueWidget = new TitleWidget;
	m_ThresholdValueWidget->setFixedHeight(TITLE_WIDGET_HEIGHT);
	m_ThresholdValueWidget->setTitleText("阈值信息");
    m_ThresholdValueWidget->setIsWidget(true);
    m_ThresholdValueWidget->setImageWidget();
}

void DLWheelTaskCheck::initLeftWidget()
{
	initLastPageWidget();
	initVisibleImageWidget();
	initFraredImageWidget();
	initAudioFileWidget();
	initThresholdValueWidget();

	QWidget* lineEditBackWidget = new QWidget;
	lineEditBackWidget->setStyleSheet("background:white;");

	m_pointInfoLineEdit = new QLineEdit;
    m_pointInfoLineEdit->setReadOnly(true);
	m_pointInfoLineEdit->setText("主变（电抗器）全景（右面）");
	m_pointInfoLineEdit->setStyleSheet("border:1px solid lightgray;");
	m_pointInfoLineEdit->setFixedSize(QSize(250, 25));

    m_timeLabel = new QLabel;

	QHBoxLayout* hPointInfoLayout = new QHBoxLayout(lineEditBackWidget);
	hPointInfoLayout->addWidget(new QLabel("设备名称"));
	hPointInfoLayout->addWidget(m_pointInfoLineEdit);
    hPointInfoLayout->addWidget(new QLabel("时间:"));
    hPointInfoLayout->addWidget(m_timeLabel);
	hPointInfoLayout->addStretch();
	hPointInfoLayout->setSpacing(15);
	hPointInfoLayout->setContentsMargins(3, 3, 0, 3);

	m_leftBackWidget = new QWidget;
	QVBoxLayout* vLeftLayout = new QVBoxLayout(m_leftBackWidget);
	vLeftLayout->addWidget(lineEditBackWidget);
	vLeftLayout->addWidget(m_visibleImageWidget);
	vLeftLayout->addWidget(m_fraredImageWidget);
	vLeftLayout->addWidget(m_audioImageWidget);
	vLeftLayout->addWidget(m_ThresholdValueWidget);
	vLeftLayout->setSpacing(10);
	vLeftLayout->setMargin(0);
}

void DLWheelTaskCheck::initRightWidget()
{
	m_rightBackWidget = new QWidget;
	m_rightBackWidget->setFixedWidth(400);
	m_rightBackWidget->setObjectName("RightBackWidget");

	QLabel* resultLabel = new QLabel("结果:");
	resultLabel->setFixedHeight(30);
	resultLabel->setAlignment(Qt::AlignLeft);
	resultLabel->setStyleSheet("font-size:16px;");

    m_resultShow = new QLabel("");
    //resultLabel->setFixedHeight(30);
    m_resultShow->setAlignment(Qt::AlignLeft);
    m_resultShow->setStyleSheet("font-size:16px;");
	
	m_checkBoxNormal = new QRadioButton("正常");
	m_checkBoxAbnormal = new QRadioButton("异常");

	QButtonGroup* resulttButtonGroup = new QButtonGroup(this);
	resulttButtonGroup->addButton(m_checkBoxNormal, 0);
	resulttButtonGroup->addButton(m_checkBoxAbnormal, 1);
    connect(resulttButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, [=](int buttonId) {
        if (buttonId == 0)
        {
            m_identifyComboBox->setDisabled(true);

        }
        else if (buttonId == 1)
        {
            m_identifyComboBox->setDisabled(false);
        }
    });


	m_identifyComboBox = new QComboBox;
    m_identifyComboBox->addItems(QStringList() << "异常一" << "异常二" << "异常三");

	QLabel* alarmLevelLabel = new QLabel("告警等级");
	
	m_alarmLavelComBoBox = new QComboBox;
// 	m_alarmLavelComBoBox->setEditable(true);
// 	m_alarmLavelComBoBox->lineEdit()->setReadOnly(true);
    m_alarmLavelComBoBox->addItems(QStringList() << "正常" << "预警" << "一般告警" << "严重告警" << "危急告警" << "识别异常");
 
	m_identifyRight = new QRadioButton("识别正确");
	m_identifyError = new QRadioButton("识别错误");

	QButtonGroup* identifyButtonGroup = new QButtonGroup(this);
	identifyButtonGroup->addButton(m_identifyRight, 0);
	identifyButtonGroup->addButton(m_identifyError, 1);

    connect(identifyButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, [=](int buttonId) {
        if (buttonId == 0)
        {
            m_realValueLineEdit->setDisabled(true);
        }
        else if (buttonId == 1)
        {
            m_realValueLineEdit->setDisabled(false);
        }
    });

	QLabel* realValueLabel = new QLabel("实际值:");

	m_realValueLineEdit = new QLineEdit;
	m_realValueLineEdit->setFixedSize(QSize(150, 25));

    m_alarmFrequencyLabel = new QLabel;
    m_alarmFrequencyLabel->setFixedSize(QSize(150, 25));

    m_alarmTypeLabel = new QLabel;

//     m_alarmThresholdLabel = new QLabel;
//     m_alarmThresholdLabel->setText("0");

	QGridLayout* gRightLayout = new QGridLayout(m_rightBackWidget);
    gRightLayout->addWidget(resultLabel, 0, 0, 1, 1);
    gRightLayout->addWidget(m_resultShow, 0, 1, 1, 1);
	gRightLayout->addWidget(m_checkBoxNormal, 1, 0, 1, 2);
	gRightLayout->addWidget(m_checkBoxAbnormal, 2, 0, 1, 1);
	gRightLayout->addWidget(m_identifyComboBox, 2, 1, 1, 1);
	gRightLayout->addItem(new QSpacerItem(20, 150, QSizePolicy::Fixed, QSizePolicy::Fixed), 3, 0);
	gRightLayout->addWidget(alarmLevelLabel, 4, 0, 1, 1);
	gRightLayout->addWidget(m_alarmLavelComBoBox, 4, 1, 1, 1);
	gRightLayout->addItem(new QSpacerItem(20, 150, QSizePolicy::Fixed, QSizePolicy::Fixed), 5, 0);
	gRightLayout->addWidget(m_identifyRight, 6, 0, 1, 2);
	gRightLayout->addWidget(m_identifyError, 7, 0, 1, 2);
	gRightLayout->addWidget(new QLabel("实际值:"), 8, 0, 1, 1);
	gRightLayout->addWidget(m_realValueLineEdit, 8, 1, 1, 1);
    gRightLayout->addWidget(new QLabel("告警频次:"), 9, 0, 1, 1);
    gRightLayout->addWidget(m_alarmFrequencyLabel, 9, 1, 1, 1);
    gRightLayout->addWidget(new QLabel("告警类型:"), 10, 0, 1, 1);
    gRightLayout->addWidget(m_alarmTypeLabel, 10, 1, 1, 1);
//     gRightLayout->addWidget(new QLabel("告警阈值:"), 11, 0, 1, 1);
//     gRightLayout->addWidget(m_alarmThresholdLabel, 11, 1, 1, 1);
	gRightLayout->addItem(new QSpacerItem(20, 40, QSizePolicy::Fixed, QSizePolicy::Expanding), 9, 0);
	gRightLayout->setContentsMargins(5, 5, 180, 40);
}

void DLWheelTaskCheck::initCenterWidget()
{
	initLeftWidget();
	initRightWidget();
	m_centerBackWidget = new QWidget;
	m_centerBackWidget->setObjectName("CenterWidget");

	QLabel* vSpliterLabel = new QLabel;
	vSpliterLabel->setFixedWidth(2);
	vSpliterLabel->setStyleSheet("background:lightgray;");

	QLabel* hSpliterLabel = new QLabel;
	hSpliterLabel->setFixedHeight(1);
	hSpliterLabel->setStyleSheet("background:rgba(63,81,181, 150);");

	QHBoxLayout* hCenterLayout = new QHBoxLayout(m_centerBackWidget);
	hCenterLayout->addWidget(m_leftBackWidget);
	hCenterLayout->addWidget(vSpliterLabel);
	hCenterLayout->addWidget(m_rightBackWidget);
	hCenterLayout->setSpacing(0);
	hCenterLayout->setContentsMargins(1, 0, 1, 1);
}

void DLWheelTaskCheck::initWidget()
{
	initTopWidget();
	initCenterWidget();	
    initTimeout();

	QHBoxLayout* hBottomLayout = new QHBoxLayout;
	hBottomLayout->addStretch();
	hBottomLayout->addWidget(m_pButtonSave);
	hBottomLayout->addWidget(m_pButtonCancel);
	hBottomLayout->setContentsMargins(0, 10, 15, 10);
	hBottomLayout->setSpacing(10);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_topBackWidget);
	vMainLayout->addWidget(m_centerBackWidget);
	vMainLayout->addLayout(hBottomLayout);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(5);
}

void DLWheelTaskCheck::initTimeout()
{
    m_saveTimeoutTimer.setInterval(15 * 1000);
    connect(&m_saveTimeoutTimer, &QTimer::timeout, this, [=] {
        m_saveTimeoutTimer.stop();
        DLMessageBox::showDLMessageBox(NULL, "错误", "发送超时", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
        this->setDisabled(false);
        m_pButtonSave->setText("保存");
    });
}

void DLWheelTaskCheck::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.setPen(Qt::lightGray);
	painter.setBrush(QColor(200, 224, 221));
	painter.drawRoundedRect(QRect(0, 0, this->width() - 1, this->height() - 1), 3, 3);
}