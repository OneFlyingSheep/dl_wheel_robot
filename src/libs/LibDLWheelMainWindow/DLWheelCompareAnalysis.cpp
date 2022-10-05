#include "DLWheelCompareAnalysis.h"
#include <QDateTime>
#include <QCalendarWidget>
#include <QButtonGroup>
#include <QRadioButton>
#include <QApplication>
#include <QDesktopWidget>
#include "LibDLWheelCustomWidget/CustomTreeWidget.h"
#include "LibDLWheelCustomWidget/CustomButtonListWidget.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelCustomWidget/TurnPageWidget.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelRobotDeviceCure/CompareAnalyzeCurve.h"
#include "LibDLWheelRobotCreateReport/SearchRecordCreateExcel.h"
#include "RCFImageDownload.h"
#include "LibDLWheelCustomWidget/DefData.h"
#include "LibDLCreateExcel/CreateExcelForQt5.h"

#pragma execution_character_set("utf-8")

DLWheelCompareAnalysis::DLWheelCompareAnalysis()
	: m_isInitWidget(false)
    , m_currentPageIndex(1)
    , m_tableMaxShowCount(6)
    , m_currentChooseDeviceId("")
    , m_RCFImageDownload(NULL)
{
	this->setStyleSheet("QWidget#DataTimeBackWidget,QWidget#CollectInfoBackWidget,\
							QWidget#LayoutButtonBackWidget{background:rgb(175, 191, 255);}\
							QWidget#LineChartBackWidget,QWidget#ImageShowBackWidget{background:white;}");
}

DLWheelCompareAnalysis::~DLWheelCompareAnalysis()
{
    if (m_RCFImageDownload != NULL)
    {
        m_RCFImageDownload->stopDownloadImage();
    }
}

void DLWheelCompareAnalysis::initDeviceTreeWidget()
{
	m_deviceTreeWidget = new CustomTreeWidget;
	m_deviceTreeWidget->setTreeWidgetType(TreeItemWidgetType::ColorRect_CheckBox_Without);
	m_deviceTreeWidget->setSearchLineEditVisible(false);
	m_deviceTreeWidget->refreshTreeItemList();

    connect(m_deviceTreeWidget, &CustomTreeWidget::signalDeviceNodeClicked, this, [=](QString strDeviceId) {
        m_currentChooseDeviceId = QStringList() << strDeviceId;
        initTableData();
    });
}

void DLWheelCompareAnalysis::initDateTimeSelectWidget()
{
	QLabel* labelStartTime = new QLabel("开始时间:");

	QLabel* labelEndTime = new QLabel("结束时间:");

	QCalendarWidget* startDateWidget = new QCalendarWidget;
	m_startDateEdit = new QDateTimeEdit;
	m_startDateEdit->setFixedSize(QSize(150, 25));
	m_startDateEdit->setCalendarPopup(true);
	m_startDateEdit->setCalendarWidget(startDateWidget);
	m_startDateEdit->setDateTime(QDateTime::currentDateTime());
	m_startDateEdit->setDisplayFormat("yyyy-MM-dd hh:MM:ss");
    m_startTime = QDateTime::currentDateTime().toString("yyyy-MM-dd");

	QCalendarWidget* endDateWidget = new QCalendarWidget;
	m_endDateEdit = new QDateTimeEdit;
	m_endDateEdit->setFixedSize(QSize(150, 25));
	m_endDateEdit->setCalendarPopup(true);
	m_endDateEdit->setCalendarWidget(endDateWidget);
	m_endDateEdit->setDateTime(QDateTime::currentDateTime().addMonths(1));
	m_endDateEdit->setDisplayFormat("yyyy-MM-dd hh:MM:ss");
    m_endTime = QDateTime::currentDateTime().addMonths(1).toString("yyyy-MM-dd");

	m_dataTimeBackWidget = new QWidget;
	m_dataTimeBackWidget->setObjectName("DataTimeBackWidget");
	m_dataTimeBackWidget->setFixedHeight(30);
	QHBoxLayout* hTimeLayout = new QHBoxLayout(m_dataTimeBackWidget);
	hTimeLayout->addWidget(labelStartTime);
	hTimeLayout->addWidget(m_startDateEdit);
	hTimeLayout->addWidget(labelEndTime);
	hTimeLayout->addWidget(m_endDateEdit);
	hTimeLayout->addStretch();
	hTimeLayout->setSpacing(5);
	hTimeLayout->setContentsMargins(5, 0, 0, 5);
}

void DLWheelCompareAnalysis::initCollectInfoSelectWidget()
{
	QLabel* collotInfoLabel = new QLabel("采集信息:");
		
	QRadioButton* pButtonVisibleLight = new QRadioButton("可见光");
	QRadioButton* pButtonInfrared = new QRadioButton("红外");
	QRadioButton* pButtonAudioVideo = new QRadioButton("音视频");

	QButtonGroup* radioButtonGroup = new QButtonGroup(this);
	radioButtonGroup->addButton(pButtonVisibleLight, CollectInfo_VisibleLight);
	radioButtonGroup->addButton(pButtonInfrared, CollectInfo_Infrared);
	radioButtonGroup->addButton(pButtonAudioVideo, CollectInfo_AudioVideo);
	connect(radioButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &DLWheelCompareAnalysis::onRadioButtonClicked);

	m_collectInfoBackWidget = new QWidget;
	m_collectInfoBackWidget->setObjectName("CollectInfoBackWidget");
	m_collectInfoBackWidget->setFixedHeight(30);
	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addWidget(collotInfoLabel);
	hButtonLayout->addWidget(pButtonVisibleLight);
	hButtonLayout->addWidget(pButtonInfrared);
	hButtonLayout->addWidget(pButtonAudioVideo);
	hButtonLayout->addStretch();
	hButtonLayout->setSpacing(5);
	hButtonLayout->setContentsMargins(5, 0, 5, 0);

    QLabel* deviceAreaLabel = new QLabel("设备区域:");
    QCheckBox* deviceAreaOne = new QCheckBox("500kv设备区");
    QCheckBox* deviceAreaTwo = new QCheckBox("220kv设备区");
    QCheckBox* deviceAreaThree = new QCheckBox("110kv设备区");

    QHBoxLayout* hDeviceAreaLayout = new QHBoxLayout;
    hDeviceAreaLayout->addWidget(deviceAreaLabel);
    hDeviceAreaLayout->addWidget(deviceAreaOne);
    hDeviceAreaLayout->addWidget(deviceAreaTwo);
    hDeviceAreaLayout->addWidget(deviceAreaThree);
    hDeviceAreaLayout->setSpacing(10);
    hDeviceAreaLayout->setContentsMargins(5, 0, 0, 0);
    
    QLabel* deviceTypeLabel = new QLabel("设备类型:");
    QCheckBox* deviceTypeOne = new QCheckBox("主变");
    QCheckBox* deviceTypeTwo = new QCheckBox("断路器");
    QCheckBox* deviceTypeThree = new QCheckBox("隔离开关");

    QHBoxLayout* hDeviceTypeLayout = new QHBoxLayout;
    hDeviceTypeLayout->addWidget(deviceTypeLabel);
    hDeviceTypeLayout->addWidget(deviceTypeOne);
    hDeviceTypeLayout->addWidget(deviceTypeTwo);
    hDeviceTypeLayout->addWidget(deviceTypeThree);
    hDeviceTypeLayout->setSpacing(10);
    hDeviceTypeLayout->setContentsMargins(5, 0, 0, 0);

    QLabel* recognitionTypeLabel = new QLabel("识别类型:");
    QCheckBox* recognitionTypeOne = new QCheckBox("表计读取");
    QCheckBox* recognitionTypeTwo = new QCheckBox("位置状态识别");
    QCheckBox* recognitionTypeThree = new QCheckBox("设备外观查看");

    QHBoxLayout* hRecognitionTypeLayout = new QHBoxLayout;
    hRecognitionTypeLayout->addWidget(recognitionTypeLabel);
    hRecognitionTypeLayout->addWidget(recognitionTypeOne);
    hRecognitionTypeLayout->addWidget(recognitionTypeTwo);
    hRecognitionTypeLayout->addWidget(recognitionTypeThree);
    hRecognitionTypeLayout->setSpacing(10);
    hRecognitionTypeLayout->setContentsMargins(5, 0, 0, 0);

    QLabel* meterTypeLabel = new QLabel("表计类型:");
    QCheckBox* meterTypeLabelOne = new QCheckBox("设备区域");
    QCheckBox* meterTypeLabelTwo = new QCheckBox("设备类型");
    QCheckBox* meterTypeLabelThree = new QCheckBox("识别类型");

    QHBoxLayout* hMeterTypeLabelLayout = new QHBoxLayout;
    hMeterTypeLabelLayout->addWidget(meterTypeLabel);
    hMeterTypeLabelLayout->addWidget(meterTypeLabelOne);
    hMeterTypeLabelLayout->addWidget(meterTypeLabelTwo);
    hMeterTypeLabelLayout->addWidget(meterTypeLabelThree);
    hMeterTypeLabelLayout->setSpacing(10);
    hMeterTypeLabelLayout->setContentsMargins(5, 0, 0, 0);

    QVBoxLayout* vCollectLayout = new QVBoxLayout(m_collectInfoBackWidget);
    vCollectLayout->addLayout(hButtonLayout);
     vCollectLayout->addLayout(hDeviceAreaLayout);
     vCollectLayout->addLayout(hDeviceTypeLayout);
     vCollectLayout->addLayout(hRecognitionTypeLayout);
     vCollectLayout->addLayout(hMeterTypeLabelLayout);
    vCollectLayout->setSpacing(0);
    vCollectLayout->setMargin(0);
}

void DLWheelCompareAnalysis::initButtonWidget()
{
	m_customButtonListWidget = new CustomButtonListWidget;
	m_customButtonListWidget->addToolButton(0, "查询", ":/Resources/Common/image/Search.png");
	m_customButtonListWidget->addToolButton(1, "重置", ":/Resources/Common/image/Reset.png");
	m_customButtonListWidget->addToolButton(2, "导出", ":/Resources/Common/image/ExportButton.png");
	m_customButtonListWidget->addWidgetFinished();

    connect(m_customButtonListWidget, &CustomButtonListWidget::signalButtonClicked, this, &DLWheelCompareAnalysis::onCustomButtonClicked);
}

void DLWheelCompareAnalysis::initTableWidget()
{
	m_customTableWidget = new CustomTableWidget(5);
	m_customTableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "识别时间" << "点位名称" << "识别结果" << "采集信息");
	
    connect(m_customTableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableData();
    });
    connect(m_customTableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableData();
    });
}

void DLWheelCompareAnalysis::initLineChartWidget()
{
	m_lineChartBackWidget = new CompareAnalyzeCurve;
	m_lineChartBackWidget->setObjectName("LineChartBackWidget");
	m_lineChartBackWidget->setFixedHeight(250);
}

void DLWheelCompareAnalysis::initCenterWidget()
{
	initDateTimeSelectWidget();
	initCollectInfoSelectWidget();
	initButtonWidget();
	initTableWidget();
	initLineChartWidget();

	m_centerBackWidget = new QWidget;
	m_centerBackWidget->setFixedWidth(550);
	QVBoxLayout* vCenterLayout = new QVBoxLayout(m_centerBackWidget);
	vCenterLayout->addWidget(m_dataTimeBackWidget);
	vCenterLayout->addWidget(m_collectInfoBackWidget);
	vCenterLayout->addWidget(m_customButtonListWidget);
	vCenterLayout->addWidget(m_customTableWidget);
	vCenterLayout->addWidget(m_lineChartBackWidget);
	vCenterLayout->setSpacing(0);
	vCenterLayout->setMargin(0);
}

void DLWheelCompareAnalysis::initRightWidget()
{
	QRadioButton* pButton22Layout = new QRadioButton("2*2");
	QRadioButton* pButton23Layout = new QRadioButton("2*3");
    pButton23Layout->setChecked(true);
	QRadioButton* pButton33Layout = new QRadioButton("3*3");

	QButtonGroup* layoutButtonGroup = new QButtonGroup(this);
	layoutButtonGroup->addButton(pButton22Layout, Layout_22);
	layoutButtonGroup->addButton(pButton23Layout, Layout_23);
	layoutButtonGroup->addButton(pButton33Layout, Layout_33);
	connect(layoutButtonGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this, &DLWheelCompareAnalysis::onImageLayoutSelect);

	QWidget* layoutButtonBackWidget = new QWidget;
	layoutButtonBackWidget->setFixedHeight(30);
	layoutButtonBackWidget->setObjectName("LayoutButtonBackWidget");
	QHBoxLayout* hLayoutButtonLayout = new QHBoxLayout(layoutButtonBackWidget);
	hLayoutButtonLayout->addWidget(pButton22Layout);
	hLayoutButtonLayout->addWidget(pButton23Layout);
	hLayoutButtonLayout->addWidget(pButton33Layout);
	hLayoutButtonLayout->addStretch();
	hLayoutButtonLayout->setSpacing(50);
	hLayoutButtonLayout->setContentsMargins(5, 0, 0, 0);

	m_imageShowBackWidget = new QWidget;
	m_imageShowBackWidget->setObjectName("ImageShowBackWidget");
    QScreen *screen = QApplication::primaryScreen();
    QRect screenRect = screen->availableGeometry();

    m_imageShowBackWidget->setFixedWidth(screenRect.width() - 860);

	imageShowRelayout(3, 2);

	m_imageTurnPageWidget = new TurnPageWidget;
	
	m_rightBackWidget = new QWidget;
	QVBoxLayout* vRightLayout = new QVBoxLayout(m_rightBackWidget);
	vRightLayout->addWidget(layoutButtonBackWidget);
	vRightLayout->addWidget(m_imageShowBackWidget);
	vRightLayout->addWidget(m_imageTurnPageWidget);
	vRightLayout->setSpacing(0);
	vRightLayout->setMargin(0);
}

void DLWheelCompareAnalysis::initTableData()
{
    m_startTime = m_startDateEdit->dateTime().toString("yyyy-MM-dd hh-mm-ss");
    m_endTime = m_endDateEdit->dateTime().toString("yyyy-MM-dd hh-mm-ss");

    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
    QList<WheelPatrolResultCompareStruct> tableDataList = WHEEL_PATROL_RESULT.getWheelPatrolResultCompareList(m_currentPageIndex, m_tableMaxShowCount, m_currentChooseDeviceId, m_currentCollectInfoId, m_startTime, m_endTime);
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    int totalPage, totalCount;
    WHEEL_PATROL_RESULT.getWheelPatrolResultCompareCount(totalPage, totalCount, m_tableMaxShowCount, m_currentChooseDeviceId, m_currentCollectInfoId, m_startTime, m_endTime);
    m_customTableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, m_tableMaxShowCount);

    for (int i = 0; i < m_imageShowWidgetList.count(); i++)
    {
        m_imageShowWidgetList[i]->setImage("");
        m_imageShowWidgetList[i]->setText("");
    }

    if (!tableDataList.isEmpty())
    {
        QStringList strTaskIdList, strDeviceIdList;
        for (int i = 0; i < tableDataList.count(); i++)
        {
            strTaskIdList.append(tableDataList[i].task_uuid);
            strDeviceIdList.append(tableDataList[i].device_uuid);

            QString strImageText = tableDataList[i].device_point_type_name + "  " + tableDataList[i].inspect_time;
            m_imageShowWidgetList[i]->setText(strImageText);
        }

        // 如果table进行了刷新，释放之前下载图片的线程，重新开辟线程进行新的下载;
        if (m_RCFImageDownload != NULL)
        {
            m_RCFImageDownload->stopDownloadImage();
            disconnect(m_RCFImageDownload, &RCFImageDownload::signalDowmloadFinished, this, &DLWheelCompareAnalysis::onRcfDownloadImageFinished);
            m_RCFImageDownload->deleteLater();
        }

        // 这里加上一个图片加载窗口，防止用户多次切换任务，下载和显示时界面操作卡顿;
        DLMessageBox* messageBox = new DLMessageBox(this);
        messageBox->setFixedWidth(250);
        messageBox->setMessageContent("图片加载中");
        messageBox->setButtonOKVisible(false);
        messageBox->setWindowModality(Qt::ApplicationModal);

        m_RCFImageDownload = new RCFImageDownload(this);
        connect(m_RCFImageDownload, &RCFImageDownload::signalDowmloadFinished, this, &DLWheelCompareAnalysis::onRcfDownloadImageFinished);
        // 所有图片下载完成时，关闭图片加载窗口;
        connect(m_RCFImageDownload, &RCFImageDownload::signalAllDownloaded, this, [=] {
            messageBox->close();
        });

        m_RCFImageDownload->setImageInfoList(strTaskIdList, strDeviceIdList);
        m_RCFImageDownload->start();

        messageBox->show();
    }

    for (int i = 0; i < tableDataList.count(); i++)
    {
        tableWidget->insertRow(i);
        QTableWidgetItem* checkBoxItem = new QTableWidgetItem();
        checkBoxItem->setCheckState(Qt::Unchecked);
        checkBoxItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, checkBoxItem);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 1, idItem);

        QTableWidgetItem* inspectTimeItem = new QTableWidgetItem(tableDataList[i].inspect_time);
        tableWidget->setItem(i, 2, inspectTimeItem);
        inspectTimeItem->setTextAlignment(Qt::AlignCenter);

        QTableWidgetItem* pointNameItem = new QTableWidgetItem(tableDataList[i].device_point_type_name);
        pointNameItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, pointNameItem);

        QTableWidgetItem* recognitionResultItem = new QTableWidgetItem(tableDataList[i].inspect_result);
        recognitionResultItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 4, recognitionResultItem);

        QTableWidgetItem* collectInfoItem = new QTableWidgetItem(tableDataList[i].save_type_name);
        collectInfoItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 5, collectInfoItem);

        tableWidget->setRowHeight(i, 40);
    }

    updateLineChart();
}

void DLWheelCompareAnalysis::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initDeviceTreeWidget();
	initCenterWidget();
	initRightWidget();

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_deviceTreeWidget);
	hMainLayout->addWidget(m_centerBackWidget);
	hMainLayout->addWidget(m_rightBackWidget);
	hMainLayout->setSpacing(5);
	hMainLayout->setMargin(0);
}

void DLWheelCompareAnalysis::onRadioButtonClicked(int buttonId)
{
	CollectInfoType type = CollectInfoType(buttonId);
	switch (type)
	{
	case CollectInfo_VisibleLight:
        m_currentCollectInfoId = QStringList() << "1";
		break;
	case CollectInfo_Infrared:
        m_currentCollectInfoId = QStringList() << "2" << "4";
		break;
	case CollectInfo_AudioVideo:
        m_currentCollectInfoId = QStringList() << "3";
		break;
	default:
		break;
	}
}

void DLWheelCompareAnalysis::onImageLayoutSelect(int buttonId)
{
	int column, row;
	LayoutType type = LayoutType(buttonId);
	switch (type)
	{
	case Layout_22:
	{
		row = 2;
		column = 2;
        m_tableMaxShowCount = 4;
	}
		break;
	case Layout_23:
	{
		row = 3;
		column = 2;
        m_tableMaxShowCount = 6;
	}
		break;
	case Layout_33:
	{
		row = 3;
		column = 3;
        m_tableMaxShowCount = 9;
	}
		break;
	default:
		break;
	}

    m_currentPageIndex = 1;
    imageShowRelayout(row, column);
    initTableData();
}

void DLWheelCompareAnalysis::onCustomButtonClicked(int buttonId)
{
    switch (buttonId)
    {
    case 0:
    {
        if (m_currentChooseDeviceId.isEmpty())
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "请先选择设备", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return;
        }
        initTableData();
    }
        break;
    case 1:
        break;
    case 2:
    // 导出;
    {
        QList<QStringList> excelData;
        excelData.append(QStringList() << "识别时间" << "点位名称" << "识别结果" << "采集信息" << "采集信息");

        QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
        for (int i = 0; i < tableWidget->rowCount(); i++)
        {
            QStringList strDataList;
            if (tableWidget->item(i, 0)->checkState() == Qt::Checked)
            {
                for (int j = 2; j < tableWidget->columnCount() - 1; j++)
                {
                    strDataList.append(tableWidget->item(i, j)->text());
                }

                strDataList << "" << "";
                excelData.append(strDataList);
            }
        }

        if (excelData.count() == 1)
        {
            DLMessageBox::showDLMessageBox(NULL, "提示", "未选择导出项", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
            return;
        }

        QString dir = QFileDialog::getSaveFileName(this, tr("Save File"), "", tr("excel(*.xlsx)"));
        if (dir.isEmpty())
        {
            return;
        }

        CreateExcelForQt5 excel;
        excel.import_excel_report(excelData, dir);
        //excel.CreateNewExcelForList(excelData, dir, EXCEL_COMPARE_ANALYZE);
    }
        break;
    default:
        break;
    }
}

void DLWheelCompareAnalysis::onRcfDownloadImageFinished(int result, int imageIndex, QString strImagePath)
{
    if (imageIndex >= m_imageShowWidgetList.count())
    {
        return;
    }

    if (result >= 0)
    {
        m_imageShowWidgetList[imageIndex]->setImage(strImagePath);
    }
    else
    {
        m_imageShowWidgetList[imageIndex]->setImage(":/Resources/Common/image/ImageLoadFailed.png");
    }
}

void DLWheelCompareAnalysis::imageShowRelayout(int row, int column)
{
	for (int i = 0; i < m_imageShowWidgetList.count(); i++)
	{
		delete m_imageShowWidgetList[i];
	}
	m_imageShowWidgetList.clear();

	QLayout* oldLayout = m_imageShowBackWidget->layout();
	if (oldLayout != NULL)
	{
		delete oldLayout;
	}

	QGridLayout* gImageShowLayout = new QGridLayout(m_imageShowBackWidget);
	for (int i = 0; i < row; i++)
	{
		for (int j = 0; j < column; j++)
		{
			ImageWidget* imageWidget = new ImageWidget;
			m_imageShowWidgetList.append(imageWidget);
			gImageShowLayout->addWidget(imageWidget, i, j);
		}
	}
	gImageShowLayout->setHorizontalSpacing(5);
	gImageShowLayout->setVerticalSpacing(10);
	gImageShowLayout->setContentsMargins(5, 5, 10, 15);
}

void DLWheelCompareAnalysis::updateLineChart()
{
    QTableWidget* tableWidget = m_customTableWidget->getTableWidget();
   
    QVector<double> enviData;
    QVector<QString> timeValue;
    QString pointName = "";
    if (tableWidget->rowCount() != 0)
    {
        pointName = tableWidget->item(0, 3)->text();
    }   

    for (int i = 0; i < tableWidget->rowCount(); i ++)
    {
        enviData.append(tableWidget->item(i, 4)->text().toDouble());
        timeValue.append(tableWidget->item(i, 2)->text());
    }

    m_lineChartBackWidget->setCompareDataVector(enviData, timeValue, pointName);
    m_lineChartBackWidget->DataClear();
}

void DLWheelCompareAnalysis::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), QColor(240, 240, 240));
}