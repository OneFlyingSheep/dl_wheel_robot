#include "DLWheelRobotAlarmSearch.h"
#include <QHBoxLayout>
#include <QHeaderView>
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelTaskDeviceDBSearch/DLWheelPatrolResultData.h"

#pragma execution_character_set("utf-8")

#define TABLE_PER_PAGE_COUNT 25

DLWheelRobotAlarmSearch::DLWheelRobotAlarmSearch(QWidget* parent /* = NULL */)
	: QWidget(parent)
	, m_isInitWidget(false)
    , m_currentPageIndex(1)
{
	this->setStyleSheet("QToolButton{border:none;}\
							QWidget#CommonBackWidget{background:rgb(175, 191, 255);border:1px solid rgb(166, 233, 210);}\
							QTableWidget{border:none;}");
}

void DLWheelRobotAlarmSearch::initTimeSearchWidget()
{
	m_topBackWidget = new QWidget;
	m_topBackWidget->setObjectName("CommonBackWidget");
	m_topBackWidget->setFixedHeight(30);

	QLabel* labelStartTime = new QLabel("开始时间:");
	QLabel* labelEndTime = new QLabel("结束时间:");

	m_pButtonSearch = new QToolButton;
	m_pButtonSearch->setText("查询");
	m_pButtonSearch->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
	m_pButtonSearch->setFixedSize(QSize(60, 20));
	m_pButtonSearch->setIcon(QIcon(":/Resources/Common/image/Search.png"));
	m_pButtonSearch->setIconSize(QSize(16, 16));
    connect(m_pButtonSearch, &QPushButton::clicked, this, [=] {
        m_startDate = m_startTimeWidget->date();
        m_endData = m_endTimeWidget->date();
        initTableData();
    });

	QCalendarWidget* startDateWidget = new QCalendarWidget;
	m_startTimeWidget = new QDateEdit;
	m_startTimeWidget->setFixedSize(QSize(100, 25));
	m_startTimeWidget->setCalendarPopup(true);
	m_startTimeWidget->setCalendarWidget(startDateWidget);
    m_startTimeWidget->setDate(QDate::currentDate());
    m_startDate = QDate::currentDate();

	QCalendarWidget* endDateWidget = new QCalendarWidget;
	m_endTimeWidget = new QDateEdit;
	m_endTimeWidget->setFixedSize(QSize(100, 25));
	m_endTimeWidget->setCalendarPopup(true);
	m_endTimeWidget->setCalendarWidget(endDateWidget);
    m_endTimeWidget->setDate(QDate::currentDate().addMonths(1));
    m_endData = QDate::currentDate().addMonths(1);

	QHBoxLayout* hExecuteTimeLayout = new QHBoxLayout(m_topBackWidget);
	hExecuteTimeLayout->addWidget(labelStartTime);
	hExecuteTimeLayout->addWidget(m_startTimeWidget);
	hExecuteTimeLayout->addWidget(labelEndTime);
	hExecuteTimeLayout->addWidget(m_endTimeWidget);
	hExecuteTimeLayout->addWidget(m_pButtonSearch);
	hExecuteTimeLayout->addStretch();
	hExecuteTimeLayout->setSpacing(15);
	hExecuteTimeLayout->setContentsMargins(10, 0, 0, 0);
}

void DLWheelRobotAlarmSearch::initTabelWidget()
{
	m_alarmInfoTableWidget = new CustomTableWidget(4, false);
	QTableWidget* tableWidget = m_alarmInfoTableWidget->getTableWidget();
	tableWidget->setHorizontalHeaderLabels(QStringList() << "序号" << "报警类型" << "报警内容" << "报警时间");

	tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
	tableWidget->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Fixed);

    connect(m_alarmInfoTableWidget, &CustomTableWidget::signalPageChanged, this, [=](int currentPageIndex) {
        m_currentPageIndex = currentPageIndex;
        initTableData();
    });
    connect(m_alarmInfoTableWidget, &CustomTableWidget::signalRefreshCurrentPage, this, [=] {
        initTableData();
    });

    tableWidget->setColumnWidth(0, 40);
}

void DLWheelRobotAlarmSearch::initTableData()
{
    QString strStartTime = m_startDate.toString("yyyy-MM-dd");
    QString strEndTime = m_endData.toString("yyyy-MM-dd");

    QTableWidget* tableWidget = m_alarmInfoTableWidget->getTableWidget();
    QList<WheelRobortAlarmSearchStruct> tableDataList = WHEEL_PATROL_RESULT.getWheelRobortAlarmSearchList(m_currentPageIndex, TABLE_PER_PAGE_COUNT, strStartTime, strEndTime);
    tableWidget->clearContents();
    tableWidget->setRowCount(0);

    int totalPage, totalCount;
    WHEEL_PATROL_RESULT.getWheelRobortAlarmSearchCount(totalPage, totalCount, TABLE_PER_PAGE_COUNT, strStartTime, strEndTime);
    m_alarmInfoTableWidget->setTurnPageInfo(m_currentPageIndex, totalPage, totalCount, TABLE_PER_PAGE_COUNT);

    for (int i = 0; i < tableDataList.count(); i++)
    {
        tableWidget->insertRow(i);

        QTableWidgetItem* idItem = new QTableWidgetItem(QString::number(i + 1));
        idItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 0, idItem);

        QTableWidgetItem* alarmTypeItem = new QTableWidgetItem(tableDataList[i].AlarmType);
        alarmTypeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 1, alarmTypeItem);

        QTableWidgetItem* alarmContenItem = new QTableWidgetItem(tableDataList[i].AlarmContent);
        alarmContenItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 2, alarmContenItem);

        QTableWidgetItem* alarmTimeItem = new QTableWidgetItem(tableDataList[i].AlarmTime);
        alarmTimeItem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setItem(i, 3, alarmTimeItem);
     
        tableWidget->setRowHeight(i, 40);
    }
}

void DLWheelRobotAlarmSearch::initWidget()
{
	if (m_isInitWidget)
	{
		return;
	}
	m_isInitWidget = true;

	initTimeSearchWidget();
	initTabelWidget();
    initTableData();

	QVBoxLayout* vMainLayout = new QVBoxLayout(this);
	vMainLayout->addWidget(m_topBackWidget);
	vMainLayout->addWidget(m_alarmInfoTableWidget);
	vMainLayout->setSpacing(0);
	vMainLayout->setMargin(5);
}