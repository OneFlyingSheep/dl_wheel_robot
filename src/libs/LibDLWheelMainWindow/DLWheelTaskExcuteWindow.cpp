#include "DLWheelTaskExcuteWindow.h"
#include <QHBoxLayout>
#include <QMenu>
#include <QApplication>

#pragma execution_character_set("utf-8")

DLWheelFixedTaskExcuteWindow::DLWheelFixedTaskExcuteWindow(QWidget* parent /*= NULL*/)
	: BaseWidget(parent, PopupWindow)
{
	initWidget();
	this->setTitleContent("定期执行");
	this->setAttribute(Qt::WA_DeleteOnClose);
	this->setFixedSize(QSize(300, 160));
	this->setStyleSheet("QPushButton#CommonButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(145, 167, 255);border-radius:3px;}\
						QPushButton#CommonButton:pressed{padding-left:2px;padding-top:2px;}\
						QWidget#TitleBackWidget{border-top:3px solid rgb(145, 167, 255); }");

}

DLWheelFixedTaskExcuteWindow::~DLWheelFixedTaskExcuteWindow()
{

}

void DLWheelFixedTaskExcuteWindow::initWidget()
{
	m_dateEdit = new QDateEdit;
	m_dateEdit->setCalendarPopup(true);
	m_dateEdit->setDate(QDate::currentDate());
	QCalendarWidget* calendarWidget = new QCalendarWidget;
	m_dateEdit->setCalendarWidget(calendarWidget);

	m_timeEdit = new QTimeEdit;
	m_timeEdit->setTime(QTime::currentTime());
	m_timeEdit->setDisplayFormat("hh:mm:ss");

	QHBoxLayout* hTimeLayout = new QHBoxLayout;
	hTimeLayout->addWidget(m_dateEdit);
	hTimeLayout->addWidget(m_timeEdit);
	hTimeLayout->setSpacing(15);
	hTimeLayout->setMargin(0);

	QPushButton* pButtonOk = new QPushButton("确定");
	pButtonOk->setFixedSize(QSize(70, 25));
	pButtonOk->setObjectName("CommonButton");

	connect(pButtonOk, &QPushButton::clicked, this, [=] {
		QString strDate = m_dateEdit->date().toString("yyyy-MM-dd");
		QTime strTime = m_timeEdit->time();
		emit signalOKButtonClikced(strDate, strTime);
	});

	QPushButton* pButtonCancel = new QPushButton("取消");
	pButtonCancel->setFixedSize(QSize(70, 25));
	pButtonCancel->setObjectName("CommonButton");

	connect(pButtonCancel, &QPushButton::clicked, this, [=] {
		this->close();
	});

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(pButtonOk);
	hButtonLayout->addWidget(pButtonCancel);
	hButtonLayout->setSpacing(15);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vCenterLayout = new QVBoxLayout(this->getCenterWidget());
	vCenterLayout->addLayout(hTimeLayout);
	vCenterLayout->addLayout(hButtonLayout);
	vCenterLayout->setSpacing(20);
	vCenterLayout->setMargin(10);
}

DLWheelPeriodTaskExcuteWindow::DLWheelPeriodTaskExcuteWindow(QWidget* parent /*= NULL*/)
	: BaseWidget(parent, PopupWindow)
{
	initWidget();
	this->setTitleContent("周期执行");
	this->setAttribute(Qt::WA_DeleteOnClose);
	this->setFixedSize(QSize(550, 350));
	this->setStyleSheet("QPushButton#CommonButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(145, 167, 255);border-radius:3px;}\
						QPushButton#CommonButton:pressed{padding-left:2px;padding-top:2px;}\
						QWidget#TitleBackWidget{border-top:3px solid rgb(145, 167, 255); }\
								QComboBox{color:black;background:white;padding-left:5px;border-radius:3px;border:1px solid gray;}\
								QComboBox:hover{border: 1px solid rgb(21 , 131 , 221);}\
								QComboBox QAbstractItemView::item{height:30px;}\
								QComboBox::down-arrow{border-image:url(:/Resources/Common/image/arrow_Down.png);height:10px;width:13px;}\
								QComboBox::down-arrow:on{border-image:url(:/Resources/Common/image/arrow_Up.png);height:10px;width:13px;}\
								QComboBox::drop-down{width:20px;background:transparent;padding-right:5px;}");

}

DLWheelPeriodTaskExcuteWindow::~DLWheelPeriodTaskExcuteWindow()
{

}

QTime DLWheelPeriodTaskExcuteWindow::getEveryDayTime()
{
    return m_everyDayTimeEdit->time();
}

void DLWheelPeriodTaskExcuteWindow::getEveryWeekTime(QString& chooseWeeks, QTime& strTime)
{
    for (int i = 0; i < m_weekChooseCheckBoxList.count(); i++)
    {
        if (m_weekChooseCheckBoxList.at(i)->isChecked())
        {
            chooseWeeks.append(QString::number(i + 1)).append(",");
        }
    }
    chooseWeeks.remove(chooseWeeks.length() - 1, 1);
    strTime = m_everyWeekTimeEdit->time();    
}

void DLWheelPeriodTaskExcuteWindow::getEveryMonthTime(QString& day, QTime& strTime)
{
    day = QString::number(m_dayChooseSpinBox->value());
    strTime = m_everyMonthTimeEdit->time();
}

void DLWheelPeriodTaskExcuteWindow::getFixTime(QString& strDate, int& intervalDay, QTime& strTime)
{
    strDate = m_startDateEdit->date().toString("yyyy-MM-dd");
    intervalDay = m_intervalDaysSpinBox->value();
    strTime = m_fixedIntervalTimeEdit->time();
}

void DLWheelPeriodTaskExcuteWindow::getMultiDateChoose(QString& strDateList, QTime& strTime)
{
    for (int i = 0; i < m_multiDayChooseListWidget->count(); i++)
    {
        strDateList.append(m_multiDayChooseListWidget->item(i)->text()).append(",");
    }
    strDateList.remove(strDateList.length() - 1, 1);
    strTime = m_multiDateTimeEdit->time();
}

void DLWheelPeriodTaskExcuteWindow::initEveryDayTask()
{
	QWidget* pEveryDayWidget = new QWidget;
	
	m_everyDayTimeEdit = new QTimeEdit;
    m_everyDayTimeEdit->setTime(QTime::currentTime());
	m_everyDayTimeEdit->setDisplayFormat("hh:mm:ss");
    m_everyDayTimeEdit->setFixedWidth(80);

	QHBoxLayout* hLayout = new QHBoxLayout();
	hLayout->addWidget(m_everyDayTimeEdit);
    hLayout->addStretch();
	hLayout->setMargin(0);

    QVBoxLayout* vLayout = new QVBoxLayout(pEveryDayWidget);
    vLayout->addLayout(hLayout);
    vLayout->addStretch();
    vLayout->setMargin(10);

	m_stackedWidget->insertWidget(EveryDay, pEveryDayWidget);
}

void DLWheelPeriodTaskExcuteWindow::initEveryWeek()
{
	QWidget* pEveryWeekWidget = new QWidget;
	
	m_everyWeekTimeEdit = new QTimeEdit;
	m_everyWeekTimeEdit->setDisplayFormat("hh:mm:ss");
    m_everyWeekTimeEdit->setTime(QTime::currentTime());

	QGridLayout* gCheckBoxLayout = new QGridLayout();
	int row = 0;
	for (int i = 0; i < 7; i++)
	{
		if (i > 2 && i < 6)
		{
			row = 1;
		}
        else if (i >= 6)
        {
            row = 2;
        }
		QCheckBox* checkBox = new QCheckBox;
		m_weekChooseCheckBoxList.append(checkBox);
		gCheckBoxLayout->addWidget(checkBox, row, i % 3);
	}

	m_weekChooseCheckBoxList[0]->setText("星期一");
	m_weekChooseCheckBoxList[1]->setText("星期二");
	m_weekChooseCheckBoxList[2]->setText("星期三");
	m_weekChooseCheckBoxList[3]->setText("星期四");
	m_weekChooseCheckBoxList[4]->setText("星期五");
	m_weekChooseCheckBoxList[5]->setText("星期六");
	m_weekChooseCheckBoxList[6]->setText("星期七");

	gCheckBoxLayout->addWidget(m_everyWeekTimeEdit, 0, 3);
    gCheckBoxLayout->setVerticalSpacing(20);
	gCheckBoxLayout->setMargin(0);

    QVBoxLayout* vLayout = new QVBoxLayout(pEveryWeekWidget);
    vLayout->addLayout(gCheckBoxLayout);
    vLayout->addStretch();
    vLayout->setMargin(10);

	m_stackedWidget->insertWidget(EveryWeek, pEveryWeekWidget);
}

void DLWheelPeriodTaskExcuteWindow::initEveryMonth()
{
	QWidget* pEveryMonth = new QWidget;

	m_dayChooseSpinBox = new QSpinBox;
	m_dayChooseSpinBox->setRange(1, 31);
	m_dayChooseSpinBox->setFixedWidth(50);

	m_everyMonthTimeEdit = new QTimeEdit;
	m_everyMonthTimeEdit->setDisplayFormat("hh:mm:ss");
    m_everyMonthTimeEdit->setFixedWidth(80);
    m_everyMonthTimeEdit->setTime(QTime::currentTime());

	QHBoxLayout* hLayout = new QHBoxLayout();
	hLayout->addWidget(m_dayChooseSpinBox);
	hLayout->addWidget(m_everyMonthTimeEdit);
    hLayout->addStretch();
	hLayout->setSpacing(15);
	hLayout->setMargin(0);

    QVBoxLayout* vLayout = new QVBoxLayout(pEveryMonth);
    vLayout->addLayout(hLayout);
    vLayout->addStretch();
    vLayout->setMargin(10);

	m_stackedWidget->insertWidget(EveryMonth, pEveryMonth);
}

void DLWheelPeriodTaskExcuteWindow::initFixedInterval()
{
	QWidget* pFixedIntervalWidget = new QWidget;
	
	m_startDateEdit = new QDateEdit;
    m_startDateEdit->setDate(QDate::currentDate());
    m_startDateEdit->setCalendarPopup(true);
    QCalendarWidget* calendarWidget = new QCalendarWidget(m_startDateEdit);
    m_startDateEdit->setCalendarWidget(calendarWidget);

	m_intervalDaysSpinBox = new QSpinBox;
	m_intervalDaysSpinBox->setRange(1, 1000);

	m_fixedIntervalTimeEdit = new QTimeEdit;
	m_fixedIntervalTimeEdit->setDisplayFormat("hh:mm:ss");
    m_fixedIntervalTimeEdit->setTime(QTime::currentTime());

	QHBoxLayout* hLayout = new QHBoxLayout();
	hLayout->addWidget(m_startDateEdit);
	hLayout->addWidget(m_intervalDaysSpinBox);
	hLayout->addWidget(m_fixedIntervalTimeEdit);
	hLayout->setSpacing(15);
	hLayout->setMargin(0);

    QVBoxLayout* vLayout = new QVBoxLayout(pFixedIntervalWidget);
    vLayout->addLayout(hLayout);
    vLayout->addStretch();
    vLayout->setMargin(10);

	m_stackedWidget->insertWidget(FixedInterval, pFixedIntervalWidget);
}

void DLWheelPeriodTaskExcuteWindow::initMultiDate()
{
	QWidget* pMultiDateWidget = new QWidget;

	m_calendarWidget = new QCalendarWidget;
	connect(m_calendarWidget, &QCalendarWidget::clicked, [=](const QDate& data) {
		QString strDate = data.toString("yyyy-MM-dd");
		if (!m_dateChooseMap.contains(strDate))
		{
			m_dateChooseMap.insert(strDate, "");
			m_multiDayChooseListWidget->addItem(new QListWidgetItem(strDate));
            m_multiDayChooseListWidget->sortItems();
		}
	});

	m_multiDayChooseListWidget = new QListWidget;
    m_multiDayChooseListWidget->setStyleSheet("QListView:item{height:25px;}");
    connect(m_multiDayChooseListWidget, &QListWidget::itemPressed, this, [=](QListWidgetItem* item) {
        if (qApp->mouseButtons() == Qt::RightButton)
        {
            QMenu* deleteMenu = new QMenu;
            QAction* deleteAction = deleteMenu->addAction("删除");
            connect(deleteAction, &QAction::triggered, this, [=] {
                m_dateChooseMap.remove(item->text());
                m_multiDayChooseListWidget->takeItem(m_multiDayChooseListWidget->row(item));
                delete item;
            });
            deleteMenu->exec(QCursor::pos());
            deleteMenu->deleteLater();
        }        
    });

	m_multiDateTimeEdit = new QTimeEdit;
    m_multiDateTimeEdit->setDisplayFormat("hh:mm:ss");
    m_multiDateTimeEdit->setTime(QTime::currentTime());

    QVBoxLayout* vTimeLayout = new QVBoxLayout;
    vTimeLayout->addWidget(m_multiDateTimeEdit);
    vTimeLayout->addWidget(m_multiDayChooseListWidget);
    vTimeLayout->setSpacing(15);
    vTimeLayout->setMargin(0);

	QHBoxLayout* hLayout = new QHBoxLayout(pMultiDateWidget);
	hLayout->addWidget(m_calendarWidget);
	hLayout->addLayout(vTimeLayout);
	hLayout->setSpacing(15);
	hLayout->setMargin(10);

	m_stackedWidget->insertWidget(MultiDate, pMultiDateWidget);
}

void DLWheelPeriodTaskExcuteWindow::initWidget()
{
	m_stackedWidget = new QStackedWidget;

	initEveryDayTask();
	initEveryWeek();
	initEveryMonth();
	initFixedInterval();
	initMultiDate();

	m_taskTypeComboBox = new QComboBox;
	m_taskTypeComboBox->addItems(QStringList() << "每天" << "每周" << "每月" << "固定间隔" << "多日期选择");
	connect(m_taskTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [=](int pageId) {
		m_stackedWidget->setCurrentIndex(pageId);
	});

    QVBoxLayout* vComboBoxLayout = new QVBoxLayout;
    vComboBoxLayout->addWidget(m_taskTypeComboBox);
    vComboBoxLayout->addStretch();
    vComboBoxLayout->setMargin(10);

	QHBoxLayout* hTaskChooseLayout = new QHBoxLayout;
	hTaskChooseLayout->addLayout(vComboBoxLayout);
	hTaskChooseLayout->addWidget(m_stackedWidget);
	hTaskChooseLayout->setSpacing(15);
	hTaskChooseLayout->setMargin(10);

	QPushButton* pButtonOk = new QPushButton("确定");
	pButtonOk->setFixedSize(QSize(70, 25));
	pButtonOk->setObjectName("CommonButton");

	connect(pButtonOk, &QPushButton::clicked, this, [=] {
        PeriodTaskType taskType = PeriodTaskType(m_taskTypeComboBox->currentIndex());
        emit signalOKButtonClikced(taskType);
	});

	QPushButton* pButtonCancel = new QPushButton("取消");
	pButtonCancel->setFixedSize(QSize(70, 25));
	pButtonCancel->setObjectName("CommonButton");

	connect(pButtonCancel, &QPushButton::clicked, this, [=] {
		this->close();
	});

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(pButtonOk);
	hButtonLayout->addWidget(pButtonCancel);
	hButtonLayout->setSpacing(15);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vCenterLayout = new QVBoxLayout(this->getCenterWidget());
	vCenterLayout->addLayout(hTaskChooseLayout);
    vCenterLayout->addStretch();
	vCenterLayout->addLayout(hButtonLayout);
	vCenterLayout->setSpacing(20);
	vCenterLayout->setMargin(10);
}

DLWheelImmediatelyExcuteWindow::DLWheelImmediatelyExcuteWindow(QWidget* parent /*= NULL*/)
    : BaseWidget(parent, PopupWindow)
{
    initWidget();
    this->setTitleContent("立即执行");
    this->setAttribute(Qt::WA_DeleteOnClose);
    this->setFixedSize(QSize(350, 300));
    this->setStyleSheet("QPushButton#CommonButton{font-size:12px;font-family:Microsoft Yahei;color:white;background-color:rgb(145, 167, 255);border-radius:3px;}\
						QPushButton#CommonButton:pressed{padding-left:2px;padding-top:2px;}\
						QWidget#TitleBackWidget{border-top:3px solid rgb(145, 167, 255); }\
								QComboBox{color:black;background:white;padding-left:5px;border-radius:3px;border:1px solid gray;}\
								QComboBox:hover{border: 1px solid rgb(21 , 131 , 221);}\
								QComboBox QAbstractItemView::item{height:30px;}\
								QComboBox::down-arrow{border-image:url(:/Resources/Common/image/arrow_Down.png);height:10px;width:13px;}\
								QComboBox::down-arrow:on{border-image:url(:/Resources/Common/image/arrow_Up.png);height:10px;width:13px;}\
								QComboBox::drop-down{width:20px;background:transparent;padding-right:5px;}");
}

DLWheelImmediatelyExcuteWindow::~DLWheelImmediatelyExcuteWindow()
{

}

void DLWheelImmediatelyExcuteWindow::setImmediatelyTaskInfo(WheelRobotAssignTask task)
{
    m_task = task;
}

void DLWheelImmediatelyExcuteWindow::initWidget()
{
    m_isOptimizeDeviceCheckBox = new QCheckBox("是否优化设备顺序");
	m_isOptimizeDeviceCheckBox->setChecked(true);

    m_isBreakCurrentTaskCheckBox = new QCheckBox("是否打断当前任务");

    QLabel* taskPriorityLabel = new QLabel("任务优先级");
    m_taskPriorityComboBox = new QComboBox();
    m_taskPriorityComboBox->addItems(QStringList() << "优先级一" << "优先级二" << "优先级三" << "优先级四" << "优先级五");

    QHBoxLayout* hTaskPriorityLayout = new QHBoxLayout;
    hTaskPriorityLayout->addWidget(taskPriorityLabel);
    hTaskPriorityLayout->addWidget(m_taskPriorityComboBox);
    hTaskPriorityLayout->setSpacing(10);
    hTaskPriorityLayout->setMargin(0);

    QLabel* taskOverActionLabel = new QLabel("任务结束动作");
    m_taskOverActionComboBox = new QComboBox;
    m_taskOverActionComboBox->addItems(QStringList() << "留在原地" << "返回充电");

    QHBoxLayout* hTaskOverActionLayout = new QHBoxLayout;
    hTaskOverActionLayout->addWidget(taskOverActionLabel);
    hTaskOverActionLayout->addWidget(m_taskOverActionComboBox);
    hTaskOverActionLayout->setSpacing(10);
    hTaskOverActionLayout->setMargin(0);
    
    QPushButton* pButtonOk = new QPushButton("确定");
    pButtonOk->setFixedSize(QSize(70, 25));
    pButtonOk->setObjectName("CommonButton");

    connect(pButtonOk, &QPushButton::clicked, this, [=] {
        getImmediatelyExcuteInfo();
        signalOKButtonClikced(m_task);
        accepted();
        close();
    });

    QPushButton* pButtonCancel = new QPushButton("取消");
    pButtonCancel->setFixedSize(QSize(70, 25));
    pButtonCancel->setObjectName("CommonButton");

    connect(pButtonCancel, &QPushButton::clicked, this, [=] {
        reject();
        close();
    });

    QHBoxLayout* hButtonLayout = new QHBoxLayout;
    hButtonLayout->addStretch();
    hButtonLayout->addWidget(pButtonOk);
    hButtonLayout->addWidget(pButtonCancel);
    hButtonLayout->setSpacing(15);
    hButtonLayout->setMargin(0);

    QVBoxLayout* vMainLayout = new QVBoxLayout(this->getCenterWidget());
    vMainLayout->addWidget(m_isOptimizeDeviceCheckBox);
    vMainLayout->addWidget(m_isBreakCurrentTaskCheckBox);
    vMainLayout->addLayout(hTaskPriorityLayout);
    vMainLayout->addLayout(hTaskOverActionLayout);
    vMainLayout->addLayout(hButtonLayout);
    vMainLayout->setSpacing(10);
    vMainLayout->setContentsMargins(50, 10, 50, 0);
}

void DLWheelImmediatelyExcuteWindow::getImmediatelyExcuteInfo()
{
    m_task.dev_optimize = m_isOptimizeDeviceCheckBox->isChecked();
    m_task.breakTask = m_isBreakCurrentTaskCheckBox->isChecked();

    m_task.priority = WheelRobotTaskpriority(m_taskPriorityComboBox->currentIndex());
    m_task.task_end_action = WheelRobotTaskEndActionType(m_taskOverActionComboBox->currentIndex() + 1);
}
