#include "BaseWidget.h"
#include <QPainter>
#include <QHBoxLayout>
#include <QKeyEvent>

#define TITLE_WIDGET_HEIGHT 40

DLMessageBox::DLMessageBox(QWidget* parent)
	: BaseWidget(parent, PopupWindow)
{
	initWidget();
	this->setFixedSize(QSize(320, 180));
	this->setAttribute(Qt::WA_DeleteOnClose);
}

DLMessageBox::~DLMessageBox()
{

}

void DLMessageBox::initWidget()
{
	this->setTitleContent("提示");
	m_tipIconLabel = new QLabel;
	m_tipIconLabel->setFixedSize(QSize(48, 48));
	m_tipIconLabel->setPixmap(QPixmap(":/Resources/TipIcon.png"));

	m_tipTextLabel = new QLabel;
	m_tipTextLabel->setFont(QFont("Microsoft YaHei", 13));
	m_tipTextLabel->setWordWrap(true);

	m_pButtonOk = new QPushButton("确定");
	m_pButtonOk->setObjectName("PatrolButton");
	m_pButtonOk->setFixedSize(QSize(60, 30));

	connect(m_pButtonOk, &QPushButton::clicked, this, [=] {
		emit signalButtonOkClicked();
		accept();
		close();
	});

	m_pButtonCancel = new QPushButton("取消");
	m_pButtonCancel->setObjectName("PatrolButton");
	m_pButtonCancel->setFixedSize(QSize(60, 30));
	m_pButtonCancel->setVisible(false);

	connect(m_pButtonCancel, &QPushButton::clicked, this, [=] {
		reject();
		close();
	});
	QHBoxLayout* hContentLayout = new QHBoxLayout;
	hContentLayout->addWidget(m_tipIconLabel);
	hContentLayout->addStretch();
	hContentLayout->addWidget(m_tipTextLabel);
	hContentLayout->addStretch();
	hContentLayout->setSpacing(10);
	hContentLayout->setMargin(0);

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(m_pButtonOk);
	hButtonLayout->addWidget(m_pButtonCancel);
	hButtonLayout->setSpacing(10);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vMainLayout = new QVBoxLayout(this->getCenterWidget());
	vMainLayout->addLayout(hContentLayout);
	vMainLayout->addLayout(hButtonLayout);
	vMainLayout->setSpacing(20);
	vMainLayout->setContentsMargins(20, 25, 20, 10);
}

void DLMessageBox::setMessageContent(QString strContent)
{
	m_tipTextLabel->setText(strContent);
}

void DLMessageBox::setButtonOKVisible(bool isVisible)
{
    m_pButtonOk->setVisible(isVisible);
}

void DLMessageBox::setButtonCancelVisible(bool isVisible)
{
	m_pButtonCancel->setVisible(isVisible);
}

void DLMessageBox::setButtonType(MessageButtonType buttonType)
{
	switch (buttonType)
	{
	case BUTTON_OK:
	{
		m_pButtonCancel->setVisible(false);
	}
		break;
	case BUTTON_OK_AND_CANCEL:
	{
		m_pButtonOk->setVisible(true);
		m_pButtonCancel->setVisible(true);
	}
		break;
	case BUTTON_CLOSE:
	{
		m_pButtonOk->setVisible(false);
		m_pButtonCancel->setVisible(false);
        m_pButtonClose->setVisible(true);
	}
        break;
    case BUTTON_NONE:
    {
        m_pButtonOk->setVisible(false);
        m_pButtonCancel->setVisible(false);
        m_pButtonClose->setVisible(false);
    }
		break;
	default:
		break;
	}
}

int DLMessageBox::showDLMessageBox(QWidget* parent, const QString &title, const QString &contentText, MessageButtonType messageButtonType, bool isModelWindow /* = false */, QSize windowSize /* = QSize(300, 180) */)
{
	DLMessageBox * myMessageBox = new DLMessageBox(parent);
	myMessageBox->setWindowTitle(title);
	myMessageBox->setMessageContent(contentText);
	myMessageBox->setButtonType(messageButtonType);
    myMessageBox->setFixedSize(windowSize);
	if (isModelWindow)
	{
		// 设置为模态窗口时，参数parent必须设置父窗口指针，否则模态设置无效;
		// 因为 Qt::WindowModal 参数只对父窗口有效，如果想要模态对全局窗口都有效可以设置 Qt::ApplicationModal
		return myMessageBox->exec();
	}
	else
	{
		myMessageBox->show();
	}

	return -1;
}

BaseWidget::BaseWidget(QWidget *parent, BaseWidgetType baseWidgetType)
	: QDialog(parent)
	, m_baseWidgetType(baseWidgetType)
	, m_isDrawBorderLine(false)
	, m_pupUpTreeView(NULL)
	, m_isPopupWindow(false)
	, m_chechBoxTrack(NULL)
	, m_taskProcessLabel(NULL)
    , m_nextTaskCountDownLabel(NULL)
	, m_isStopTaskClicked(false)
	, m_centerStackedWidget(NULL)
	, m_stackedButtonGroup(NULL)
	, m_isPressed(false)
	, m_pButtonClose(NULL)
	, m_titleBackWidget(NULL)
{
	initControl();

	setTitleColor(QColor(98, 98, 98));
	setTitleBackColor(Qt::white);

	this->setStyleSheet("*{font-family:Microsoft Yahei;}\
						QPushButton#PatrolButton{font-family:Microsoft Yahei;color:white;background-color:rgb(14 , 150 , 254);border-radius:3px;}\
						QPushButton#PatrolButton:hover{background-color:rgb(44 , 137 , 255);}\
						QPushButton#PatrolButton:pressed{background-color:rgb(14 , 135 , 228);padding-left:2px;padding-top:2px;}\
						QScrollBar:vertical{background: white;width:10px;margin: 10px 0px 10 0px;}\
						QScrollBar::handle:vertical{border-radius:3px;background:lightgray;border-width:2 0 2 0;}\
						QScrollBar::sub-line:vertical{border-image:url(:/Resources/common/arrowTop.png);height:10px;subcontrol-origin:margin;}\
						QScrollBar::add-line:vertical{border-image:url(:/Resources/common/arrowBottom.png);height:10px;subcontrol-origin:margin;}\
						QScrollBar:horizontal{background:white; height: 8px; }\
						QScrollBar::handle:horizontal{border-radius:3px;background:gray;}");

	if (m_baseWidgetType == PopupWindow)
	{
		this->setWindowFlags(Qt::FramelessWindowHint | Qt::Tool | Qt::WindowStaysOnTopHint);
	}
	else
	{
		this->setWindowFlags(Qt::FramelessWindowHint);
	}
}

BaseWidget::~BaseWidget()
{
}

void BaseWidget::initCommonWidget()
{
	QHBoxLayout* hLayout = new QHBoxLayout(m_titleBackWidget);
	hLayout->addWidget(m_titleLabel);
	hLayout->addStretch();
	hLayout->setMargin(0);
	hLayout->setSpacing(15);
	hLayout->setContentsMargins(20, 0, 20, 0);
}

void BaseWidget::initVideoSwitchWidget()
{
	m_checkeBox = new QCheckBox;
	m_checkeBox->setText(("切换视频"));
	m_checkeBox->setCheckable(true);
	m_checkeBox->setChecked(true);
	m_checkeBox->setStyleSheet("QCheckBox{color:rgb(101 , 101 , 101);}\
								QCheckBox::indicator:unchecked{border-image:url(:/Resources/checkbox.png) 0 39 0 0;}\
								QCheckBox::indicator:hover{border-image:url(:/Resources/checkbox.png) 0 26 0 13;}\
								QCheckBox::indicator:pressed{border-image:url(:/Resources/checkbox.png) 0 13 0 26;}\
								QCheckBox::indicator:checked{border-image:url(:/Resources/checkbox.png) 0 0 0 39;}");

	connect(m_checkeBox, SIGNAL(stateChanged(int)), this, SLOT(onCheckBoxStateChanged(int)));

	QHBoxLayout* hLayout = new QHBoxLayout(m_titleBackWidget);
	hLayout->addWidget(m_titleLabel);
	hLayout->addWidget(m_checkeBox);
	hLayout->addStretch();
	hLayout->setMargin(0);
	hLayout->setSpacing(20);
	hLayout->setContentsMargins(20, 0, 20, 0);
}

void BaseWidget::initPatrolWidget()
{
	QButtonGroup* patrolButtonGroup = new QButtonGroup(this);
	// 智能巡检;
	m_pButtonPatrolOverAll = new QPushButton;
	m_pButtonPatrolOverAll->setText(("全面巡检"));
	m_pButtonPatrolOverAll->setObjectName("PatrolButton");
	m_pButtonPatrolOverAll->setFixedSize(QSize(75, 25));
	patrolButtonGroup->addButton(m_pButtonPatrolOverAll, IntelligentPatrolButtonType::PatrolOverAll);

	m_pButtonPatrolOrienteering = new QPushButton;
	m_pButtonPatrolOrienteering->setText(("定向巡检"));
	m_pButtonPatrolOrienteering->setObjectName("PatrolButton");
	m_pButtonPatrolOrienteering->setFixedSize(QSize(75, 25));
	patrolButtonGroup->addButton(m_pButtonPatrolOrienteering, IntelligentPatrolButtonType::PatrolOrienteering);

	m_pButtonPatrolSpecific = new QPushButton;
	m_pButtonPatrolSpecific->setText(("特制巡检"));
	m_pButtonPatrolSpecific->setObjectName("PatrolButton");
	m_pButtonPatrolSpecific->setFixedSize(QSize(75, 25));
	patrolButtonGroup->addButton(m_pButtonPatrolSpecific, IntelligentPatrolButtonType::PatrolSpecific);

	m_taskProcessLabel = new QLabel;

    m_nextTaskCountDownLabel = new QLabel("下次任务倒计时: 0 S");

	m_pButtonPatrolPauseTask = new QPushButton;
	m_pButtonPatrolPauseTask->setText(("暂停任务"));
	m_pButtonPatrolPauseTask->setObjectName("PatrolButton");
	m_pButtonPatrolPauseTask->setFixedSize(QSize(75, 25));
	patrolButtonGroup->addButton(m_pButtonPatrolPauseTask, IntelligentPatrolButtonType::PatrolPauseTask);

	m_pButtonPatrolStopTask = new QPushButton;
	m_pButtonPatrolStopTask->setText(("停止任务"));
	m_pButtonPatrolStopTask->setObjectName("PatrolButton");
	m_pButtonPatrolStopTask->setFixedSize(QSize(75, 25));
	patrolButtonGroup->addButton(m_pButtonPatrolStopTask, IntelligentPatrolButtonType::PatrolStopTask);

	connect(patrolButtonGroup, SIGNAL(buttonClicked(int)), this, SIGNAL(signalPatrolButtonClicked(int)));

	m_chechBoxTrack = new QCheckBox;
	m_chechBoxTrack->setText(("跟踪"));
	m_chechBoxTrack->setStyleSheet("QCheckBox{color:rgb(101 , 101 , 101);}\
								QCheckBox::indicator:unchecked{border-image:url(:/Resources/checkbox.png) 0 39 0 0;}\
								QCheckBox::indicator:hover{border-image:url(:/Resources/checkbox.png) 0 26 0 13;}\
								QCheckBox::indicator:pressed{border-image:url(:/Resources/checkbox.png) 0 13 0 26;}\
								QCheckBox::indicator:checked{border-image:url(:/Resources/checkbox.png) 0 0 0 39;}");

	QHBoxLayout* patrolHLayout = new QHBoxLayout(m_titleBackWidget);
	patrolHLayout->addWidget(m_pButtonPatrolOverAll);
	patrolHLayout->addWidget(m_pButtonPatrolOrienteering);
	patrolHLayout->addWidget(m_pButtonPatrolSpecific);
	patrolHLayout->addWidget(m_taskProcessLabel);
	patrolHLayout->addStretch();
    patrolHLayout->addWidget(m_nextTaskCountDownLabel);
	patrolHLayout->addWidget(m_pButtonPatrolPauseTask);
	patrolHLayout->addWidget(m_pButtonPatrolStopTask);
	patrolHLayout->addWidget(m_chechBoxTrack);
	patrolHLayout->setSpacing(10);
	patrolHLayout->setMargin(0);
	patrolHLayout->setContentsMargins(20, 0, 20, 0);
}

void BaseWidget::initRobotControlWidget()
{
	// 智能巡检;
	m_pButtonAllApply = new QPushButton;
	m_pButtonAllApply->setObjectName("PatrolButton");
	m_pButtonAllApply->setText(("全部应用"));
	m_pButtonAllApply->setFixedSize(QSize(75, 25));

	connect(m_pButtonAllApply, SIGNAL(clicked()), this, SIGNAL(signalRobotControlAllApplyClicked()));

	QHBoxLayout* robotControlHLayout = new QHBoxLayout(m_titleBackWidget);
	robotControlHLayout->addWidget(m_titleLabel);
	robotControlHLayout->addStretch();
	robotControlHLayout->addWidget(m_pButtonAllApply);
	robotControlHLayout->setSpacing(10);
	robotControlHLayout->setMargin(0);
	robotControlHLayout->setContentsMargins(20, 0, 20, 0);
}

void BaseWidget::initStackedWidget()
{
	// 用来放置按钮组;
	m_stackedButtonGroup = new QButtonGroup(this);
	connect(m_stackedButtonGroup, SIGNAL(buttonClicked(int)), this, SLOT(onStackedButtonClicked(int)));

	QHBoxLayout* hLayout = new QHBoxLayout(m_titleBackWidget);
	hLayout->setMargin(0);
	hLayout->setSpacing(0);
	hLayout->setContentsMargins(20, 0, 20, 0);

	m_centerStackedWidget = new QStackedWidget;
	QVBoxLayout* vLayout = new QVBoxLayout(m_centerWidget);
	vLayout->addWidget(m_centerStackedWidget);
	vLayout->setSpacing(0);
	vLayout->setMargin(10);
}

void BaseWidget::initTreeViewWindow()
{
	m_isPopupWindow = true;

	QHBoxLayout* hLayout = new QHBoxLayout(m_titleBackWidget);
	hLayout->addWidget(m_titleLabel);
	hLayout->addStretch();
	hLayout->setMargin(0);
	hLayout->setSpacing(20);
	hLayout->setContentsMargins(20, 0, 20, 0);

	m_pupUpTreeView = new QTreeView();
	m_pupUpTreeView->setHeaderHidden(true);
	m_pupUpTreeView->setStyleSheet("QTreeView{\
									border: 1px solid lightgray;\
									}\
									QTreeView::item {\
											height: 25px;\
											border: none;\
											background: transparent;\
											color: black;\
											outline:none;\
									}\
									QTreeView::item:hover{\
											background: rgba(33, 150, 243, 80);\
									}\
									QTreeView::branch:open:has-children {\
											image: url(:/Resources/TreeView/branch_Open.png);\
									}\
									QTreeView::branch:closed:has-children {\
											image: url(:/Resources/TreeView/branch_Close.png);\
									QCheckBox{background:red;}\
									}");

	m_pButtonOk = new QPushButton(("确定"));
	m_pButtonOk->setFixedSize(QSize(90, 30));
	m_pButtonOk->setObjectName("PatrolButton");

	m_pButtonCancel = new QPushButton(("取消"));
	m_pButtonCancel->setFixedSize(QSize(90, 30));
	m_pButtonCancel->setObjectName("PatrolButton");

	QHBoxLayout* hButtonLayout = new QHBoxLayout;
	hButtonLayout->addStretch();
	hButtonLayout->addWidget(m_pButtonOk);
	hButtonLayout->addWidget(m_pButtonCancel);
	hButtonLayout->setSpacing(20);
	hButtonLayout->setMargin(0);

	QVBoxLayout* vCenterLayout = new QVBoxLayout(m_centerWidget);
	vCenterLayout->addWidget(m_pupUpTreeView);
	vCenterLayout->addLayout(hButtonLayout);
	vCenterLayout->setSpacing(20);
	vCenterLayout->setMargin(20);

	connect(m_pButtonOk, &QPushButton::clicked, this, [=]() {
		QStringList taskIdList;
		QStandardItemModel* treeModel = static_cast<QStandardItemModel*>(m_pupUpTreeView->model());
		for (int row = 0; row < treeModel->rowCount(); row++)
		{
			QStandardItem* rootItem = treeModel->item(row);
			for (int rootRow = 0; rootRow < rootItem->rowCount(); rootRow++)
			{
				QStandardItem* childItem = rootItem->child(rootRow);
				if (childItem->checkState() == Qt::Checked)
				{
					QString strData = childItem->data(Qt::UserRole).toString();
					taskIdList.append(strData);
				}
			}
		}

		if (!taskIdList.isEmpty())
		{
			emit signalSendNewCustomTask(m_titleLabel->text(),taskIdList);
		}
		hide();
	});

	connect(m_pButtonCancel, &QPushButton::clicked, this, [=]() {
		hide();
	});

	this->setDrawBorderLine();
}

void BaseWidget::initPopupWindow()
{
    m_isPopupWindow = true;

	m_pButtonClose = new QPushButton;
	m_pButtonClose->setObjectName("closeButton");
	m_pButtonClose->setIcon(QIcon(":/Resources/close.png"));
	m_pButtonClose->setIconSize(QSize(20, 20));
	m_pButtonClose->setStyleSheet("QPushButton{border:none;}QPushButton:pressed{padding-left:3px;padding-top:3px;}");
	connect(m_pButtonClose, &QPushButton::clicked, this, &BaseWidget::signalCloseButtonClicked);

	QHBoxLayout* hLayout = new QHBoxLayout(m_titleBackWidget);
	hLayout->addWidget(m_titleLabel);
	hLayout->addStretch();
	hLayout->addWidget(m_pButtonClose);
	hLayout->setContentsMargins(0, 0, 0, 0);
	hLayout->setSpacing(20);
	hLayout->setContentsMargins(20, 0, 20, 0);
	this->setDrawBorderLine();

	m_pButtonClose->setVisible(false);
}

void BaseWidget::initEnvironmentStateWidget()
{
	QStringList environmentState = QStringList() << "温度" << "湿度" <<"SF6" << "03";
	m_environmentStateComboBox = new QComboBox;
	m_environmentStateComboBox->addItems(environmentState);
	m_environmentStateComboBox->setFixedSize(QSize(60, 25));
	m_environmentStateComboBox->setStyleSheet("QComboBox{margin-top:5px;color:black;background:white;padding-left:5px;border-radius:3px;border:1px solid gray;}\
								QComboBox:hover{border: 1px solid rgb(21 , 131 , 221);}\
								QComboBox QAbstractItemView::item{height:30px;}\
								QComboBox::down-arrow{border-image:url(:/Resources/arrow_Down.png);height:10px;width:13px;}\
								QComboBox::down-arrow:on{border-image:url(:/Resources/arrow_Up.png);height:10px;width:13px;}\
								QComboBox::drop-down{width:20px;background:transparent;padding-right:5px;}");

	connect(m_environmentStateComboBox, static_cast<void (QComboBox:: *)(int)>(&QComboBox::currentIndexChanged), this, [=](int index) {
		emit signalEnvironmentStateChanged(index);
	});

	QHBoxLayout* hLayout = new QHBoxLayout(m_titleBackWidget);
	hLayout->addWidget(m_titleLabel);
	hLayout->addWidget(m_environmentStateComboBox);
	hLayout->addStretch();
	hLayout->setMargin(0);
	hLayout->setSpacing(20);
	hLayout->setContentsMargins(20, 0, 20, 0);
}

void BaseWidget::initTableWidgetWithTrack()
{
	m_chechBoxTrack = new QCheckBox;
	m_chechBoxTrack->setText(("跟踪"));
	m_chechBoxTrack->setStyleSheet("QCheckBox{color:rgb(101 , 101 , 101);}\
								QCheckBox::indicator:unchecked{border-image:url(:/Resources/checkbox.png) 0 39 0 0;}\
								QCheckBox::indicator:hover{border-image:url(:/Resources/checkbox.png) 0 26 0 13;}\
								QCheckBox::indicator:pressed{border-image:url(:/Resources/checkbox.png) 0 13 0 26;}\
								QCheckBox::indicator:checked{border-image:url(:/Resources/checkbox.png) 0 0 0 39;}");

	QHBoxLayout* hLayout = new QHBoxLayout(m_titleBackWidget);
	hLayout->addWidget(m_titleLabel);
	hLayout->addStretch();
	hLayout->addWidget(m_chechBoxTrack);
	hLayout->setMargin(0);
	hLayout->setSpacing(20);
	hLayout->setContentsMargins(20, 0, 20, 0);
}

void BaseWidget::initControl()
{
	m_titleLabel = new QLabel;
	QFont font;
	font.setPointSize(12);
	m_titleLabel->setFont(font);

	m_titleBackWidget = new QWidget;
	m_titleBackWidget->setObjectName("TitleBackWidget");
	m_titleBackWidget->setFixedHeight(TITLE_WIDGET_HEIGHT);

	m_centerWidget = new QWidget;

	QVBoxLayout* vLayout = new QVBoxLayout(this);
	vLayout->addWidget(m_titleBackWidget);
	vLayout->addWidget(m_centerWidget);
	vLayout->setMargin(1);
	vLayout->setSpacing(0);

	// 针对每个不同的窗口，自定义标题栏;
	switch (m_baseWidgetType)
	{
	case CommonWidget:
		initCommonWidget();
		break;
	case VideoSwitchWidget:
		initVideoSwitchWidget();
		break;
	case PatrolWidget:
		initPatrolWidget();
		break;
	case StackedWidget:
		initStackedWidget();
		break;
	case RobotControlWidget:
		initRobotControlWidget();
		break;
	case TreeViewWindow:
		initTreeViewWindow();
		vLayout->setContentsMargins(1, 0, 1, 1);
		break;
	case PopupWindow:
		initPopupWindow();
		vLayout->setContentsMargins(1, 0, 1, 1);
		break;
	case EnvironmentStateWidget:
		initEnvironmentStateWidget();
		break;
	case TableWidgetWithTrack:
		initTableWidgetWithTrack();
		break;
	default:
		break;
	}

	return;
}

void BaseWidget::setTitleContent(QString strContent)
{
	m_titleLabel->setText(strContent);
}

void BaseWidget::setTitleColor(QColor titleColor)
{
	QString strStyleSheet = QString("QLabel{color:rgb(%1, %2, %3);}")\
		.arg(titleColor.red()).arg(titleColor.green()).arg(titleColor.blue());
	m_titleLabel->setStyleSheet(strStyleSheet);
}

void BaseWidget::setTitleBackColor(QColor titleBackColor)
{
	QString strStyleSheet = m_titleBackWidget->styleSheet();
	strStyleSheet.append(QString("QWidget#TitleBackWidget{background:rgb(%1, %2, %3);border-bottom:1px solid rgb(230, 230, 230);}")\
		.arg(titleBackColor.red()).arg(titleBackColor.green()).arg(titleBackColor.blue()));
	
	if (m_isPopupWindow)
	{
		strStyleSheet.append(QString("QWidget#TitleBackWidget{border-top:3px solid rgb(14, 150, 254); }"));
	}
	m_titleBackWidget->setStyleSheet(strStyleSheet);
}

QWidget* BaseWidget::getCenterWidget()
{
	return m_centerWidget;
}

void BaseWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	painter.fillRect(this->rect(), QBrush(Qt::white));

	if (m_isDrawBorderLine)
	{
		painter.setPen(QPen(QColor(230, 230, 230)));
		painter.drawRect(QRect(0, 0, this->width() - 1, this->height() - 1));
	}
}

void BaseWidget::onCheckBoxStateChanged(int state)
{
	if (state == Qt::Checked)
	{
		emit signalCheckBoxStateChanged(true);
	}
	else if (state == Qt::Unchecked)
	{
		emit signalCheckBoxStateChanged(false);
	}
}

void BaseWidget::insertWidget(int buttonId, QString buttonText, QWidget* widget)
{
	QPushButton* pushButton = new QPushButton(buttonText);
	pushButton->setFixedSize(QSize(120, 38));
	pushButton->setCheckable(true);
	pushButton->setStyleSheet("QPushButton{border:none;margin:0px;font-size:16px;}\
								QPushButton:hover{background:rgba(33, 150, 243, 100);}\
								QPushButton:checked{border-bottom:3px solid rgb(33, 150, 243);margin:0px;}");

	m_stackedButtonGroup->addButton(pushButton, buttonId);

	QHBoxLayout* hLayout = (QHBoxLayout*)m_titleBackWidget->layout();
	hLayout->addWidget(pushButton);

	m_centerStackedWidget->insertWidget(buttonId, widget);
}

void BaseWidget::insertWidgetDone()
{
	QList<QAbstractButton *> buttonList = m_stackedButtonGroup->buttons();
	if (buttonList.isEmpty())
	{
		return;
	}
	// 如果当前存在按钮,将第一个按钮置为checked状态;
	buttonList.first()->setChecked(true);

	QHBoxLayout* hLayout = (QHBoxLayout*)m_titleBackWidget->layout();
	hLayout->addStretch();
}

void BaseWidget::onStackedButtonClicked(int buttonId)
{
    m_stackedButtonGroup->button(buttonId)->setChecked(true);
	emit signalStackedWidgetChanged(buttonId);
    m_stackedButtonGroup->button(buttonId)->setChecked(true);
	QWidget* widget = m_centerStackedWidget->widget(buttonId);
	if (widget != NULL)
	{
		m_centerStackedWidget->setCurrentIndex(buttonId);
	}
}

void BaseWidget::setDrawBorderLine(bool isDrawBorderLine)
{
	m_isDrawBorderLine = isDrawBorderLine;
}

void BaseWidget::setTreeViewModel(QStandardItemModel* model)
{
	if (m_pupUpTreeView != NULL)
	{
		m_pupUpTreeView->setModel(model);
	}
}

bool BaseWidget::getPatrolWidgetCheckBoxState()
{
	if (m_chechBoxTrack != NULL)
	{
		return m_chechBoxTrack->isChecked();
	}

	return false;
}

QTreeView* BaseWidget::getPatrolTreeWidget()
{
	return m_pupUpTreeView;
}

void BaseWidget::setTaskProcess(QString taskProcess)
{
	if (m_taskProcessLabel != NULL)
	{
		m_taskProcessLabel->setText(taskProcess);
	}
}

void BaseWidget::setNextTaskCountDown(int iCountDown)
{
    if (m_nextTaskCountDownLabel != NULL)
    {
        m_nextTaskCountDownLabel->setText(QString("下次任务倒计时: %1 S").arg(iCountDown));
    }
}

void BaseWidget::setStopTaskButtonState(bool isStopTaskClicked)
{
	m_isStopTaskClicked = isStopTaskClicked;
	if (m_isStopTaskClicked)
	{
		m_pButtonPatrolPauseTask->setText("恢复任务");
	}
	else
	{
		m_pButtonPatrolPauseTask->setText("暂停任务");
	}
}

bool BaseWidget::getStopTaskButtonState()
{
	return m_isStopTaskClicked;
}

void BaseWidget::keyPressEvent(QKeyEvent *event)
{
	// 防止鼠标按下esc键，窗口自动关闭(QDialog属性);
	if (event->key() == Qt::Key_Escape)
	{
		return;
	}

	return __super::keyPressEvent(event);
}

void BaseWidget::setCurrentStackedWidgetPage(int pageIndex)
{
	if (m_centerStackedWidget != NULL && m_stackedButtonGroup != NULL)
	{
		QAbstractButton* pageButton = m_stackedButtonGroup->button(pageIndex);
		if (pageButton != NULL)
		{
			pageButton->click();
		}
	}
}

void BaseWidget::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == Qt::LeftButton && m_isPopupWindow)
	{
		m_isPressed = true;
		m_startMovePos = event->pos();
	}

	return QWidget::mousePressEvent(event);
}

void BaseWidget::mouseMoveEvent(QMouseEvent *event)
{
	if (m_isPressed)
	{
		QPoint movePoint = event->pos() - m_startMovePos;
		QPoint widgetPos = this->pos() + movePoint;
		this->move(widgetPos.x(), widgetPos.y());
	}
	return QWidget::mouseMoveEvent(event);
}

void BaseWidget::mouseReleaseEvent(QMouseEvent *event)
{
	m_isPressed = false;
	return QWidget::mouseReleaseEvent(event);
}

void BaseWidget::setShowCloseButton(bool isShow /* = true */)
{
	if (m_pButtonClose != NULL)
	{
		m_pButtonClose->setVisible(isShow);
	}
}

QWidget* BaseWidget::getTitleBackWidget()
{
	return m_titleBackWidget;
}

int BaseWidget::getCurrentPageIndex()
{
	return m_centerStackedWidget->currentIndex();
}

void BaseWidget::setMovable(bool isMovable)
{
    m_isPopupWindow = isMovable;
}

void BaseWidget::setTitleHide()
{
//    m_titleBackWidget->hide();
    m_titleBackWidget->setFixedHeight(30);
}
