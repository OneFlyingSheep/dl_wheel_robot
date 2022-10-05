#ifndef BASE_WIDGET_H
#define BASE_WIDGET_H

#include <QDialog>
#include <QLabel>
#include <QCheckBox>
#include <QPushButton>
#include <QStackedWidget>
#include <QButtonGroup>
#include <QTreeView>
#include <QStandardItemModel>
#include <QComboBox>
#include <QTimer>

#pragma execution_character_set("utf-8") 

enum BaseWidgetType
{
	CommonWidget = 0,			// 一般窗口;
	VideoSwitchWidget,			// 视频监控视频切换窗口(带checkBox);
	PatrolWidget,				// 智能巡检窗口;
	StackedWidget,				// 层叠窗口;
	RobotControlWidget,			// 机器人控制;
	TreeViewWindow,				// 智能巡检树控件窗口;
	PopupWindow,				// 弹出式窗口;
	EnvironmentStateWidget,		// 环境状态曲线图窗口;
	TableWidgetWithTrack,		// 带跟踪的tableWidget;
};

enum IntelligentPatrolButtonType
{
	PatrolOverAll,							// 全面巡检;
	PatrolOrienteering,						// 定向巡检;
	PatrolSpecific,							// 特制巡检;
	PatrolPauseTask,						// 暂停任务;
	PatrolStopTask							// 停止任务;
};

enum EnvironmentStateType
{
	TemperatureState,						// 温度;
	TumidityState,							// 湿度;
	SF6State,								// SF6;
	O3State,								// O3;
};

class BaseWidget : public QDialog
{
	Q_OBJECT

public:
	BaseWidget(QWidget *parent = NULL, BaseWidgetType baseWidgetType = CommonWidget);
	~BaseWidget();

	// 设置标题内容;
	void setTitleContent(QString strContent);
	// 设置标题文字颜色;
	void setTitleColor(QColor titleColor);
	// 设置标题背景色;
	void setTitleBackColor(QColor titleBackColor);
	// 获取中心部件指针;
	QWidget* getCenterWidget();
	// 向centerWidget插入widget,针对于stackedWidget类型的窗口;
	void insertWidget(int buttonId, QString buttonText, QWidget* widget);
	// 插入完毕,必须调用;
	void insertWidgetDone();
	// 设置是否绘制边框线;
	void setDrawBorderLine(bool isDrawBorderLine = true);

	// 设置弹出式窗口中树控件的model;
	void setTreeViewModel(QStandardItemModel* model);

	// 获取智能巡检中checkBox状态;
	bool getPatrolWidgetCheckBoxState();
	
	// 返回树控件;
	QTreeView* getPatrolTreeWidget();

	// 设置任务进度;
	void setTaskProcess(QString taskProcess);

    // 设置下次任务倒计时时间;
    void setNextTaskCountDown(int iCountDown);

	// 设置智能巡检暂停任务按钮状态;
	void setStopTaskButtonState(bool isStopTaskClicked);

	// 获取智能巡检暂停任务按钮状态;
	bool getStopTaskButtonState();

	// StackedWidget类型窗口跳转至某一页;
	void setCurrentStackedWidgetPage(int pageIndex);

	// 设置是否显示右上角关闭按钮;
	void setShowCloseButton(bool isShow = true);

	// 获取tilteWidget(用于自定义标题栏上控件);
	QWidget* getTitleBackWidget();

	// 获取当前StackedWidget的pageIndex;
	int getCurrentPageIndex();

    // 设置窗口是否可以移动;
    void setMovable(bool isMovable);

    void setTitleHide();
signals:
	// 视频切换状态改变;
	void signalCheckBoxStateChanged(bool isChecked);

	// 机器人控制-全部应用按钮点击;
	void signalRobotControlAllApplyClicked();

	// 智能巡检标题栏按钮点击;
	void signalPatrolButtonClicked(int);

	// 通知发送新任务;
	void signalSendNewCustomTask(QString taskName, QStringList taskIdList);

	// 环境状态曲线图选择状态;
	void signalEnvironmentStateChanged(int index);

	// 弹出式窗口关闭按钮点击;
	void signalCloseButtonClicked();

	// StackedWidget窗口切换;
	void signalStackedWidgetChanged(int button);

public slots:
	// 视频窗口切换;
	void onCheckBoxStateChanged(int state);
	// stackedWidget页面切换;
	void onStackedButtonClicked(int buttonId);
private:
	// 窗口控件初始化;
	void initControl();
	// 初始化一般窗口;
	void initCommonWidget();
	// 初始化视频切换窗口;
	void initVideoSwitchWidget();
	// 智能巡检窗口;
	void initPatrolWidget();
	// 机器人控制;
	void initRobotControlWidget();
	// stackedWidget;
	void initStackedWidget();
	// 初始化弹出式窗口;
	void initTreeViewWindow();
	// 弹出式窗口;;
	void initPopupWindow();
	// 环境状态曲线图窗口;
	void initEnvironmentStateWidget();
	// 带跟踪的tableWidget;
	void initTableWidgetWithTrack();

	// 绘制事件;
	void paintEvent(QPaintEvent *event);

	// 按键事件;
	void keyPressEvent(QKeyEvent *event);

	// 拖动操作事件;
	void mousePressEvent(QMouseEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);

protected:
	QLabel * m_titleLabel;
	QWidget* m_titleBackWidget;
	QWidget* m_centerWidget;
	// 视频切换窗口;
	QCheckBox* m_checkeBox;

	// 智能巡检;
	QPushButton* m_pButtonPatrolOverAll;
	QPushButton* m_pButtonPatrolOrienteering;
	QPushButton* m_pButtonPatrolSpecific;
	QPushButton* m_pButtonPatrolPauseTask;
	QPushButton* m_pButtonPatrolStopTask;
	QCheckBox* m_chechBoxTrack;
	QLabel* m_taskProcessLabel;
    QLabel* m_nextTaskCountDownLabel;

	// 机器人控制;
	// 全部应用;
	QPushButton* m_pButtonAllApply;

	// stackedWidget;
	QStackedWidget* m_centerStackedWidget;
	QButtonGroup* m_stackedButtonGroup;

	// 部件类型;
	BaseWidgetType m_baseWidgetType;

	// 弹出式窗口控件;
	QTreeView* m_pupUpTreeView;
	QPushButton* m_pButtonOk;
	QPushButton* m_pButtonCancel;

	// 环境状态曲线图窗口添加下拉列表选择某个环境;
	QComboBox* m_environmentStateComboBox;

	// 是否绘制边框线;
	bool m_isDrawBorderLine;

	// 是否是弹出式窗口;
	bool m_isPopupWindow;

	// 当前是否暂停任务(智能巡检);
	bool m_isStopTaskClicked;

	// 鼠标左键是否按下,用于鼠标拖动窗口;
	bool m_isPressed;
	QPoint m_startMovePos;

    // 关闭按钮;
	QPushButton* m_pButtonClose;
};

enum MessageButtonType
{
	BUTTON_OK = 0,					// 只有确定按钮;
	BUTTON_OK_AND_CANCEL,			// 确定、取消按钮;
	BUTTON_CLOSE,					// 关闭按钮;
    BUTTON_NONE                     // 没有按钮;
};

// 消息提示框;
class DLMessageBox : public BaseWidget
{
	Q_OBJECT

public:
	DLMessageBox(QWidget* parent = NULL);
	~DLMessageBox();

	// 设置MessageBox内容;
	void setMessageContent(QString strContent);

    // 是否显示OK按钮;
    void setButtonOKVisible(bool isVisible);

	// 是否显示cancel按钮;
	void setButtonCancelVisible(bool isVisible);

	void setButtonType(MessageButtonType buttonType);

	int static showDLMessageBox(QWidget* parent, const QString &title, const QString &contentText, MessageButtonType messageButtonType, bool isModelWindow = false, QSize windowSize = QSize(300, 180));

signals:
	void signalButtonOkClicked();

private:
	void initWidget();

protected:
	QLabel* m_tipTextLabel;
	QLabel* m_tipIconLabel;
};

// 调用Core操作发送命令阻塞窗口;
class BlockMessageBox : public DLMessageBox
{
    Q_OBJECT
public:
    BlockMessageBox(QWidget* parent = NULL)
        : DLMessageBox(parent)
    {
        this->setWindowTitle("提示");
        this->setWindowModality(Qt::ApplicationModal);
        this->setFixedSize(QSize(250, 160));
        this->setMovable(false);
        initTimer();
        connect(m_pButtonClose, &QPushButton::clicked, this, &BlockMessageBox::hide);
        m_tipTextLabel->setStyleSheet("QLabel{font-size:20px;}");

    }

    // 单位秒(需要在show之前调用);
    void setBlockTime(int timeCount)
    {
        m_operateTimer.setInterval(timeCount * 1000);
    }

    void show()
    {
        this->setMessageContent("操作进行中");
        this->setButtonType(BUTTON_NONE);
        m_tipTextLabel->setStyleSheet("QLabel{font-size:20px;color:black;}");
        m_operateTimer.start();
        m_showTimer.start();
    }

    void hide()
    {
        m_showTimer.stop();
        m_operateTimer.stop();
        QWidget::hide();
    }

private:
    void initTimer()
    {
        m_operateTimer.setInterval(5 * 1000);
        connect(&m_operateTimer, &QTimer::timeout, this, [=] {
            this->setMessageContent("操作超时");
            m_tipTextLabel->setStyleSheet("QLabel{font-size:20px;color:red;}");
            this->setButtonType(BUTTON_CLOSE);
        });

        m_showTimer.setInterval(300);
        connect(&m_showTimer, &QTimer::timeout, this, [=] {
            QWidget::show();
        });
    }

private:
    QTimer m_operateTimer;
    QTimer m_showTimer;
};

#endif // BASE_WIDGET_H