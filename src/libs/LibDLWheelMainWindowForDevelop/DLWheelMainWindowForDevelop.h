#ifndef _WHEEL_MAINWINDOW_H
#define _WHEEL_MAINWINDOW_H

#include <QMainWindow>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>
#include <QSlider>
#include <QPainter>
#include <QListWidget>
#include <QTimer>
#include "common/DLWheelRobotGlobalDef.hpp"

class WelcomeWindow;
class ConfigWindow;
class CameraObject;
class LoginWindow;
class DLHangWheelCollectControlWidget;
class DLHangUserControl;
class DLWheelCollectMapWidget;
class DLWheelRobotStateShow;
class InfraredTemperature;
class DLWheelPowerStationEdit;
class BaseWidget;
class DLWheelVirtualDeviceWidget;
class ClearOutDateFileObject;
class ConfidenceValueShowWidget;
class DLWheelTaskManage;
class DLWheelPatrolResultBrowse;
class DLWheelStandardPointPosMaintain;
class DLWheelPatrolPointPosSet;
class InputWidget;

enum StackedPageType
{
	CollectControlWidget = 0,					// 采集页面;
	UserControlWidget,							// 用户管理;
	CustomTaskManageWidget,					   // 任务管理
	Robot_PatrolResultBrowse,					// 结果浏览	
	//CollectMapWidget,							// 采集地图;
	RobotStatus,								// 机器人状态;
	PowerStationEdit,							// 电站编辑;
	PatrolProEdit,								// 巡检点位编辑;
	PatrolPosSet,								// 巡检点位设置;
    VirtualDeviceWidget,                        // 虚拟设备;
};

class DLHangWheelMainWindowFroDevelop : public QWidget
{
	Q_OBJECT

public:
	DLHangWheelMainWindowFroDevelop(QWidget *parent = Q_NULLPTR);
	~DLHangWheelMainWindowFroDevelop();

protected:
	virtual bool eventFilter(QObject *watched, QEvent *event);

	
private:
	// 初始化窗口;
	void initWindow();
	// 初始化标题栏;
	void initTitleWidget();
	// 初始化左侧按钮;
	void initLeftButton();
	// 初始化stackedWidget；
	void initStackedWidget();
    // 初始化操作信息widget;
    void initOperateMsgWidget();

	// 初始化绑定所有Core的信号;
	void initCoreSignals();

    // 中转所有Core发送过来的信号;
    void initTransferFunction();

	// 初始化视频对象;
	void initCameraObject();

	// 初始化信号槽连接;
	void initConnections();

    // 初始化操作消息闪烁;
    void initOperateMessageFlickerTimer();

    // 初始化需要清理的文件;
    void initClearOutDateFile();

	// 登录成功;
	void loginSuccess();


	// 窗口关闭事件;
	void closeEvent(QCloseEvent *event);

signals:
	// 从MainWindow中发出信号去通知对应的页面（进行中转，以免在子线程中创建控件，导致报错）;
	
	// 更新任务展示中table数据删除结果;
	void signalUpdateDeleteResult(int, bool , QString);

	// 任务管理中保存任务编制返回结果;
	void signalTaskManageSaveRsp(int, bool, QString);

	// 任务管理中更新任务编制返回结果;
	void signalTaskManageUpdateRsp();

	//任务管理中是删除任务编制返回结果;
	void signalDeleteTaskCallBack();


	// Core通知删除设备结果;
	void signalDeleteDeviceResult(bool isSuccess, QString msg);

    // Core通知电站编辑插入结果;
    void signalStationEditInsertResult(bool isSuccess, QString strMsg);

    // Core通知电站编辑删除结果;
    void signalStationEditDeleteResult(bool isSuccess, QString strMsg);

    // Core 通知更新实时信息;
    void signalUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);
    // Core 通知更新非实时信息;
    void signalUpdateRobotNoneRealtimeStatus(WheelRobotNoneRealtimeStatus noneRealTimeStatus);

    // 通知刷新采集页面中的树控件;
    void signalRefreshTreeWidget();

	void UpdateEquipmentTreeSignal(bool, QStringList, RootNodeType);

    void DownloadImageSignal(QString strPath, QString strFileName, int type);

private slots:
	// 登录;
	void onLogin(QByteArray userName, QByteArray password, QString strIp, QString strPort);
	// 退出;
	void onQuit();

	void onLeftButtonClicked(int buttonId);

    // 获取到操作信息;
    void onGetOperateMsg(QString strMsg);

	void UpdateConfidenceValueSlot();				//刷新value

	void StutterStopBtnSlot();						//急停按钮的槽函数

private:
	// 图标最底层widget;
	QWidget * m_iconBackWidget;
	// 图标maskwidget;
	QWidget* m_iconBackMaskWidget;
	// 图标label;
	QLabel* m_iconLabel;
	// 左侧按钮背景widget;
	QWidget* m_leftBackWidget;
	// 标题栏背景widget;
	QWidget* m_titleBackWidget;
	// 中心背景widget;
	QWidget* m_centerBackWidget;

	//急停按钮
	QPushButton *m_pStutterStopBtn;

	//返回充电点
	QPushButton *m_back_charge_button;

	//终止任务
	QPushButton *m_stop_task_button;

	//	机器人切换按钮
	InputWidget* m_currentRobotWidget;

	// 关闭按钮;
	QPushButton* m_pButtonClose;

	// 启动界面;
	WelcomeWindow* m_welcomeWindow;

	// 可见光视频显示/操作对象;
	CameraObject* m_visibleLightCameraObject;
    // 红外视频显示/操作对象;
    CameraObject* m_infraredCameraObject;

	// 登录窗口;
	LoginWindow* m_loginWindow;

	// 红外对象;
//	InfraredTemperature* m_infraredTemperature;

	// 切换页widget;
	QStackedWidget* m_stackedWidget;
	// 采集控制;
	DLHangWheelCollectControlWidget* m_collectControlWidget;
	// 用户管理;
	DLHangUserControl* m_userControlWidget;

	// 任务管理;
	DLWheelTaskManage* m_robotTaskManageCustomTaskWidget;

	// 巡检结果浏览;
	DLWheelPatrolResultBrowse* m_patrolResultBrowse;

	// 巡检点位维护;
	DLWheelStandardPointPosMaintain* m_standardPointPosMaintain;

	// 巡检点位设置;
	DLWheelPatrolPointPosSet* m_patrolPointPosSet;

	// 采集地图;
	DLWheelCollectMapWidget* m_dLWheelCollectMapWidget;
	// 机器人状态;
	DLWheelRobotStateShow* m_robotStatusWidget;
	// 电站编辑;
	DLWheelPowerStationEdit* m_powerStationEdit;
    // 虚拟设备;
    DLWheelVirtualDeviceWidget* m_virtualDeviceWidget;

    // 置信度显示;
	QLabel *m_pConfidenValueLbl;
    ConfidenceValueShowWidget* m_pConfidenceValueShowWgt;

	//置信度图片
	QPixmap m_confidenceImages[2];
	int m_iConfidenceValue;					//置信度值

	ConfidenceValueShowWidget* m_pBatteryLevelWgt;	//电池电量
	bool m_bIsChargeState;
	float m_fCurvoltage;							//电量
	QLabel *m_pChargeState;
	QLabel *m_pBatteryLevelLbl;						//电池电量图标

    // 操作信息列表;
    BaseWidget* m_operateMsgListBackWidget;
    QListWidget* m_operateMsgListWidget;

    // 当前操作信息显示;
    QWidget* m_operateMessageBackWidget;
    QLabel* m_operateMessageLabel;

    // 消息闪烁时钟;
    QTimer m_operateMessageFlickerTimer;

    // 消息闪烁次数;
    int m_operateMessageFlickerCount;

    // 当前登录界面选择的屏幕index;
    int m_currentScreenChoosedIndex;

    // 定时清理对象;
    ClearOutDateFileObject* m_clearOutDateFileObject;

	bool m_bIsPressed;		//判断处理鼠标状态
	QPoint m_ptStartPos;

	QTimer *m_pUpdateConfidenceValueTimer;				//刷新置信度

	bool m_login = false;
};

#endif // MAINWINDOW_H
