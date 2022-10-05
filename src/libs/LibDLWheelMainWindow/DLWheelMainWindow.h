#pragma once

#include <QWidget>
#include <QToolButton>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>
#include <QTimer>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibRobotControlWidget/libRobotControlWidget.h"
// 不同页面枚举;
enum StackedWidgetType
{
	Robot_RealTimeManager = 0,				// 实时监控;
											// 任务管理;
	Robot_TaskManage_All_Patrol,						
	Robot_TaskManage_Routine_Patrol,
	Robot_TaskManage_InfraredCalculateTmp,
	Robot_TaskManage_OilTableRecord,
	Robot_TaskManage_ArresterTableRead,
	Robot_TaskManage_SF6PressureRecord,
	Robot_TaskManage_HydraumaticTableRecord,
	Robot_TaskManage_PosStateRecognition,
	Robot_TaskManage_BadWeatherPatrol,
	Robot_TaskManage_DefectTrack,
    Robot_TaskManage_DistanceAbnormalAlarmAffirm,
	Robot_TaskManage_DistanceStateAffirm,
	Robot_TaskManage_SecurityLinkage,
	Robot_TaskManage_AssistAccidentDeal,
	Robot_TaskManage_CustomTask,

	Robot_TaskShow,							// 任务展示;
	Robot_SoftSet,							// 软件设置;
	Robot_Set,								// 机器人设置;
	Robot_AlarmSearch,						// 机器人告警查询;
	Robot_StateShow,						// 机器人状态显示;
	Robot_IdentifyAbnormalPointSearch,		// 识别异常点位查询;
	Robot_PatrolPointPosSet,				// 巡检点位设置;
	Robot_DeviceAlarmSearchAffirm,			// 设备告警查询确认;
	Robot_MainLineShow,						// 主接线展示;
	Robot_PatrolResultBrowse,				// 巡检结果浏览;
    Robot_PatrolReportGenerate,				// 巡检报告生成;
	Robot_IntervalShow,						// 间隔展示;
	Robot_CompareAnalysis,					// 对比分析;
	Robot_GenerateExcel,					// 生成报表;
    Robot_AlarmThresholdSet,                // 告警阈值设置;
	Robot_AlarmMessageSubscription,			// 告警消息订阅设置;
    Robot_PermissionManager,                // 权限管理;
	Robot_StandardPointPosMaintain,			// 标准点位库维护;
    Robot_FixedAreaSet,                     // 检修区域设置;
    Robot_PatrolMapMaintain,                // 巡检地图维护;
    Robot_CompareDetection                  // 异物检测;
};

class BottomWidget;
class SystemGuideWidget;
class WheelRobotLoginWindow;
class DLWheelRobotManager;
class DLWheelTaskManage;
class DLWheelTaskShow;
class DLWheelSoftSet;
class DLWheelRobotSet;
class DLWheelRobotAlarmSearch;
class DLWheelRobotStateShow;
class DLWheelIdentifyAbnormalPointSearch;
class DLWheelPatrolPointPosSet;
class DLWheelDeviceAlarmSearchAffirm;
class DLWheelMainLineShow;
class DLWheelPatrolResultBrowse;
class DLWheelIntervalShow;
class DLWheelCompareAnalysis;
class DLWheelGenerateExcel;
class DLWheelAlarmMessageSubscription;
class DLWheelStandardPointPosMaintain;
class CameraObject;
class DLWheelAlarmThresholdSet;
class DLWheelPermissionManager;
class InfraredTemperature;
class DLWheelPatrolReportGenerate;
class DLFixedAreaWidget;
class DLWheelCollectMapWidget;
class DLMessageBox;
class ClearOutDateFileObject;
class ConfidenceValueShowWidget;
class DLWheelCompareDetection;
class BatteryWgt;
class DataTransfer;

/**********轮式后台主窗口**********/

class DLWheelMainWindow : public QWidget
{
	Q_OBJECT

public:
	DLWheelMainWindow(QWidget *parent = NULL);
	~DLWheelMainWindow();

protected:
	virtual bool eventFilter(QObject *watched, QEvent *event);

private:
	// 初始化控件;
	void initWidget();
	void initTitleWidget();
    
    void initStackedWidget();
    void initBlockWidget();

	// 初始化绑定所有Core的信号;
	void initCoreSignals();

    // 中转所有Core发送过来的信号;
    void initTransferFunction();

    // 初始化视频对象;
    void initCameraObject();

    // 图像和视频保存期限设置;
    void initImageAndVideoSaveSet();

    // 初始化需要清理的文件;
    void initClearOutDateFile();

	// 显示实时监控不同菜单页面;
	void showRealTimeMonitorWidget(SystemGuideMenuItemType itemType);
	// 显示任务管理不同菜单页面;
	void showTaskManageWidget(SystemGuideMenuItemType itemType);

	// 登录成功;
	void loginSuccess();

    // 关闭事件;
    void closeEvent(QCloseEvent *event);

    // 清除过期的图片、视频文件;
    void clearOutDateImageAndVideoFiles();

    // 权限设置;
    void userAuthoritySet();

signals:
	// 通知任务显示页面删除结果;
	void signalUpdateDeleteResult(QString strUuid, bool isSuccess, QString strMsg);
    // 通知任务管理保存按钮点击返回;
    void signalTaskManageSaveRsp(int typeId, bool isSuccess, QString strMsg);

    // 通知任务管理保存按钮(对任务编制进行修改)点击返回;
    void signalTaskManageUpdateRsp(int typeId, bool isSuccess, QString strMsg);

    // core通知当前任务开始回调;
    void signalTaskCallBack(WheelRobotCurrentTaskInfoShow);
    // core通知当前点状态开始回调;
    void signalPointStatusCallBack(WheelRobotTaskCurrentPointStatus);
    // core通知巡检结果回调;
    void signalWheelRobotInspectResultCallback(WheelInspectResultStruct inspectResultData);
    
    // core通知实时状态刷新;
    void signalUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);

    // core通知非实时状态刷新;
    void signalUpdateRobotNoneRealtimeStatus(WheelRobotNoneRealtimeStatus noneRealTimeStatus);

    // 巡检结果审核人提交回调;
    void signalPatrolResultBrowseCheckPeopleCallBack(bool isSuccess, QString strTaskUuid, QString strMsg);
    
    // 删除任务编制返回;
    void signalDeleteTaskCallBack(int, bool, QString);

    // core通知添加系统告警信息;
    void signalSystemWarningCallback(QString strMsg);

    // core通知机器人硬件异常代码;
    void signalHardwareWarningCallback(QVector<int>);

    // core通知切换机器人结果;
    void signalConnectToNewRobotResult(bool, WheelRobotCoreRobotConfig);

	//更新树节点的告警level
	void signalUpdateTreeItemLevel(QString strUUid, DeviceAlarmLevel iLevel);

	//一键审核
	void signalKeyAduit(bool, QString);

	//下载图片的信号
	void DownloadImageSignal(QString strPath, QString strFileName, int type);

private slots:
	// 登录点击;
	void onLogin(QString userName, QString password);
	// 系统菜单点击;
	void onSystemMenuItemClicked(SystemGuideMenuItemType itemType);
	
	void CloseSystemMenuItemSlot(SystemGuideMenuItemType);							//关闭菜单
	// 任务展示表格中修改按钮点击;
	void onTaskShowTableModifyButtonClicked(WheelTaskAdminType task_edit_type_id, QString task_edit_uuid);
    // 更新实时信息;
    void onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);

    // 异物检测;
    void onCompareDetection();

	
	void OpenSmapFileSlot(QString strSmapFile);											//打开smap文件


	void UpdateRobotRealtimeStatusSlot(WheelRobotRealtimeStatus realTimeStatus);

	void UpdateRobotStatusTimerSlot();

	void DownloadImageSlot(QString strPath, QString strFileName, int type);

	void slot_on_transfer_finished(int command_type, int execCode, QString fileName);
    void slot_on_transfer_finished_infrared(QString path, QString name);

    void StutterStopBtnSlot();

	void slot_m_sys_info_btn_clicked();

    void keyPressEvent(QKeyEvent *event);
private:
	QWidget* m_pTitleWgt;												//窗口title整体（内包含导航栏）
	QToolButton* m_pButtonHomePage;		
	QToolButton* m_pButtonHelp;
	QToolButton* m_pButtonMin;
	QToolButton* m_pButtonQuit;
	QLabel* m_labelWelcomeText;											//欢迎文本

	QWidget *m_pMouseWgt;												//窗口头拖拽
	bool m_bIsPressed;													//判断处理鼠标状态
	QPoint m_ptStartPos;

	SystemGuideWidget* m_pSystemGuideWgt;
	QLabel* m_pLblGuideText;
	QWidget* m_pGuideBackWgt;

	QPushButton* m_pButtonTitleLogo;
	QLabel* m_labelTitleSplite;
	QLabel* m_labelTitleText;
	QLabel* m_labelProvinceText;

	QWidget* m_centerWidget;

	BottomWidget* m_pBottomWgt;

	WheelRobotLoginWindow* m_loginWindow;

	QStackedWidget* m_pStackedWidget;

	// 实时监控;
	DLWheelRobotManager* m_pRobotRealTimeManager;
	// 任务管理;
	DLWheelTaskManage* m_robotTaskManageAll_Patrol;
	DLWheelTaskManage* m_robotTaskManageRoutine_Patrol;
	DLWheelTaskManage* m_robotTaskManageInfraredCalculateTmp;
	DLWheelTaskManage* m_robotTaskManageOilTableRecord;
	DLWheelTaskManage* m_robotTaskManageArresterTableRead;
	DLWheelTaskManage* m_robotTaskManageSF6PressureRecord;
	DLWheelTaskManage* m_robotTaskManageHydraumaticTableRecord;
	DLWheelTaskManage* m_robotTaskManagePosStateRecognition;
	DLWheelTaskManage* m_robotTaskManageBadWeatherPatrol;
	DLWheelTaskManage* m_robotTaskManageDefectTrack;
	DLWheelTaskManage* m_robotTaskManageDistanceStateAffirm;
	DLWheelTaskManage* m_robotTaskManageDistanceAbnormalAlarmAffirm;
	DLWheelTaskManage* m_robotTaskManageSecurityLinkage;
	DLWheelTaskManage* m_robotTaskManageAssistAccidentDeal;
	DLWheelTaskManage* m_robotTaskManageCustomTask;

	QList<DLWheelTaskManage*> m_rootTaskManagerWidgetList;
	// 任务展示;
	DLWheelTaskShow* m_robotTaskShow;
	// 软件设置;
	DLWheelSoftSet* m_robotSoftSet;
	// 机器人设置;
	DLWheelRobotSet* m_robotSet;
	LibRobotControlWidget* m_robotControl;
	// 机器人告警查询;
	DLWheelRobotAlarmSearch* m_robotAlarmSearch;
	// 机器人状态显示;
	DLWheelRobotStateShow* m_pRobotStateShow;
	// 识别异常点位查询;
	DLWheelIdentifyAbnormalPointSearch* m_identifyAbnormalPointSearch;
	// 巡检点位设置;
	DLWheelPatrolPointPosSet* m_patrolPointPosSet;
	// 设备告警查询确认;
	DLWheelDeviceAlarmSearchAffirm* m_deviceAlarmSearchAffirm;
	// 主接线展示;
	DLWheelMainLineShow* m_mainLineShow;
	// 巡检结果浏览;
	DLWheelPatrolResultBrowse* m_patrolResultBrowse;
    // 巡检报告生成;
    DLWheelPatrolReportGenerate* m_patrolReportGenerate;
	// 间隔展示;
	DLWheelIntervalShow* m_intervalShow;
	// 对比分析;
	DLWheelCompareAnalysis* m_compareAnalysis;
	// 生成报表;
	DLWheelGenerateExcel* m_generateExcel;
    // 告警阈值设置;
    DLWheelAlarmThresholdSet* m_alarmThresholdSet;
	// 告警消息订阅设置;
	DLWheelAlarmMessageSubscription* m_alarmMessageSubscription;
    // 权限管理;
    DLWheelPermissionManager* m_permissionManager;
	// 标准点位库维护;
	DLWheelStandardPointPosMaintain* m_standardPointPosMaintain;

    // 检修区域设置;
    DLFixedAreaWidget* m_fixedAreaWidget;

    // 巡检地图维护;
    DLWheelCollectMapWidget* m_pPatrolMapMaintainWidget;

    // 异物检测;
    DLWheelCompareDetection* m_compareDetection;

    // 可见光视频显示/操作对象;
    CameraObject* m_visibleLightCameraObject;
    // 红外视频显示/操作对象;
    CameraObject* m_infraredCameraObject;

    // 红外视频操作对象;
//    InfraredTemperature* m_infraredTemperature;

    // 当前登录角色;
    WheelUserType m_currentLoginRole;

    // 图片、视频保存的最长天数;
    int m_imageSaveMaxDays;
    int m_videoSaveMaxDays;

    QTimer m_imageVideoSaveDealLineSetTimer;

    // 操作阻塞窗口;
    DLMessageBox* m_blockMessageBox;

    // 定时清理对象;
    ClearOutDateFileObject* m_clearOutDateFileObject;

    // 置信度;
	QLabel *m_pConfidenValueLbl;
    ConfidenceValueShowWidget* m_pConfidenceValueWgt;

	QLabel *m_pBatteryLevelLbl;
	//BatteryWgt *m_pBatteryWgt;
	ConfidenceValueShowWidget* m_pBatteryLevelWgt;					//电池电量
	//QLabel * m_electricQuantityLabel;

	QPushButton *m_pStutterStopBtn;		//急停按钮

	QPushButton *m_sys_info_btn;		//系统信息按钮

	QPixmap m_confidenceImages[2];

	QTimer		*m_pUpdateRobotStatusTimer;			//更新置信度和电池电量的定时器
	qreal m_rBatteryValue;		//电池电量
	qreal m_rConfidenValue;		//置信度的值
	bool m_bIsChargeState;		//是否为充电状态
	QLabel *m_pChargeStateLbl;

	DataTransfer *m_pThreadDataTransfer;						//线程用来上传smap到机器人

};
