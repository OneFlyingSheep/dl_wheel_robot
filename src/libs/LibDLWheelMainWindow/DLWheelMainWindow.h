#pragma once

#include <QWidget>
#include <QToolButton>
#include <QPushButton>
#include <QLabel>
#include <QStackedWidget>
#include <QTimer>
#include "common/DLWheelRobotGlobalDef.hpp"
#include "LibRobotControlWidget/libRobotControlWidget.h"
// ��ͬҳ��ö��;
enum StackedWidgetType
{
	Robot_RealTimeManager = 0,				// ʵʱ���;
											// �������;
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

	Robot_TaskShow,							// ����չʾ;
	Robot_SoftSet,							// �������;
	Robot_Set,								// ����������;
	Robot_AlarmSearch,						// �����˸澯��ѯ;
	Robot_StateShow,						// ������״̬��ʾ;
	Robot_IdentifyAbnormalPointSearch,		// ʶ���쳣��λ��ѯ;
	Robot_PatrolPointPosSet,				// Ѳ���λ����;
	Robot_DeviceAlarmSearchAffirm,			// �豸�澯��ѯȷ��;
	Robot_MainLineShow,						// ������չʾ;
	Robot_PatrolResultBrowse,				// Ѳ�������;
    Robot_PatrolReportGenerate,				// Ѳ�챨������;
	Robot_IntervalShow,						// ���չʾ;
	Robot_CompareAnalysis,					// �Աȷ���;
	Robot_GenerateExcel,					// ���ɱ���;
    Robot_AlarmThresholdSet,                // �澯��ֵ����;
	Robot_AlarmMessageSubscription,			// �澯��Ϣ��������;
    Robot_PermissionManager,                // Ȩ�޹���;
	Robot_StandardPointPosMaintain,			// ��׼��λ��ά��;
    Robot_FixedAreaSet,                     // ������������;
    Robot_PatrolMapMaintain,                // Ѳ���ͼά��;
    Robot_CompareDetection                  // ������;
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

/**********��ʽ��̨������**********/

class DLWheelMainWindow : public QWidget
{
	Q_OBJECT

public:
	DLWheelMainWindow(QWidget *parent = NULL);
	~DLWheelMainWindow();

protected:
	virtual bool eventFilter(QObject *watched, QEvent *event);

private:
	// ��ʼ���ؼ�;
	void initWidget();
	void initTitleWidget();
    
    void initStackedWidget();
    void initBlockWidget();

	// ��ʼ��������Core���ź�;
	void initCoreSignals();

    // ��ת����Core���͹������ź�;
    void initTransferFunction();

    // ��ʼ����Ƶ����;
    void initCameraObject();

    // ͼ�����Ƶ������������;
    void initImageAndVideoSaveSet();

    // ��ʼ����Ҫ������ļ�;
    void initClearOutDateFile();

	// ��ʾʵʱ��ز�ͬ�˵�ҳ��;
	void showRealTimeMonitorWidget(SystemGuideMenuItemType itemType);
	// ��ʾ�������ͬ�˵�ҳ��;
	void showTaskManageWidget(SystemGuideMenuItemType itemType);

	// ��¼�ɹ�;
	void loginSuccess();

    // �ر��¼�;
    void closeEvent(QCloseEvent *event);

    // ������ڵ�ͼƬ����Ƶ�ļ�;
    void clearOutDateImageAndVideoFiles();

    // Ȩ������;
    void userAuthoritySet();

signals:
	// ֪ͨ������ʾҳ��ɾ�����;
	void signalUpdateDeleteResult(QString strUuid, bool isSuccess, QString strMsg);
    // ֪ͨ��������水ť�������;
    void signalTaskManageSaveRsp(int typeId, bool isSuccess, QString strMsg);

    // ֪ͨ��������水ť(��������ƽ����޸�)�������;
    void signalTaskManageUpdateRsp(int typeId, bool isSuccess, QString strMsg);

    // core֪ͨ��ǰ����ʼ�ص�;
    void signalTaskCallBack(WheelRobotCurrentTaskInfoShow);
    // core֪ͨ��ǰ��״̬��ʼ�ص�;
    void signalPointStatusCallBack(WheelRobotTaskCurrentPointStatus);
    // core֪ͨѲ�����ص�;
    void signalWheelRobotInspectResultCallback(WheelInspectResultStruct inspectResultData);
    
    // core֪ͨʵʱ״̬ˢ��;
    void signalUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);

    // core֪ͨ��ʵʱ״̬ˢ��;
    void signalUpdateRobotNoneRealtimeStatus(WheelRobotNoneRealtimeStatus noneRealTimeStatus);

    // Ѳ����������ύ�ص�;
    void signalPatrolResultBrowseCheckPeopleCallBack(bool isSuccess, QString strTaskUuid, QString strMsg);
    
    // ɾ��������Ʒ���;
    void signalDeleteTaskCallBack(int, bool, QString);

    // core֪ͨ���ϵͳ�澯��Ϣ;
    void signalSystemWarningCallback(QString strMsg);

    // core֪ͨ������Ӳ���쳣����;
    void signalHardwareWarningCallback(QVector<int>);

    // core֪ͨ�л������˽��;
    void signalConnectToNewRobotResult(bool, WheelRobotCoreRobotConfig);

	//�������ڵ�ĸ澯level
	void signalUpdateTreeItemLevel(QString strUUid, DeviceAlarmLevel iLevel);

	//һ�����
	void signalKeyAduit(bool, QString);

	//����ͼƬ���ź�
	void DownloadImageSignal(QString strPath, QString strFileName, int type);

private slots:
	// ��¼���;
	void onLogin(QString userName, QString password);
	// ϵͳ�˵����;
	void onSystemMenuItemClicked(SystemGuideMenuItemType itemType);
	
	void CloseSystemMenuItemSlot(SystemGuideMenuItemType);							//�رղ˵�
	// ����չʾ������޸İ�ť���;
	void onTaskShowTableModifyButtonClicked(WheelTaskAdminType task_edit_type_id, QString task_edit_uuid);
    // ����ʵʱ��Ϣ;
    void onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);

    // ������;
    void onCompareDetection();

	
	void OpenSmapFileSlot(QString strSmapFile);											//��smap�ļ�


	void UpdateRobotRealtimeStatusSlot(WheelRobotRealtimeStatus realTimeStatus);

	void UpdateRobotStatusTimerSlot();

	void DownloadImageSlot(QString strPath, QString strFileName, int type);

	void slot_on_transfer_finished(int command_type, int execCode, QString fileName);
    void slot_on_transfer_finished_infrared(QString path, QString name);

    void StutterStopBtnSlot();

	void slot_m_sys_info_btn_clicked();

    void keyPressEvent(QKeyEvent *event);
private:
	QWidget* m_pTitleWgt;												//����title���壨�ڰ�����������
	QToolButton* m_pButtonHomePage;		
	QToolButton* m_pButtonHelp;
	QToolButton* m_pButtonMin;
	QToolButton* m_pButtonQuit;
	QLabel* m_labelWelcomeText;											//��ӭ�ı�

	QWidget *m_pMouseWgt;												//����ͷ��ק
	bool m_bIsPressed;													//�жϴ������״̬
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

	// ʵʱ���;
	DLWheelRobotManager* m_pRobotRealTimeManager;
	// �������;
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
	// ����չʾ;
	DLWheelTaskShow* m_robotTaskShow;
	// �������;
	DLWheelSoftSet* m_robotSoftSet;
	// ����������;
	DLWheelRobotSet* m_robotSet;
	LibRobotControlWidget* m_robotControl;
	// �����˸澯��ѯ;
	DLWheelRobotAlarmSearch* m_robotAlarmSearch;
	// ������״̬��ʾ;
	DLWheelRobotStateShow* m_pRobotStateShow;
	// ʶ���쳣��λ��ѯ;
	DLWheelIdentifyAbnormalPointSearch* m_identifyAbnormalPointSearch;
	// Ѳ���λ����;
	DLWheelPatrolPointPosSet* m_patrolPointPosSet;
	// �豸�澯��ѯȷ��;
	DLWheelDeviceAlarmSearchAffirm* m_deviceAlarmSearchAffirm;
	// ������չʾ;
	DLWheelMainLineShow* m_mainLineShow;
	// Ѳ�������;
	DLWheelPatrolResultBrowse* m_patrolResultBrowse;
    // Ѳ�챨������;
    DLWheelPatrolReportGenerate* m_patrolReportGenerate;
	// ���չʾ;
	DLWheelIntervalShow* m_intervalShow;
	// �Աȷ���;
	DLWheelCompareAnalysis* m_compareAnalysis;
	// ���ɱ���;
	DLWheelGenerateExcel* m_generateExcel;
    // �澯��ֵ����;
    DLWheelAlarmThresholdSet* m_alarmThresholdSet;
	// �澯��Ϣ��������;
	DLWheelAlarmMessageSubscription* m_alarmMessageSubscription;
    // Ȩ�޹���;
    DLWheelPermissionManager* m_permissionManager;
	// ��׼��λ��ά��;
	DLWheelStandardPointPosMaintain* m_standardPointPosMaintain;

    // ������������;
    DLFixedAreaWidget* m_fixedAreaWidget;

    // Ѳ���ͼά��;
    DLWheelCollectMapWidget* m_pPatrolMapMaintainWidget;

    // ������;
    DLWheelCompareDetection* m_compareDetection;

    // �ɼ�����Ƶ��ʾ/��������;
    CameraObject* m_visibleLightCameraObject;
    // ������Ƶ��ʾ/��������;
    CameraObject* m_infraredCameraObject;

    // ������Ƶ��������;
//    InfraredTemperature* m_infraredTemperature;

    // ��ǰ��¼��ɫ;
    WheelUserType m_currentLoginRole;

    // ͼƬ����Ƶ����������;
    int m_imageSaveMaxDays;
    int m_videoSaveMaxDays;

    QTimer m_imageVideoSaveDealLineSetTimer;

    // ������������;
    DLMessageBox* m_blockMessageBox;

    // ��ʱ�������;
    ClearOutDateFileObject* m_clearOutDateFileObject;

    // ���Ŷ�;
	QLabel *m_pConfidenValueLbl;
    ConfidenceValueShowWidget* m_pConfidenceValueWgt;

	QLabel *m_pBatteryLevelLbl;
	//BatteryWgt *m_pBatteryWgt;
	ConfidenceValueShowWidget* m_pBatteryLevelWgt;					//��ص���
	//QLabel * m_electricQuantityLabel;

	QPushButton *m_pStutterStopBtn;		//��ͣ��ť

	QPushButton *m_sys_info_btn;		//ϵͳ��Ϣ��ť

	QPixmap m_confidenceImages[2];

	QTimer		*m_pUpdateRobotStatusTimer;			//�������ŶȺ͵�ص����Ķ�ʱ��
	qreal m_rBatteryValue;		//��ص���
	qreal m_rConfidenValue;		//���Ŷȵ�ֵ
	bool m_bIsChargeState;		//�Ƿ�Ϊ���״̬
	QLabel *m_pChargeStateLbl;

	DataTransfer *m_pThreadDataTransfer;						//�߳������ϴ�smap��������

};
