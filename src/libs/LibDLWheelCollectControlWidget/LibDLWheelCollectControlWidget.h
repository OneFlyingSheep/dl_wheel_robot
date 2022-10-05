#ifndef __DL_WHEEL_ROBOT_COLLECT_CONTROL_H__
#define __DL_WHEEL_ROBOT_COLLECT_CONTROL_H__

#include <QWidget>
#include <QTreeWidget>
#include <QTimer>
#include <QThread>
#include <QLabel>
#include <QPushButton>
#include "LibDLHangRailConfigData/DLHangRailDeviceCfgData.h"
#include <QListWidget>
#include <QRadioButton>
#include <QStackedWidget>
#include "LibDLWheelRobotConfigData/DLWheelRobotDeviceConfig.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include <QThread>
#include "LibHCNetCamera/HikCameraPointData.h"
//#include "CollectStandardizationWidget.h"

class BaseWidget;
class InputWidget;
class CustomButton;
class VideoBackWidget;
class CameraObject;
class InfraredTemperature;
class AddEquipmentTreeWgt;
class DLWheelPointTreeData;
class DLCustomScene;
enum TreeItemWidgetType;
enum RootNodeType;
struct TreeItemInfo;
class DataTransfer;
class DLMapListWidget;
class DLStatusBar;

/******���̰��������߳�*******/

class KeyboardOperateThread : public QThread
{

public:
	KeyboardOperateThread()
		: m_isOperationStop(true)
		, m_isRelease(false)
		, m_isThreadStop(false)
		, m_moveSpeedScale(1)
		, m_bIsPressedState(true)
	{

	}

	~KeyboardOperateThread()
	{
		this->wait();
		this->quit();
	}

	void stop()
	{
		m_isThreadStop = true;
	}

	// ���õ�ǰ���̲�������;
	void setCurrentOperationType(int operationType, bool isRelease)
	{
		m_operationType = operationType;
		if (isRelease)
		{
			m_isRelease = true;
		}
		else
		{
			m_isRelease = false;
		}
		m_isOperationStop = false;
	}

	// ���õ�ǰ�Ƿ����;
	void setRobotSpeedUp(bool isSpeedUp)
	{
		if (isSpeedUp)
		{
			m_moveSpeedScale = 2;
		}
		else
		{
			m_moveSpeedScale = 1;
		}
	}

	void setRealtimeStatus(WheelRobotPtzStatus pantitl)
	{
		m_pantitl = pantitl;
	}

	void clickMovePanTilt(int check)
	{
		switch (check)
		{
		case 1:
		{
// 			int iPanValue = 0;
// 			iPanValue = m_pantitl.pan - 23;
// 			if (iPanValue < 0)
// 			{
// 				iPanValue = 36000 + iPanValue;
// 			}
// 			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(iPanValue, m_pantitl.tilt);
			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_relative_req(5, 0);
			break;
		}
			
		case 2:
		{
// 			int iTiltValue = 0;
// 			if (m_pantitl.tilt > 0 && m_pantitl.tilt < 3066)
// 			{
// 				iTiltValue = m_pantitl.tilt + 23;
// 				if (iTiltValue > 3066)
// 				{
// 					iTiltValue = 3066;
// 				}
// 			}
// 			if (m_pantitl.tilt > 27090 && m_pantitl.tilt < 36000)
// 			{
// 				iTiltValue = m_pantitl.tilt + 23;
// 				if (iTiltValue > 36000)
// 				{
// 					iTiltValue = iTiltValue - 36000;
// 				}
// 			}
// 			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(m_pantitl.pan, iTiltValue);
			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_relative_req(0, -5);
			break;
		}
		case 3:
		{
// 			int iPanValue = 0;
// 			iPanValue = m_pantitl.pan + 48;
// 			if (iPanValue > 36000)
// 			{
// 				iPanValue = iPanValue - 36000;
// 			}
// 			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(iPanValue, m_pantitl.tilt);
			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_relative_req(-5, 0);
			break;
		}
		case 4:
		{
// 			int iTiltValue = 0;
// 			if (m_pantitl.tilt > 0 && m_pantitl.tilt < 3066)
// 			{
// 				iTiltValue = m_pantitl.tilt - 45;
// 				if (iTiltValue < 0)
// 				{
// 					iTiltValue = iTiltValue + 36000;
// 				}
// 			}
// 			if (m_pantitl.tilt > 27090 && m_pantitl.tilt < 36000)
// 			{
// 				iTiltValue = m_pantitl.tilt - 45;
// 				if (iTiltValue < 27090)
// 				{
// 					iTiltValue = 27090;
// 				}
// 			}
// 			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_abs_req(m_pantitl.pan, iTiltValue);
			WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_relative_req(0, 5);
			break;
		}
			
		default:
			break;
		}
	}

	void SetPressedState(bool bIsPressedState)
	{
		m_bIsPressedState = bIsPressedState;
	}
private:
	void run()
	{
		while(!m_isThreadStop)
		{
			if (m_isRelease && !m_isOperationStop)
			{
				m_isOperationStop = true;
				switch (m_operationType) {
					// ǰ��;
				case Qt::Key_Up:
				case Qt::Key_Down:
				case Qt::Key_Left:
				case Qt::Key_Right:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, 0);
					break;
				case Qt::Key_A:
					if (!m_bIsPressedState)
					{
						clickMovePanTilt(1);
						break;
					}
				case Qt::Key_S:
					if (!m_bIsPressedState)
					{
						clickMovePanTilt(2);
						break;
					}
				case Qt::Key_D:
					if (!m_bIsPressedState)
					{
						clickMovePanTilt(3);
						break;
					}
				case Qt::Key_W:
					if (!m_bIsPressedState)
					{
						clickMovePanTilt(4);
						break;
					}
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_STOP, 0, 0);
					break;
				default:
					break;
				}

			}
			else if (!m_isRelease && !m_isOperationStop)
			{
				switch (m_operationType) {
					// ǰ��;
				case Qt::Key_Up:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0.3 * m_moveSpeedScale, 0, 0);
					break;
					// ����;
				case Qt::Key_Down:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(-0.3 * m_moveSpeedScale, 0, 0);
					break;
					// ��;
				case Qt::Key_Left:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, 0.5 * m_moveSpeedScale);
					break;
					//
				case Qt::Key_Right:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0, 0, -0.5 * m_moveSpeedScale);
					break;
				case Qt::Key_A:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_LEFT, 0, 0.6 * m_moveSpeedScale);
					break;
				case Qt::Key_S:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_DOWN, 0, 0.6 * m_moveSpeedScale);
					break;
				case Qt::Key_D:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_RIGHT, 0, 0.6 * m_moveSpeedScale);
					break;
				case Qt::Key_W:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_ptz_motion_req(WheelRobotPtzMoveType::WHEEL_PTZ_MOVE_UP, 0, 0.6 * m_moveSpeedScale);
					break;
				default:
					break;
				}

			}
			QThread::msleep(100);
		}
	}
private:
	int m_operationType;
	bool m_isOperationStop;
	bool m_isRelease;
	bool m_isThreadStop;
	// ��ǰ�ٶȱ���;
	int m_moveSpeedScale;
	bool m_bIsPressedState;
	WheelRobotPtzStatus m_pantitl;
};


class DLOverfittingDlg;				//��ϴ���


/**************�ɼ�����ҳ��*************/
class DLHangWheelCollectControlWidget : public QWidget
{
	Q_OBJECT
	enum COLLECT_BTN_TYPE
	{
		OPEN_SMAP_TYPE,					//�򿪵����Ϣ�ļ�
		SAVE_SMAP_TYPE,					//��������Ϣ�ļ�
		SAVEAS_SMAP_TYPE,				//�������Ϣ�ļ�
		DOWNLOAD_2D_MAP_TYPE,			//����2D��ͼ
		UPLOAD_SMAP_TYPE,				//�ϴ�Smap
		UPLOAD_MAPINFO_TYPE,			//�ϴ������Ϣ�ļ�
		SELECT_TYPE,					//ѡ��
		OVERFITTING_TYPE,				//���	
		ADD_PATH_TYPE,					//���·��
		ADD_ADVANCED_AREA_TYPE,			//�߼�����
		RELOCATION_TYPE,				//�ض�λ
		KEY_MODE_TYPE,					//����ģʽ
		TASK_MODE_TYPE,					//����ģʽ
		HAND_MODE_TYPE,					//�ֱ�ģʽ
		URGENCY_RELOCATION_TYPE			//������λģʽ
	};

public:
	DLHangWheelCollectControlWidget(QWidget *parent = NULL);
	~DLHangWheelCollectControlWidget();

	// ���ÿɼ�����Ƶ��������;
	void setCameraObject(CameraObject* cameraObject);

	// ���ú�����Ƶ��������;
    void setInfraredObject(void*, CameraObject* camerObject);

	// ˢ���������ؼ�;
	void onRefreshTreeWodget();

	// ��Core����ɾ�����ڵ�����;
	//void onDeleteTreeItemNodeReq(RootNodeType nodeType, QString deviceId, QString parentId);

	// �޸��豸��Ϣ;
	void onModifyDeviceInfo(QString strDeviceId, QString strPointId);

	// ���ؼ��Ҽ��˵�����ƶ����õ�;
	void onMoveToDevicePoint(QString strDeviceId);

	// Core֪ͨ�����豸���;
	void onAddDeviceResult(QString deviceId, bool isSuccess, QString msg);

	// Core֪ͨ�����豸���;
	void onUpdateDeviceResult(QString deviceId, bool isSuccess, QString msg);

	// Core֪ͨɾ���豸���;
	//void onDeleteDeviceResult(bool isSuccess, QString msg);

	// Core֪ͨ����ʵʱ����;
	void onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);

	// �����¼�;
	void setCurrentOperationType(int operationType, bool isRelease);

	// ���û����˿����Ƿ����;
	void setRobotSpeedUp(bool isSoeedUp);
	
    // �ļ���������;
	bool copyRecursively(const QString &srcFilePath, const QString &tgtFilePath);

	void onStartCollectZoomPicture();
	//��ȡ����ֵ
	void getPtzPanAndTilt(zoomInfo info_, int &_panValue, int &_tiltValue);
	void createTxt(QList<QStringList> _data);

    // protected:
// 	virtual void showEvent(QShowEvent *event);

private:
	void initWidget();
	// ��ʼ����ͼ;
	void initMapWidget();
	// ��ʼ����������Ϣ;
	void initRobotInfo();
	// ��ʼ�������˿��ư�ť;
	void initRobotControl();
	// ��ʼ������豸��һ��widget;
	void initAddDeviceFirstStageWidget();
	void initAddDeviceComboBox();
	// ��ʼ������豸�ɼ���Զ����Ϣwidget;
	void initAddDeviceVisibleFarStageWidget();
	// ��ʼ������豸�ɼ��������Ϣwidget;
	void initAddDeviceVisibleNearStageWidget();
	// ��ʼ������豸������Ϣwidget;
	void initAddDeviceInfraredStageWidget();
	// ��ʼ�������豸��Ϣ;
	void initAddDeviceWidget();
	// ��ʼ���豸�б�;
	void initDeviceList();
	// ��ʼ����Ƶ����;
	void initVideoWidget();
	// ��ʼ��������Ϣwidget;
	void initErrorMessageWidget();
	// ������̨�Ƕ�����ʱ��;
	void initUpdatePtzAngleDataTimer();
	// ��ʼ��������ť;
	void initOperationButton();

	// ���������������;
	void clearAllInputContent();

	// �����豸������豸��λ���Ƿ��X�ж�radioButton�Ƿ��û�;
	void updateAddDeviceRadioState();

	// ��ȡ��ǰʱ��;
	QString getCurrentTime();

	void UploadSMAPFile();							//�ϴ�smap��ͼ

	QPushButton *CreateToolBtn(QWidget *parentWgt, const QString strName, const QString &strObjectName, const QString &strIconPath);  //������ť��

	void InitStatus();

    void changeWidgetButtonStyle(WheelButtonStyleSheet type);
signals:
    // �򿪵�ͼ�ź�;
    //void signalOpenSmap(QString);
    //֪ͨ�ɼ�Ѳ���;
    void signalCollectPatrolPoint();
	// ��ʼץͼ;
	void signalStartAutoCapture();
	// ֹͣץͼ;
	void signalStopAutoCaptrue();

	// ֪ͨ���ؼ���ӽڵ�;
	void signalAddNewItemToTree(QString deviceId, QList<WheelRobortAlarmPathStruct>);

	// ֪ͨ���ؼ�����ˢ��;
	void signalTreeWidgetRefresh(QList<WheelPointTreeDataStruct>);

	// ���Ϳɼ��⵱ǰ����;
	void signalUpdateVisibleVideoParam(int zoomValue);

    // ���Ͳ�����Ϣ;
    void signalSendOperateMsg(QString strMsg);

	void OpenSmapFileSignal(QString strSmapFile);							//���ļ�


private slots:
	// ���»�������Ϣ;
	void onUpdateRobotInfo();

	// Բ�̰�ť����;
	void onRobotVideoControlCustomButtonPressed(int buttonId);
	void onRobotVideoControlCustomButtonReleased(int buttonId);

	// ��������ؿ���
	void onRobotBodyControlPressed(int buttonId);	///< ���������˿��ƽ�������������¼�
	void onRobotBodyControlReleased(int buttonId);	///< ���������˿��ƽ����������ͷ��¼�
	void onRobotPTZControlPressed(int buttonId);	///< ������������̨���ƽ�������������¼�
	void onRobotPTZControlReleased(int buttonId);	///< ������������̨���ƽ����������ͷ��¼�

	// �豸�б�˫���¼�;
	void onDeviceItemDoubleClicked(QTreeWidgetItem* item, int column);

	// �豸�б������¼�;
	void onDeviceItemClicked(QTreeWidgetItem* item, int column);

	// ͼƬ����;
	void onExportCameraCaptureImage();

	//�����豸���ڵ�
	void UpdateEquipmentTreeSlot(bool, QStringList, RootNodeType);

	//void CollectEquipmentSlot(QString strDeviceID);			//�Ѽ��豸

	void PressedRadioBtnSlot(bool bIsChecked);
	void ClickedRadioBtnSlot(bool bIsChecked);
	void BtnClickeSlot(int);  //��ťclicked�Ĳۺ���
	void slot_on_choose_map(QString download_path, QString filename, int file_type);
	void slot_on_transfer_finished(int command_type, int execCode, QString exeParam);
    void slot_on_transfer_finished_infrared(QString path, QString name);

	void ChangeTypeSlot( const QString &, int iSelectType);			//scene���ͷ����ı�Ĳۺ���
	void SMAPChangedSlot();								//��ť״̬�����ı�Ĳۺ���
	//void ReadFinishedSlot(bool bIsRunning);			//��дsmap�ļ������Ĳۺ���
	//void OverFittingBtnSlot();						//��ϰ�ť�Ĳۺ���

	void ScannMapBtnSlot();

    void DownloadImageSlot(QString strPath, QString strFileName, int type);

	void MoveCollectEquipmentPosSlot(bool bIsCenter);

    void SlotSetInfraredFocus(int focus)
    {
        m_infraredFocusValueCore = focus;
    }

    void slot_ret_finished_upload(int ret_code, QString err_msg);
private:
	DLCustomScene * m_pCollectMapScene;
	// ��ͼ;
	BaseWidget* m_mapWidget;

	// ��������Ϣ;
	QWidget * m_robotInfoWidget;
	// ��������ؿ���;
	//BaseWidget * m_robotControlWidget;

	// �ɼ������;
	BaseWidget* m_visibleLightControlWidget;
	// �������;
	BaseWidget* m_infraredControlWidget;
	
	// �豸�б�;
	BaseWidget* m_deviceListWidget;
	// �豸��;
	AddEquipmentTreeWgt* m_pDeviceTreeWidget;

	//��ͼ����
	BaseWidget* m_pMapToolWgt;
	QButtonGroup *m_pFileBtnGroup;				//��ʼɨ��/����ɨ��/����2d��ͼ/�ϴ�smap/����smap/��������Ϣ
	QButtonGroup *m_pOperatorBtnGroup;				//ѡ��/�Զ��ӵ�/�ֶ��ӵ�/���·��/�߼�����/�ض�λ
	QButtonGroup *m_pModelBtnGroup;				//����ģʽ/����ģʽ/�ֱ�ģʽ/������λģʽ

	//QLabel *m_pModelWgt;
	//bool m_bIsFirstIn;


	// �ɼ�����Ƶ��ʾ;
	//BaseWidget* m_samplingVisibleLightVideoBack;
	QWidget *m_samplingVisibleLightVideoBack;
	// ������Ƶ;
	QWidget* m_samplinginfraredVideoBack;
	// �ɼ����������;
	CustomButton* m_robotVideoControlCustomButton;
	// �ɼ�����ƵWidget;
	VideoBackWidget* m_visibleLightVideoBackWidget;
	// ������ƵWidget;
	VideoBackWidget* m_infraredVideoBackWidget;

	// ��������Ϣ;
	// �ɼ��ⱶ��;
	InputWidget* m_visibleLightRate;
	// �ɼ��⽹��;
	InputWidget* m_visibleLightFocus;
	// ���⽹��;
	InputWidget* m_infraredFocus;
	// ��̨ˮƽ��ת;
	InputWidget* m_PtzHRotate;
	// ��̨��ֱ��ת;
	InputWidget* m_PtzVRotate;

	// ����ʱ��;
	QTimer m_updateDataTimer;

	// ��Ҫ���LineEdit��List;
	QList<InputWidget*> m_clearLineEditList;

	// �ɼ��⡢������¼���;
	int m_updateCount;

	// �豸 ID-Name Map;
	QMap<QString, QString> m_voltageLevelMap;
	QMap<QString, QString> m_areaNameMap;
	QMap<QString, QString> m_intervalNameMap;
	QMap<QString, QString> m_deviceTypeMap;
	QMap<QString, QString> m_deviceChildTypeMap;
	QMap<QString, WheelDevicePointNameStruct> m_devicePointPosMap;
	// QString - ssid, QString - deviceName;
	QMap<QString, QString> m_deviceNameMapBySsid;
	// �������� ID-Name Map;
	QMap<int, QString> m_areasTypeMap;
	// �����豸 ID-Name Map;
	QMap<int, QString> m_virtualDeviceNameMap;

	// ������Ҫ�޸ĵ��豸��Ϣ;
	deviceConfigType m_needModifydevConfig;

	// ����豸��Ϣwidget;
	BaseWidget* m_addDeviceInfoWidget;
	QStackedWidget* m_addDeviceStackedWidget;

	// ����豸���ζԽ�����̨�������Ϣ;
	int m_focusNearValue;
	int m_focusFarValue;
	int m_zoomNearValue;
	int m_zoomFarValue;
	int m_ptzPanValue;
	int m_ptzTiltValue;
	int m_pointId;

	// ����豸��������豸����;
	int m_infraredFocusValue;
	int m_infraredPtzPanValue;
	int m_infraredPtzTileValue;

	QWidget* m_addDeviceFirstStageWidget;
	InputWidget* m_voltageLevel;
	InputWidget* m_areaName;
	InputWidget* m_intervalName;
	InputWidget* m_deviceType;
	InputWidget* m_deviceChildType;
	InputWidget* m_devicePointPos;

	// ��������豸����֮ǰ�Ĳ�����¼;
	int m_voltageLevelIndex;
	int m_areaNameIndex;
	int m_intervalNameIndex;
	int m_deviceTypeIndex;
	int m_deviceChildTypeIndex;
	int m_devicePointPosIndex;

	QRadioButton* m_radioButtonA;
	QRadioButton* m_radioButtonB;
	QRadioButton* m_radioButtonC;
	QPushButton* m_pButtonAddDeviceNextStage;

	// ���/�޸� �ɼ����豸��ͼ����;
	QWidget* m_addDeviceVisibleNearWidget;
	// �ɼ������-����;
	InputWidget* m_visibleLightScaleNearWidget;
	// �ɼ������-����;
	InputWidget* m_visibleLightFocusNearWidget;
	// ��̨ˮƽ������ת;
	InputWidget* m_ptzHRotateWidget;
	// ��̨��ֱ������ת;
	InputWidget* m_ptzVRotateWidget;
	QPushButton* m_pButtonAddDeviceVisibleNearNextStage;
	QPushButton* m_pButtonAddDeviceVisibleNearLastStage;

	// �ɼ����һ��;
	InputWidget* m_saveZoomShowWidget;
	InputWidget* m_saveFocusShowWidget;
	InputWidget* m_savePtzPanShowWidget;
	InputWidget* m_savePtzTiltShowWidget;
	// �ɼ���ڶ���;
	InputWidget* m_saveFarZoomShowWidget;
	InputWidget* m_saveFarFocusShowWidget;
	// ����;
	InputWidget* m_saveInfraredFocusShowWidget;
	InputWidget* m_saveInfraredPtzPanShowWidget;
	InputWidget* m_saveInfraredPtzTiltShowWidget;

	QWidget* m_addDeviceVisibleFarWidget;
	// �ɼ������-����;
	InputWidget* m_visibleLightScaleFarWidget;
	// �ɼ������-����;
	InputWidget* m_visibleLightFocusFarWidget;
	QPushButton* m_pButtonAddDeviceVisibleFarLastStage;
	QPushButton* m_pButtonAddDeviceVisibleFinish;

	QWidget* m_addDeviceInfraredWidget;
	// �������-����;
	InputWidget* m_infraredFocusWidget;	
	// ������̨ˮƽ������ת;
	InputWidget* m_ptzHRotateInfraredWidget;
	// ������̨��ֱ������ת;
	InputWidget* m_ptzVRotateInfraredWidget;
	QPushButton* m_pButtonAddDeviceInfraredLastStage;
	QPushButton* m_pButtonAddDeviceInfraredFinish;

	// ������Ϣ��ʾ;
	QWidget* m_errorMessageBackWidget;
	QLabel* m_errorMessageLabel;

	// ���̰��������߳�;
	KeyboardOperateThread* m_keyboardOperateThread;

	// �����ж���̨�����Ƿ�ʱ����;
	QTimer m_updatePtzAngleDataTimer;

	QString m_strModifyDeviceId;
	bool m_isModifyDevice;

	BaseWidget* m_errorMsgListBackWidget;
	QListWidget* m_errorMsgListWidget;
	
	// ���ؼ����ݲ�ѯ����;
	DLWheelPointTreeData* m_wheelPointTreeData;

	// ��ǰ�Ƿ���¹�ʵʱ����;
	bool m_isUpdateRealTimeData;

	// ��ǰ����豸��uuid;
	QString m_currentAddDeviceId;

	// ������ťwidget;
	QWidget* m_operationButtonBackWidget;

	CameraObject* m_cameraObject;

	bool m_bIsPressedState;
	QList<QStringList> sliZoomPtz;

	//CollectStandardizationWidget *m_collectStWidget;

	DLMapListWidget *m_pMapListWgt;		//���ص�ͼ
	DataTransfer *m_pThreadDataTransfer;

	bool m_bSMAPIsChanged;
	//QString m_strFileName;										//���浱ǰ���ļ���
	//bool m_bIsUploadSmap;										//��¼�Ƿ��ϴ�������

	DLStatusBar *m_pStatusBar;									//״̬��
	QString m_strOpenMapPath;

    int m_infraredFocusValueCore = -1;

    bool m_bIsVisibleFull = false;
    bool m_bIsShrink = false;

//    InputWidget* m_deviceNameLabel_1;
};

#endif