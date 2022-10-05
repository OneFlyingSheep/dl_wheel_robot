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

/******键盘按键操作线程*******/

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

	// 设置当前键盘操作类型;
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

	// 设置当前是否加速;
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
					// 前进;
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
					// 前进;
				case Qt::Key_Up:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(0.3 * m_moveSpeedScale, 0, 0);
					break;
					// 后退;
				case Qt::Key_Down:
					WHEEL_BACK_TO_CORE_SOCKET.robot_control_motion_req(-0.3 * m_moveSpeedScale, 0, 0);
					break;
					// 左;
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
	// 当前速度倍数;
	int m_moveSpeedScale;
	bool m_bIsPressedState;
	WheelRobotPtzStatus m_pantitl;
};


class DLOverfittingDlg;				//拟合窗口


/**************采集控制页面*************/
class DLHangWheelCollectControlWidget : public QWidget
{
	Q_OBJECT
	enum COLLECT_BTN_TYPE
	{
		OPEN_SMAP_TYPE,					//打开点边信息文件
		SAVE_SMAP_TYPE,					//保存点边信息文件
		SAVEAS_SMAP_TYPE,				//另存点边信息文件
		DOWNLOAD_2D_MAP_TYPE,			//下载2D地图
		UPLOAD_SMAP_TYPE,				//上传Smap
		UPLOAD_MAPINFO_TYPE,			//上传点边信息文件
		SELECT_TYPE,					//选择
		OVERFITTING_TYPE,				//拟合	
		ADD_PATH_TYPE,					//添加路径
		ADD_ADVANCED_AREA_TYPE,			//高级区域
		RELOCATION_TYPE,				//重定位
		KEY_MODE_TYPE,					//键盘模式
		TASK_MODE_TYPE,					//任务模式
		HAND_MODE_TYPE,					//手柄模式
		URGENCY_RELOCATION_TYPE			//紧急定位模式
	};

public:
	DLHangWheelCollectControlWidget(QWidget *parent = NULL);
	~DLHangWheelCollectControlWidget();

	// 设置可见光视频操作对象;
	void setCameraObject(CameraObject* cameraObject);

	// 设置红外视频操作对象;
    void setInfraredObject(void*, CameraObject* camerObject);

	// 刷新整个树控件;
	void onRefreshTreeWodget();

	// 向Core发出删除树节点命令;
	//void onDeleteTreeItemNodeReq(RootNodeType nodeType, QString deviceId, QString parentId);

	// 修改设备信息;
	void onModifyDeviceInfo(QString strDeviceId, QString strPointId);

	// 树控件右键菜单点击移动到该点;
	void onMoveToDevicePoint(QString strDeviceId);

	// Core通知插入设备结果;
	void onAddDeviceResult(QString deviceId, bool isSuccess, QString msg);

	// Core通知更新设备结果;
	void onUpdateDeviceResult(QString deviceId, bool isSuccess, QString msg);

	// Core通知删除设备结果;
	//void onDeleteDeviceResult(bool isSuccess, QString msg);

	// Core通知更新实时数据;
	void onUpdateRobotRealtimeStatus(WheelRobotRealtimeStatus realTimeStatus);

	// 键盘事件;
	void setCurrentOperationType(int operationType, bool isRelease);

	// 设置机器人控制是否加速;
	void setRobotSpeedUp(bool isSoeedUp);
	
    // 文件拷贝操作;
	bool copyRecursively(const QString &srcFilePath, const QString &tgtFilePath);

	void onStartCollectZoomPicture();
	//获取处理值
	void getPtzPanAndTilt(zoomInfo info_, int &_panValue, int &_tiltValue);
	void createTxt(QList<QStringList> _data);

    // protected:
// 	virtual void showEvent(QShowEvent *event);

private:
	void initWidget();
	// 初始化地图;
	void initMapWidget();
	// 初始化机器人信息;
	void initRobotInfo();
	// 初始化机器人控制按钮;
	void initRobotControl();
	// 初始化添加设备第一步widget;
	void initAddDeviceFirstStageWidget();
	void initAddDeviceComboBox();
	// 初始化添加设备可见光远处信息widget;
	void initAddDeviceVisibleFarStageWidget();
	// 初始化添加设备可见光近处信息widget;
	void initAddDeviceVisibleNearStageWidget();
	// 初始化添加设备红外信息widget;
	void initAddDeviceInfraredStageWidget();
	// 初始化新增设备信息;
	void initAddDeviceWidget();
	// 初始化设备列表;
	void initDeviceList();
	// 初始化视频窗口;
	void initVideoWidget();
	// 初始化错误信息widget;
	void initErrorMessageWidget();
	// 更新云台角度数据时钟;
	void initUpdatePtzAngleDataTimer();
	// 初始化操作按钮;
	void initOperationButton();

	// 清楚所有输入内容;
	void clearAllInputContent();

	// 根据设备子类和设备点位中是否带X判断radioButton是否置灰;
	void updateAddDeviceRadioState();

	// 获取当前时间;
	QString getCurrentTime();

	void UploadSMAPFile();							//上传smap地图

	QPushButton *CreateToolBtn(QWidget *parentWgt, const QString strName, const QString &strObjectName, const QString &strIconPath);  //创建按钮的

	void InitStatus();

    void changeWidgetButtonStyle(WheelButtonStyleSheet type);
signals:
    // 打开地图信号;
    //void signalOpenSmap(QString);
    //通知采集巡检点;
    void signalCollectPatrolPoint();
	// 开始抓图;
	void signalStartAutoCapture();
	// 停止抓图;
	void signalStopAutoCaptrue();

	// 通知树控件添加节点;
	void signalAddNewItemToTree(QString deviceId, QList<WheelRobortAlarmPathStruct>);

	// 通知树控件重新刷新;
	void signalTreeWidgetRefresh(QList<WheelPointTreeDataStruct>);

	// 发送可见光当前参数;
	void signalUpdateVisibleVideoParam(int zoomValue);

    // 发送操作信息;
    void signalSendOperateMsg(QString strMsg);

	void OpenSmapFileSignal(QString strSmapFile);							//打开文件


private slots:
	// 更新机器人信息;
	void onUpdateRobotInfo();

	// 圆盘按钮操作;
	void onRobotVideoControlCustomButtonPressed(int buttonId);
	void onRobotVideoControlCustomButtonReleased(int buttonId);

	// 机器人相关控制
	void onRobotBodyControlPressed(int buttonId);	///< 消防机器人控制界面鼠标左键点击事件
	void onRobotBodyControlReleased(int buttonId);	///< 消防机器人控制界面鼠标左键释放事件
	void onRobotPTZControlPressed(int buttonId);	///< 消防机器人云台控制界面鼠标左键点击事件
	void onRobotPTZControlReleased(int buttonId);	///< 消防机器人云台控制界面鼠标左键释放事件

	// 设备列表双击事件;
	void onDeviceItemDoubleClicked(QTreeWidgetItem* item, int column);

	// 设备列表单击击事件;
	void onDeviceItemClicked(QTreeWidgetItem* item, int column);

	// 图片导出;
	void onExportCameraCaptureImage();

	//更新设备树节点
	void UpdateEquipmentTreeSlot(bool, QStringList, RootNodeType);

	//void CollectEquipmentSlot(QString strDeviceID);			//搜集设备

	void PressedRadioBtnSlot(bool bIsChecked);
	void ClickedRadioBtnSlot(bool bIsChecked);
	void BtnClickeSlot(int);  //按钮clicked的槽函数
	void slot_on_choose_map(QString download_path, QString filename, int file_type);
	void slot_on_transfer_finished(int command_type, int execCode, QString exeParam);
    void slot_on_transfer_finished_infrared(QString path, QString name);

	void ChangeTypeSlot( const QString &, int iSelectType);			//scene类型发生改变的槽函数
	void SMAPChangedSlot();								//按钮状态发生改变的槽函数
	//void ReadFinishedSlot(bool bIsRunning);			//读写smap文件结束的槽函数
	//void OverFittingBtnSlot();						//拟合按钮的槽函数

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
	// 地图;
	BaseWidget* m_mapWidget;

	// 机器人信息;
	QWidget * m_robotInfoWidget;
	// 机器人相关控制;
	//BaseWidget * m_robotControlWidget;

	// 可见光控制;
	BaseWidget* m_visibleLightControlWidget;
	// 红外控制;
	BaseWidget* m_infraredControlWidget;
	
	// 设备列表;
	BaseWidget* m_deviceListWidget;
	// 设备树;
	AddEquipmentTreeWgt* m_pDeviceTreeWidget;

	//地图工具
	BaseWidget* m_pMapToolWgt;
	QButtonGroup *m_pFileBtnGroup;				//开始扫描/结束扫描/下载2d地图/上传smap/下载smap/保存点边信息
	QButtonGroup *m_pOperatorBtnGroup;				//选择/自动加点/手动加点/添加路径/高级区域/重定位
	QButtonGroup *m_pModelBtnGroup;				//任务模式/键盘模式/手柄模式/紧急定位模式

	//QLabel *m_pModelWgt;
	//bool m_bIsFirstIn;


	// 可见光视频显示;
	//BaseWidget* m_samplingVisibleLightVideoBack;
	QWidget *m_samplingVisibleLightVideoBack;
	// 红外视频;
	QWidget* m_samplinginfraredVideoBack;
	// 可见光相机控制;
	CustomButton* m_robotVideoControlCustomButton;
	// 可见光视频Widget;
	VideoBackWidget* m_visibleLightVideoBackWidget;
	// 红外视频Widget;
	VideoBackWidget* m_infraredVideoBackWidget;

	// 机器人信息;
	// 可见光倍率;
	InputWidget* m_visibleLightRate;
	// 可见光焦距;
	InputWidget* m_visibleLightFocus;
	// 红外焦距;
	InputWidget* m_infraredFocus;
	// 云台水平旋转;
	InputWidget* m_PtzHRotate;
	// 云台垂直旋转;
	InputWidget* m_PtzVRotate;

	// 更新时钟;
	QTimer m_updateDataTimer;

	// 需要清空LineEdit的List;
	QList<InputWidget*> m_clearLineEditList;

	// 可见光、红外更新计数;
	int m_updateCount;

	// 设备 ID-Name Map;
	QMap<QString, QString> m_voltageLevelMap;
	QMap<QString, QString> m_areaNameMap;
	QMap<QString, QString> m_intervalNameMap;
	QMap<QString, QString> m_deviceTypeMap;
	QMap<QString, QString> m_deviceChildTypeMap;
	QMap<QString, WheelDevicePointNameStruct> m_devicePointPosMap;
	// QString - ssid, QString - deviceName;
	QMap<QString, QString> m_deviceNameMapBySsid;
	// 区域类型 ID-Name Map;
	QMap<int, QString> m_areasTypeMap;
	// 虚拟设备 ID-Name Map;
	QMap<int, QString> m_virtualDeviceNameMap;

	// 保存需要修改的设备信息;
	deviceConfigType m_needModifydevConfig;

	// 添加设备信息widget;
	BaseWidget* m_addDeviceInfoWidget;
	QStackedWidget* m_addDeviceStackedWidget;

	// 添加设备二次对焦及云台保存的信息;
	int m_focusNearValue;
	int m_focusFarValue;
	int m_zoomNearValue;
	int m_zoomFarValue;
	int m_ptzPanValue;
	int m_ptzTiltValue;
	int m_pointId;

	// 添加设备保存红外设备参数;
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

	// 保存添加设备窗口之前的操作记录;
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

	// 添加/修改 可见光设备近图参数;
	QWidget* m_addDeviceVisibleNearWidget;
	// 可见光控制-倍率;
	InputWidget* m_visibleLightScaleNearWidget;
	// 可见光控制-焦距;
	InputWidget* m_visibleLightFocusNearWidget;
	// 云台水平方向旋转;
	InputWidget* m_ptzHRotateWidget;
	// 云台垂直方向旋转;
	InputWidget* m_ptzVRotateWidget;
	QPushButton* m_pButtonAddDeviceVisibleNearNextStage;
	QPushButton* m_pButtonAddDeviceVisibleNearLastStage;

	// 可见光第一步;
	InputWidget* m_saveZoomShowWidget;
	InputWidget* m_saveFocusShowWidget;
	InputWidget* m_savePtzPanShowWidget;
	InputWidget* m_savePtzTiltShowWidget;
	// 可见光第二步;
	InputWidget* m_saveFarZoomShowWidget;
	InputWidget* m_saveFarFocusShowWidget;
	// 红外;
	InputWidget* m_saveInfraredFocusShowWidget;
	InputWidget* m_saveInfraredPtzPanShowWidget;
	InputWidget* m_saveInfraredPtzTiltShowWidget;

	QWidget* m_addDeviceVisibleFarWidget;
	// 可见光控制-倍率;
	InputWidget* m_visibleLightScaleFarWidget;
	// 可见光控制-焦距;
	InputWidget* m_visibleLightFocusFarWidget;
	QPushButton* m_pButtonAddDeviceVisibleFarLastStage;
	QPushButton* m_pButtonAddDeviceVisibleFinish;

	QWidget* m_addDeviceInfraredWidget;
	// 红外控制-焦距;
	InputWidget* m_infraredFocusWidget;	
	// 红外云台水平方向旋转;
	InputWidget* m_ptzHRotateInfraredWidget;
	// 红外云台垂直方向旋转;
	InputWidget* m_ptzVRotateInfraredWidget;
	QPushButton* m_pButtonAddDeviceInfraredLastStage;
	QPushButton* m_pButtonAddDeviceInfraredFinish;

	// 错误信息显示;
	QWidget* m_errorMessageBackWidget;
	QLabel* m_errorMessageLabel;

	// 键盘按键操作线程;
	KeyboardOperateThread* m_keyboardOperateThread;

	// 用于判断云台数据是否及时更新;
	QTimer m_updatePtzAngleDataTimer;

	QString m_strModifyDeviceId;
	bool m_isModifyDevice;

	BaseWidget* m_errorMsgListBackWidget;
	QListWidget* m_errorMsgListWidget;
	
	// 树控件数据查询对象;
	DLWheelPointTreeData* m_wheelPointTreeData;

	// 当前是否更新过实时数据;
	bool m_isUpdateRealTimeData;

	// 当前添加设备的uuid;
	QString m_currentAddDeviceId;

	// 操作按钮widget;
	QWidget* m_operationButtonBackWidget;

	CameraObject* m_cameraObject;

	bool m_bIsPressedState;
	QList<QStringList> sliZoomPtz;

	//CollectStandardizationWidget *m_collectStWidget;

	DLMapListWidget *m_pMapListWgt;		//下载地图
	DataTransfer *m_pThreadDataTransfer;

	bool m_bSMAPIsChanged;
	//QString m_strFileName;										//保存当前的文件名
	//bool m_bIsUploadSmap;										//记录是否上传机器人

	DLStatusBar *m_pStatusBar;									//状态栏
	QString m_strOpenMapPath;

    int m_infraredFocusValueCore = -1;

    bool m_bIsVisibleFull = false;
    bool m_bIsShrink = false;

//    InputWidget* m_deviceNameLabel_1;
};

#endif