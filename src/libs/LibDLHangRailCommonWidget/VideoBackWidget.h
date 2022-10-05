#ifndef VIDEO_BACK_WIDGET_H
#define VIDEO_BACK_WIDGET_H

#include <QWidget>
#include <QDockWidget>
#include <QPushButton>
#include "LibHCNetCamera/HCNetCameraInterface.h"
#include <QThread>
#include <QTimer>
#include <QDebug>
#include <QKeyEvent>
#include <QLabel>
#include <QHBoxLayout>
//#include "LibDLInfraredTemperature/InfraredTemperature.h"
#include <QVideoWidget>
#include <QPainter>
#include "LibDLHangRailCommonWidget/VideoPlayer.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"

enum VideoType
{
	CommonVideo,					            // 不需要按钮，正常视频widget;
	AssistVideo,					            // 辅助视频，需要切分四个窗口;
	VisibleLightControlVideo,		            // 可见光视频，带4个控制按钮;
    Wheel_Collect_VisibleLightControlVideo,		// 轮式采集程序可见光视频，带4个控制按钮;
	InfraredControlVideo,			            // 红外视频，带2个控制按钮;
    Wheel_Collect_InfraredControlVideo,         // 轮式采集程序红外视频，带4个控制按钮;
    Wheel_VisibleLightControlVideo,             // 轮式机器人可见光视频控制窗口;
    Wheel_InfraredControlVideo,                 // 轮式机器人红外视频控制窗口;
};

enum VideoButtonType
{
	FocusStrong = 0,
	focusWeak,
	ZoomOut,
	ZoomIn,
	Capture,
	RecordAudio,
    CameraRestart,
	CameraRestoration,
	VideoButtonTypeNum
};

enum InfraredButtonType
{
	Infrared_FocusStrong = 0,
	Infrared_FocusWeak,
	Infrared_AutoFocus,
	InfraredCapture,
    InfraredRestart
};

class CameraObject;
class VideoPlayer;
class BaseWidget;

class FullScreenVideoWidget : public QVideoWidget
{//全屏videowidget的类
	Q_OBJECT
public:
	FullScreenVideoWidget(QWidget* parent = NULL);


	~FullScreenVideoWidget() {};

	void setPixmap(QPixmap videoImage);				//设置图片
	void SetCollectAim(bool bIsCollectAim);			//设置是否绘制收集目标
	bool GetCollectAim();

signals:
	void signalWindowClose();

protected:
	void paintEvent(QPaintEvent *event);

private:
	QPixmap m_videoShowPixmap;
	bool m_isSetPixmap;
	bool m_bIsCollectAim;
};

class VideoBackWidget : public QWidget
{
	Q_OBJECT

public:
	VideoBackWidget(VideoType videoType, bool isNeedSelect = false, QWidget *parent = NULL);
	~VideoBackWidget();

	// 设置视频尺寸为16:9;
	void setVideoSize1080P();

	// 设置视频尺寸为4:3;
	void setVideoSizeScale4_3();

	// 设置倍率;
	void setVideoZoomAbs(unsigned long zoom);

	// 设置焦距;
	void setVideoFocusAbs(unsigned long focus);

    void setVideoZoomAndFocus(unsigned long zoom, unsigned long focus);
	// 设置红外焦距;
	void setInfraredVideoFocusAbs(unsigned long focus);

	// 获取当前相机倍数;
	int getCurrentZoomValue();

	// 获取当前相机焦距;
	int getCurrentVideoFocusValue();

	// 获取红外焦距;
	int getInfraredVideoFocus();

	// 设置当前是否显示选中框;
	void setIsDrawSelectRect(bool isDrawSelectRect = false);

	// 设置可见光视频操作对象;
	void setCameraObject(CameraObject* cameraObject);

	// 设置红外视频操作对象;
	//void setInfraredObject(InfraredTemperature* infraredTemperature);

	// 设置云台复位按钮是否可见;
	void setPtzResetButtonVisible(bool isVisible);

	// 红外拍照;
	void cameraCapture(QString fileName);

    bool cameraCaptureBool(QString fileName);
    // 红外光拍照;
	void infraredCapture(QString filePath, QString fileName);

    // 轮式机器人音频录制;
    void wheelRobotAudioRecord(bool isStart, QString fileName = "");

    // 获取海康对象;
    HCNetCameraInterface* getHCNetCameraInterface();

protected:
	// 绘制事件;
	void paintEvent(QPaintEvent *event);

	bool eventFilter(QObject *watched, QEvent *event);

private:
	// 初始化正常视频;
	void initCommonVideo();
	// 初始化辅助视频,切分视频;
	void initAssistVideo();
	// 初始化可见光视频(带控制按钮);
	void initVisibleLightButton();
	// 初始化红外视频(带控制按钮);
	void initInFraredButton();
    // 初始化可见光视频(带控制按钮);
    void initWheelVisibleLightButton();
    // 初始化红外视频(带控制按钮);
    void initWheelInFraredButton();
    // 初始化视频播放窗口;
    void initVideoPlayWindow(PlayType playType);

	// 获取手动巡检根路径;
	QString getPatrolRootPath(QString index, bool isNeedTime = true);

    // 获取轮式机器人拍照路径;
    QString getWheelRobotCaptureFileName();

    // 获取轮式机器人音频路径;
    QString getWheelRobotAudioFileName();

    // 获取轮式机器人红外路径;
    QString getWheelRobotInfraredFileName();

signals:
	void signalVideoWidgetSelect();

	// 发送鼠标当前点击的点坐标;
	void signalSendMousePressPoint(QPoint mousePressPoint, QSize windowSize, int zoomScale);

	// 通知按键控制云台、相机;
	void signalOperatePtz(int operationType, bool isRelease);

	// 通知发送云台复位按钮点击;
	void signalPtzResetButtonClicked();

	void MoveCollectEquipmentPosSignal(bool bIsCenter);

    void signalTreeText(bool isFull, VideoType type);

public slots:
	void onReceiveInfraredVideoImage(QPixmap image);

	// 可见光按钮点击;
	void onButtonPressed(int buttonId);

	void onButtonReleased (int buttonId);

    // 轮式可见光按钮点击;
    void onWheelVisibleLightButtonReleased(int buttonId);

	// 红外按钮点击;
	void onInfraredButtonPressed(int buttonId);
    void onInfraredButtonReleased(int buttonId);
    // 轮式红外按钮点击;
    void onWheelInfraredButtonPressed(int buttonId);

	void VisibleTenRectSlot();						//显示十字矩形框

    void setAppTypeClient(WheelAppClientSoftwareType type)
    {
        app_type_ = type;
    }

    void setVideoHeightFull(bool isFull)
    {
        m_bVideoHeightFull = isFull;
    }

private:
	// 用来保存视频控制按钮;
	QList<QPushButton*> m_VideoButtonList;
	// 视频widget类型;
	VideoType m_videoType;
	// 辅助视频切分的widget;
	QList<QWidget* > m_videoWidgetList;

	// 视频连接对象;
	CameraObject* m_cameraObject;

	// 切换视频大小显示窗口;
	FullScreenVideoWidget* m_videoBackWidget;
	// 是否需要有选中效果;
	bool m_isNeedSelect;
	// 是否绘制选中矩形框;
	bool m_isDrawSelectRect;
	// 红外视频显示接口对象;
	//InfraredTemperature* m_infraredTemperature;
	// 按钮背景;
	QWidget* m_buttonBackWidget;
	// 是否开始录音;
	bool m_isStartRecordAudio;
	
	// 判断当前鼠标单击时钟;
	QTimer m_singleButtonClickTimer;
	// 当前鼠标点击的点坐标;
	QPoint m_mousePressPoint;

	// 云台复位按钮;
	QPushButton* m_pButtonGoBack;

	QPushButton *m_pBtnVisibleTenRect;			//是否显示十字对焦的矩形框

    // 视频播放窗口;
    BaseWidget* m_videoBackWindow;
    VideoPlayer* m_videoPlayWidget;

    WheelAppClientSoftwareType app_type_;

    bool m_bVideoHeightFull = false;
};

#endif