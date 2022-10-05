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
	CommonVideo,					            // ����Ҫ��ť��������Ƶwidget;
	AssistVideo,					            // ������Ƶ����Ҫ�з��ĸ�����;
	VisibleLightControlVideo,		            // �ɼ�����Ƶ����4�����ư�ť;
    Wheel_Collect_VisibleLightControlVideo,		// ��ʽ�ɼ�����ɼ�����Ƶ����4�����ư�ť;
	InfraredControlVideo,			            // ������Ƶ����2�����ư�ť;
    Wheel_Collect_InfraredControlVideo,         // ��ʽ�ɼ����������Ƶ����4�����ư�ť;
    Wheel_VisibleLightControlVideo,             // ��ʽ�����˿ɼ�����Ƶ���ƴ���;
    Wheel_InfraredControlVideo,                 // ��ʽ�����˺�����Ƶ���ƴ���;
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
{//ȫ��videowidget����
	Q_OBJECT
public:
	FullScreenVideoWidget(QWidget* parent = NULL);


	~FullScreenVideoWidget() {};

	void setPixmap(QPixmap videoImage);				//����ͼƬ
	void SetCollectAim(bool bIsCollectAim);			//�����Ƿ�����ռ�Ŀ��
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

	// ������Ƶ�ߴ�Ϊ16:9;
	void setVideoSize1080P();

	// ������Ƶ�ߴ�Ϊ4:3;
	void setVideoSizeScale4_3();

	// ���ñ���;
	void setVideoZoomAbs(unsigned long zoom);

	// ���ý���;
	void setVideoFocusAbs(unsigned long focus);

    void setVideoZoomAndFocus(unsigned long zoom, unsigned long focus);
	// ���ú��⽹��;
	void setInfraredVideoFocusAbs(unsigned long focus);

	// ��ȡ��ǰ�������;
	int getCurrentZoomValue();

	// ��ȡ��ǰ�������;
	int getCurrentVideoFocusValue();

	// ��ȡ���⽹��;
	int getInfraredVideoFocus();

	// ���õ�ǰ�Ƿ���ʾѡ�п�;
	void setIsDrawSelectRect(bool isDrawSelectRect = false);

	// ���ÿɼ�����Ƶ��������;
	void setCameraObject(CameraObject* cameraObject);

	// ���ú�����Ƶ��������;
	//void setInfraredObject(InfraredTemperature* infraredTemperature);

	// ������̨��λ��ť�Ƿ�ɼ�;
	void setPtzResetButtonVisible(bool isVisible);

	// ��������;
	void cameraCapture(QString fileName);

    bool cameraCaptureBool(QString fileName);
    // ���������;
	void infraredCapture(QString filePath, QString fileName);

    // ��ʽ��������Ƶ¼��;
    void wheelRobotAudioRecord(bool isStart, QString fileName = "");

    // ��ȡ��������;
    HCNetCameraInterface* getHCNetCameraInterface();

protected:
	// �����¼�;
	void paintEvent(QPaintEvent *event);

	bool eventFilter(QObject *watched, QEvent *event);

private:
	// ��ʼ��������Ƶ;
	void initCommonVideo();
	// ��ʼ��������Ƶ,�з���Ƶ;
	void initAssistVideo();
	// ��ʼ���ɼ�����Ƶ(�����ư�ť);
	void initVisibleLightButton();
	// ��ʼ��������Ƶ(�����ư�ť);
	void initInFraredButton();
    // ��ʼ���ɼ�����Ƶ(�����ư�ť);
    void initWheelVisibleLightButton();
    // ��ʼ��������Ƶ(�����ư�ť);
    void initWheelInFraredButton();
    // ��ʼ����Ƶ���Ŵ���;
    void initVideoPlayWindow(PlayType playType);

	// ��ȡ�ֶ�Ѳ���·��;
	QString getPatrolRootPath(QString index, bool isNeedTime = true);

    // ��ȡ��ʽ����������·��;
    QString getWheelRobotCaptureFileName();

    // ��ȡ��ʽ��������Ƶ·��;
    QString getWheelRobotAudioFileName();

    // ��ȡ��ʽ�����˺���·��;
    QString getWheelRobotInfraredFileName();

signals:
	void signalVideoWidgetSelect();

	// ������굱ǰ����ĵ�����;
	void signalSendMousePressPoint(QPoint mousePressPoint, QSize windowSize, int zoomScale);

	// ֪ͨ����������̨�����;
	void signalOperatePtz(int operationType, bool isRelease);

	// ֪ͨ������̨��λ��ť���;
	void signalPtzResetButtonClicked();

	void MoveCollectEquipmentPosSignal(bool bIsCenter);

    void signalTreeText(bool isFull, VideoType type);

public slots:
	void onReceiveInfraredVideoImage(QPixmap image);

	// �ɼ��ⰴť���;
	void onButtonPressed(int buttonId);

	void onButtonReleased (int buttonId);

    // ��ʽ�ɼ��ⰴť���;
    void onWheelVisibleLightButtonReleased(int buttonId);

	// ���ⰴť���;
	void onInfraredButtonPressed(int buttonId);
    void onInfraredButtonReleased(int buttonId);
    // ��ʽ���ⰴť���;
    void onWheelInfraredButtonPressed(int buttonId);

	void VisibleTenRectSlot();						//��ʾʮ�־��ο�

    void setAppTypeClient(WheelAppClientSoftwareType type)
    {
        app_type_ = type;
    }

    void setVideoHeightFull(bool isFull)
    {
        m_bVideoHeightFull = isFull;
    }

private:
	// ����������Ƶ���ư�ť;
	QList<QPushButton*> m_VideoButtonList;
	// ��Ƶwidget����;
	VideoType m_videoType;
	// ������Ƶ�зֵ�widget;
	QList<QWidget* > m_videoWidgetList;

	// ��Ƶ���Ӷ���;
	CameraObject* m_cameraObject;

	// �л���Ƶ��С��ʾ����;
	FullScreenVideoWidget* m_videoBackWidget;
	// �Ƿ���Ҫ��ѡ��Ч��;
	bool m_isNeedSelect;
	// �Ƿ����ѡ�о��ο�;
	bool m_isDrawSelectRect;
	// ������Ƶ��ʾ�ӿڶ���;
	//InfraredTemperature* m_infraredTemperature;
	// ��ť����;
	QWidget* m_buttonBackWidget;
	// �Ƿ�ʼ¼��;
	bool m_isStartRecordAudio;
	
	// �жϵ�ǰ��굥��ʱ��;
	QTimer m_singleButtonClickTimer;
	// ��ǰ������ĵ�����;
	QPoint m_mousePressPoint;

	// ��̨��λ��ť;
	QPushButton* m_pButtonGoBack;

	QPushButton *m_pBtnVisibleTenRect;			//�Ƿ���ʾʮ�ֶԽ��ľ��ο�

    // ��Ƶ���Ŵ���;
    BaseWidget* m_videoBackWindow;
    VideoPlayer* m_videoPlayWidget;

    WheelAppClientSoftwareType app_type_;

    bool m_bVideoHeightFull = false;
};

#endif