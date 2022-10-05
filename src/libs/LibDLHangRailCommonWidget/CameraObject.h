#ifndef CAMERA_OBJECT_H
#define CAMERA_OBJECT_H

#include <QThread>
#include <QTimer>
#include <QPainter>
#include <QMutex>
#include <QWidget>

extern "C"
{
	#include "libavcodec/avcodec.h"
	#include "libavformat/avformat.h"
	#include "libavutil/time.h"
	#include "libavutil/pixfmt.h"
	#include "libswscale/swscale.h"
	#include "libswresample/swresample.h"
}

class HCNetCameraInterface;

class CameraObject : public QThread
{
	Q_OBJECT
public:
	CameraObject(QObject* parent = NULL, bool isMainCamera = false);

	~CameraObject();
	// 设置相机信息;
	void setCameraConnectInfo(QString ip, int port, QString userName, QString passwd, QString strRtspPort = "554");

	// 开始播放视频;
	void startPlay();
	// 停止视频播放;
	void stopPlay();

    // 是否暂停播放;
    void setIsPausePlay(bool isPlay);

    // 获取当前海康接口;
	HCNetCameraInterface* getHCNetCameraInterface();

    // 重启可见光相机连接;
    void resetCamera(QString ip, int port, QString userName, QString passwd, int ffmpegPort);

    // 设置需要重新启动相机功能, 如果需要重启相机，需要调用;
    void setNeedRestartCamera();

    // 设置当前为红外rtsp视频;
    void setInfraredVideo();

private:
	bool initFFmpeg();

	void showVideo();

	void run();
	// 重新启动;
    void restartCamera();
public:

	void OnVisibleZoomAbs(unsigned long zoom);

	void OnVisibleFocusAbs(unsigned long focus);

    void OnVisibleZoomAndFocus(unsigned long zoom, unsigned long focus);
	// 获取当前相机倍数;
	int getCurrentZoomValue();

	// 获取当前相机焦距;
	int getCurrentVideoFocusValue();

    bool cameraCaptureBool(QString fileName);
signals:
	// 发送当前解析的视频流图片;
	void signalGetCameraImage(QPixmap);

	// 相机操作信号;
	void signalVisibleZoomInClickedSlot(bool b);

	void signalVisibleZoomOutClickedSlot(bool b);

	void signalVisibleFocusNearClickedSlot(bool b);

	void signalVisibleFocusFarClickedSlot(bool b);

	// 相机手动抓图;
	void signalVisibleCaptureClicked(QString fileName);

	// 相机录制音频;
	void signalVisibleRecordAudio(bool isStartRecord, QString fileName = "");

	// 获取相机当前放大倍数信号;
	void signalGetCameraZoomValue();

	// 获取相机当前焦距数信号;
	void signalGetCameraFocusValue();

	// 通知Task对象相机连接成功(只通知一次);
	void signalNotifyCameraConnected();

    // 重启可见光相机;
    void signalRestartCamera();

	
private:
	// ffmpeg解码;
    AVDictionary* opts;
	AVFormatContext * pFormatCtx;
	AVCodecContext *pCodecCtx;
	AVCodec *pCodec;
	AVFrame *pFrame, *pFrameRGB;
	AVPacket *packet;
	uint8_t *out_buffer;
	AVStream* m_videoStream;
	SwsContext *img_convert_ctx;
	int m_iVideoStream;

	HCNetCameraInterface * m_HCNetCameraIntf;
	QTimer m_startPlayTimer;
	bool m_isReceiveImageBuf;
	char* m_imageBuf;
	QSize m_imageSize;
	QMutex m_mutex;
	// 当前线程是否退出;
	bool m_isQuit;
	// 当前是否第一次尝试连接;
	bool m_isFirstConnect;
	// 当前是否是第一次连接上;
	bool m_isFirstConnectSuccess;
	// 视频是否连接成功;
	bool m_isVideoConnectSucess;
	// 相机操作线程;
	QThread m_cameraWorkerThread;
	// 保存当前相机倍数;
	int m_cameraZoomValue;
	// 保存当前相机倍数;
	int m_cameraFocusValue;
	// 当前是否是主摄像头;
	bool m_isMainCamera;

	// 保存相机连接的信息;
	QString m_cameraIp;
	int m_cameraPort;
	QString m_cameraUserName;
	QString m_cameraPasswd;
    QString m_strRtspPort;

    // 是否暂停播放;
    bool m_isPausePlay;

    // 是否是红外视频;
    bool m_isInfraredVideo;

	bool m_isBlockDiagram;

    QWidget *playHwnd;

public:
	qint64 dwLastFrameRealtime;
	bool m_connectAgain;
};


#endif // !CAMERA_OBJECT_H
