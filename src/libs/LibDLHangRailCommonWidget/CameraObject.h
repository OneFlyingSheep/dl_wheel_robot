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
	// ���������Ϣ;
	void setCameraConnectInfo(QString ip, int port, QString userName, QString passwd, QString strRtspPort = "554");

	// ��ʼ������Ƶ;
	void startPlay();
	// ֹͣ��Ƶ����;
	void stopPlay();

    // �Ƿ���ͣ����;
    void setIsPausePlay(bool isPlay);

    // ��ȡ��ǰ�����ӿ�;
	HCNetCameraInterface* getHCNetCameraInterface();

    // �����ɼ����������;
    void resetCamera(QString ip, int port, QString userName, QString passwd, int ffmpegPort);

    // ������Ҫ���������������, �����Ҫ�����������Ҫ����;
    void setNeedRestartCamera();

    // ���õ�ǰΪ����rtsp��Ƶ;
    void setInfraredVideo();

private:
	bool initFFmpeg();

	void showVideo();

	void run();
	// ��������;
    void restartCamera();
public:

	void OnVisibleZoomAbs(unsigned long zoom);

	void OnVisibleFocusAbs(unsigned long focus);

    void OnVisibleZoomAndFocus(unsigned long zoom, unsigned long focus);
	// ��ȡ��ǰ�������;
	int getCurrentZoomValue();

	// ��ȡ��ǰ�������;
	int getCurrentVideoFocusValue();

    bool cameraCaptureBool(QString fileName);
signals:
	// ���͵�ǰ��������Ƶ��ͼƬ;
	void signalGetCameraImage(QPixmap);

	// ��������ź�;
	void signalVisibleZoomInClickedSlot(bool b);

	void signalVisibleZoomOutClickedSlot(bool b);

	void signalVisibleFocusNearClickedSlot(bool b);

	void signalVisibleFocusFarClickedSlot(bool b);

	// ����ֶ�ץͼ;
	void signalVisibleCaptureClicked(QString fileName);

	// ���¼����Ƶ;
	void signalVisibleRecordAudio(bool isStartRecord, QString fileName = "");

	// ��ȡ�����ǰ�Ŵ����ź�;
	void signalGetCameraZoomValue();

	// ��ȡ�����ǰ�������ź�;
	void signalGetCameraFocusValue();

	// ֪ͨTask����������ӳɹ�(ֻ֪ͨһ��);
	void signalNotifyCameraConnected();

    // �����ɼ������;
    void signalRestartCamera();

	
private:
	// ffmpeg����;
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
	// ��ǰ�߳��Ƿ��˳�;
	bool m_isQuit;
	// ��ǰ�Ƿ��һ�γ�������;
	bool m_isFirstConnect;
	// ��ǰ�Ƿ��ǵ�һ��������;
	bool m_isFirstConnectSuccess;
	// ��Ƶ�Ƿ����ӳɹ�;
	bool m_isVideoConnectSucess;
	// ��������߳�;
	QThread m_cameraWorkerThread;
	// ���浱ǰ�������;
	int m_cameraZoomValue;
	// ���浱ǰ�������;
	int m_cameraFocusValue;
	// ��ǰ�Ƿ���������ͷ;
	bool m_isMainCamera;

	// ����������ӵ���Ϣ;
	QString m_cameraIp;
	int m_cameraPort;
	QString m_cameraUserName;
	QString m_cameraPasswd;
    QString m_strRtspPort;

    // �Ƿ���ͣ����;
    bool m_isPausePlay;

    // �Ƿ��Ǻ�����Ƶ;
    bool m_isInfraredVideo;

	bool m_isBlockDiagram;

    QWidget *playHwnd;

public:
	qint64 dwLastFrameRealtime;
	bool m_connectAgain;
};


#endif // !CAMERA_OBJECT_H
