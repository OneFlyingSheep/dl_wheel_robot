#include "CameraObject.h"
#include "LibDLHangRailConfigData/DLHangRailStationCfgData.h"
#include "LibHCNetCamera/HCNetCameraInterface.h"
#include <QDebug>
#include <QTime>
#include "LibDLHangRailHardwareCtrl/LibDLHangRailHardwareCtrl.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

CameraObject *pThis = 0;

int interrupt_cb(void *ctx)
{
//	CameraObject *pThis = (CameraObject *)ctx;
//	qint64 ds = av_gettime();
	if (!pThis->m_connectAgain)
	{
		if ((av_gettime() - pThis->dwLastFrameRealtime) > 10 * 1000 * 1000) {//10s��ʱ�˳�
			printf("�������Ͽ�");
			return AVERROR_EOF;
		}
	}
	
	return 0;
}

CameraObject::CameraObject(QObject* parent, bool isMainCamera)
	: QThread(parent)
	, m_HCNetCameraIntf(NULL)
	, m_isQuit(false)
	, m_isFirstConnect(true)
	, m_isVideoConnectSucess(false)
	, m_isFirstConnectSuccess(false)
	, m_cameraZoomValue(0)
	, m_cameraFocusValue(0)
	, m_isMainCamera(isMainCamera)
    , packet(NULL)
    , m_isPausePlay(false)
    , opts(NULL)
    , m_isInfraredVideo(false)
	, m_connectAgain(false)
{
	pThis = this;
    // ��ʼ��ffmpeg����;
//     av_register_all(); //��ʼ��FFMPEG  �����������������ʹ�ñ������ͽ�����;
//     avformat_network_init();//��ʼ����������ʽ,ʹ��RTSP������ʱ������ִ��;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);  //��udp��ʽ�򿪣������tcp��ʽ�򿪽�udp�滻Ϊtcp
    av_dict_set(&opts, "stimeout", "5000000", 0);  //���ó�ʱ�Ͽ�����ʱ��  5s  �������ʱ�� av_read_frame avformat_open_input �Ͽ�
}

void CameraObject::setCameraConnectInfo(QString ip, int port, QString userName, QString passwd, QString strRtspPort /* = "" */)
{
    playHwnd = new QWidget;
    playHwnd->setWindowFlags(Qt::FramelessWindowHint);
    playHwnd->setMaximumSize(QSize(1, 1));
    playHwnd->show();
    playHwnd->hide();

	ROS_INFO("!!!!!!!ip = %s, port = %d, user_name = %s, password = %s, rtsp_port = %s\n", ip.toStdString().c_str(), port, userName.toStdString().c_str(), passwd.toStdString().c_str(), strRtspPort.toStdString().c_str());
	m_cameraIp = ip;
	m_cameraPort = port;
	m_cameraUserName = userName;
	m_cameraPasswd = passwd;
    m_strRtspPort = strRtspPort;

    if (m_isInfraredVideo)
    {
        return;
    }

	m_HCNetCameraIntf = new HCNetCameraInterface(m_cameraIp, m_cameraUserName, m_cameraPasswd, m_cameraPort);
	m_HCNetCameraIntf->moveToThread(&m_cameraWorkerThread);
    m_cameraWorkerThread.start();

	connect(this, &CameraObject::signalVisibleZoomInClickedSlot, m_HCNetCameraIntf, &HCNetCameraInterface::OnVisibleZoomInClickedSlot, Qt::QueuedConnection);
	connect(this, &CameraObject::signalVisibleZoomOutClickedSlot, m_HCNetCameraIntf, &HCNetCameraInterface::OnVisibleZoomOutClickedSlot, Qt::QueuedConnection);
	connect(this, &CameraObject::signalVisibleFocusNearClickedSlot, m_HCNetCameraIntf, &HCNetCameraInterface::OnVisibleFocusNearClickedSlot, Qt::QueuedConnection);
	connect(this, &CameraObject::signalVisibleFocusFarClickedSlot, m_HCNetCameraIntf, &HCNetCameraInterface::OnVisibleFocusFarClickedSlot, Qt::QueuedConnection);
// 	connect(this, &CameraObject::signalGetCameraZoomValue, m_HCNetCameraIntf, &HCNetCameraInterface::onGetCameraZoomValue, Qt::QueuedConnection);
// 	connect(this, &CameraObject::signalGetCameraFocusValue, m_HCNetCameraIntf, &HCNetCameraInterface::onGetCameraFocusValue, Qt::QueuedConnection);
	connect(this, &CameraObject::signalVisibleCaptureClicked, m_HCNetCameraIntf, &HCNetCameraInterface::onGetCameraImage, Qt::QueuedConnection);
	connect(this, &CameraObject::signalVisibleRecordAudio, m_HCNetCameraIntf, &HCNetCameraInterface::onRecordCameraAudio, Qt::QueuedConnection);
}

CameraObject::~CameraObject()
{
	stopPlay();
    if (NULL != m_HCNetCameraIntf)
    {
        delete m_HCNetCameraIntf;
        m_HCNetCameraIntf = NULL;
    }

    this->quit();
    this->wait();

    if (!m_isInfraredVideo)
    {
        m_cameraWorkerThread.quit();
        m_cameraWorkerThread.wait();
    }
}

HCNetCameraInterface* CameraObject::getHCNetCameraInterface()
{
	return m_HCNetCameraIntf;
}

void CameraObject::resetCamera(QString ip, int port, QString userName, QString passwd, int ffmpegPort)
{
    if (!m_isInfraredVideo)
    {
        delete m_HCNetCameraIntf;
    }
    setCameraConnectInfo(ip, port, userName, passwd, QString::number(ffmpegPort));
	ROS_INFO("resetCamera!!!!!!!ip = %s, port = %d, user_name = %s, password = %s, rtsp_port = %d\n", ip.toStdString().c_str(), port, userName.toStdString().c_str(), passwd.toStdString().c_str(), ffmpegPort);

    stopPlay();
    this->quit();
    this->wait();

    // ���֮ǰ�����ӳɹ��ģ�����Ҫ�ͷ��ڴ�;
    if (m_isVideoConnectSucess)
    {
        m_isVideoConnectSucess = false;
        if (m_isMainCamera)
        {
            ROBOTSTATUS.setHCNetCameraConnectionStatus(false);
        }
        av_free(pFrameRGB);
        av_free(pFrame);
        avcodec_close(pCodecCtx);
        sws_freeContext(img_convert_ctx);
        avformat_close_input(&pFormatCtx);
    }

    m_isFirstConnect = true;
    m_isQuit = false;
    this->start();
}

void CameraObject::setNeedRestartCamera()
{
     connect(this, &CameraObject::signalRestartCamera, this, &CameraObject::restartCamera);
}

void CameraObject::setInfraredVideo()
{
    m_isInfraredVideo = true;
}

void CameraObject::restartCamera()
{
    resetCamera(m_cameraIp, m_cameraPort, m_cameraUserName, m_cameraPasswd, m_strRtspPort.toInt());
}

// ��ʼ������Ƶ;
void CameraObject::startPlay()
{
	this->start();
}

// ֹͣ��Ƶ����;
void CameraObject::stopPlay()
{
	m_isQuit = true;
}

void CameraObject::setIsPausePlay(bool isPlay)
{
    m_isPausePlay = isPlay;
}

void CameraObject::run()
{

	int idx = 1;
	while (!m_isQuit)
	{
		if (!m_isVideoConnectSucess)
		{
			if (!m_isFirstConnect)
			{
				QThread::sleep(10);
				m_isFirstConnect = false;
			}
            else
            {
                QThread::sleep(2);
            }

			if (initFFmpeg())
			{
                if (m_isInfraredVideo)
                {
                    ROS_INFO("CameraObject:infrared: initFFmpeg true!");
                }
                
// 				if (m_isMainCamera)
// 				{
//                     ROBOTSTATUS.setHCNetCameraConnectionStatus(true);
// 				}
				m_isVideoConnectSucess = true;
				if (!m_isFirstConnectSuccess)
				{
					// ��ֹ�򿪽���֮�������رս��棬����m_HCNetCameraIntf���ͷ�;
					if (m_HCNetCameraIntf != NULL && !m_isInfraredVideo)
					{
						m_HCNetCameraIntf->connect();
						m_HCNetCameraIntf->initCamera(m_HCNetCameraIntf->getCurrentChannel() + 1);
                        m_HCNetCameraIntf->startPlay(m_HCNetCameraIntf->getCurrentChannel(), (HWND)playHwnd->winId());
                    //    m_HCNetCameraIntf->startPlay(m_HCNetCameraIntf->getCurrentChannel(), m_hwnd);
						m_isFirstConnectSuccess = true;
						if (m_isMainCamera)
						{
							// ֪ͨtask��ʼ����������;
							emit signalNotifyCameraConnected();
						}
					}					
				}
			}
			else
			{
				// signalVideoConnectFailed();
				// qDebug() << "�ȴ���Ƶ��...����" << idx;
				//idx++;
                if (m_isInfraredVideo)
                {
                    ROS_INFO("CameraObject:infrared: initFFmpeg false!");
                }
			}
		}
		else
		{
        //    if (!m_isPausePlay)
        //    {
                showVideo();
        //    }
		}
	}
};


// ��ʼ��FFmpeg;
bool CameraObject::initFFmpeg()
{
    int numBytes;
    int ret;
    m_iVideoStream = -1;

	av_register_all(); //��ʼ��FFMPEG  �����������������ʹ�ñ������ͽ�����
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo av_register_all true!");
    }
	avformat_network_init();//��ʼ����������ʽ,ʹ��RTSP������ʱ������ִ��
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo avformat_network_init true!");
    }
    pFormatCtx = avformat_alloc_context();
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo avformat_network_init true!");
    }
//	pFormatCtx->interrupt_callback.opaque = pMainDlg; //C++
	pFormatCtx->interrupt_callback.callback = interrupt_cb;

    QString strUrl;

    // �ж��Ƿ��Ǻ�����Ƶ��;
    if (m_isInfraredVideo)
    {
        //strUrl = QString("rtsp://%1:%2/video").arg(m_cameraIp).arg(m_strRtspPort);//ԭ�˿�Ϊ8554
		strUrl = QString("rtsp://%1:%2@%3:%4/camera0").arg(m_cameraUserName).arg(m_cameraPasswd).arg(m_cameraIp).arg(m_strRtspPort);
    }
    else
    {
	    strUrl = QString("rtsp://%1:%2@%3:%4/h264/ch1/main/av_stream").arg(m_cameraUserName).arg(m_cameraPasswd).arg(m_cameraIp).arg(m_strRtspPort);
    }

	ROS_INFO("!!!!!!camera rtsp: %s", strUrl.toStdString().c_str());
	QByteArray bUrl(strUrl.toLatin1());
	QDateTime dateTime = QDateTime::currentDateTime();
	dwLastFrameRealtime = dateTime.toMSecsSinceEpoch() * 1000;

	AVDictionary* opts_ = NULL;
	av_dict_set(&opts_, "rtsp_transport", "tcp", 0);  //��udp��ʽ�򿪣������tcp��ʽ�򿪽�udp�滻Ϊtcp
	av_dict_set(&opts_, "stimeout", "5000000", 0);  //���ó�ʱ�Ͽ�����ʱ��  2s  �������ʱ�� av_read_frame avformat_open_input �Ͽ�
	Sleep(300);
	int result = avformat_open_input(&pFormatCtx, bUrl, NULL, &opts_);

	if (result != 0) {
        if (m_isInfraredVideo)
        {
            ROS_INFO("CameraObject:infrared: showVideo avformat_open_input false!");
        }
		return false;
	}

	if ((ret = avformat_find_stream_info(pFormatCtx, NULL)) < 0) {
        if (m_isInfraredVideo)
        {
            ROS_INFO("CameraObject:infrared: showVideo avformat_find_stream_info false!");
        }
		return ret;
	}

	ret = av_find_best_stream(pFormatCtx, AVMEDIA_TYPE_VIDEO, -1, -1, &pCodec, 0);
	if (ret < 0) {
        if (m_isInfraredVideo)
        {
            ROS_INFO("CameraObject:infrared: showVideo av_find_best_stream 1 false!");
        }
		return ret;
	}

	if (pCodec == NULL) {
        if (m_isInfraredVideo)
        {
            ROS_INFO("CameraObject:infrared: showVideo av_find_best_stream 2 false!");
        }
		return false;
	}

	m_iVideoStream = ret;
	m_videoStream = pFormatCtx->streams[m_iVideoStream];
	pCodecCtx = m_videoStream->codec;

    if (pCodecCtx->width == 0 || pCodecCtx->height == 0)
    {
        return false;
    }

	// ����Ƶ������;
	if (avcodec_open2(pCodecCtx, pCodec, NULL) < 0) {
        if (m_isInfraredVideo)
        {
            ROS_INFO("CameraObject:infrared: showVideo avcodec_open2 false!");
        }
		return false;
	}

	pFrame = av_frame_alloc();
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo av_frame_alloc 1 true!");
    }
	pFrameRGB = av_frame_alloc();
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo av_frame_alloc 2 true!");
    }

	img_convert_ctx = sws_getContext(pCodecCtx->width, pCodecCtx->height,
		pCodecCtx->pix_fmt, pCodecCtx->width, pCodecCtx->height,
		AV_PIX_FMT_RGB32, SWS_BICUBIC, NULL, NULL, NULL);
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo sws_getContext true!");
    }

	numBytes = avpicture_get_size(AV_PIX_FMT_RGB32, pCodecCtx->width, pCodecCtx->height);
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo avpicture_get_size true!");
    }

	out_buffer = (uint8_t *)av_malloc(numBytes * sizeof(uint8_t));
	avpicture_fill((AVPicture *)pFrameRGB, out_buffer, AV_PIX_FMT_RGB32,
		pCodecCtx->width, pCodecCtx->height);
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo avpicture_fill true!");
    }
	int y_size = pCodecCtx->width * pCodecCtx->height;
    if (packet != NULL)
    {
        free(packet);
        packet = NULL;
    }
	packet = (AVPacket *)malloc(sizeof(AVPacket)); //����һ��packet;
//	av_new_packet(packet, y_size); //����packet������;

								   //	av_dump_format(pFormatCtx, 0, file_path, 0); //�����Ƶ��Ϣ;

	return true;
}

// ��ȡ����Ƶ���ݣ���ʾ������;
void CameraObject::showVideo()
{
	int ret;
	int got_picture = 0;
	if (av_read_frame(pFormatCtx, packet) >= 0)
	{
		if (packet->stream_index == m_iVideoStream)
		{
			ret = avcodec_decode_video2(pCodecCtx, pFrame, &got_picture, packet);

			if (got_picture) {
				sws_scale(img_convert_ctx,
					(uint8_t const * const *)pFrame->data,
					pFrame->linesize, 0, pCodecCtx->height, pFrameRGB->data,
					pFrameRGB->linesize);

				//�����RGB���� ��QImage����;
                
                if (!m_isPausePlay)
                {
                    try
                    {
                        m_connectAgain = true;
                        QImage tmpImg((uchar *)out_buffer, pCodecCtx->width, pCodecCtx->height, QImage::Format_RGB32);
                        emit signalGetCameraImage(QPixmap::fromImage(tmpImg));  //�����ź�;
                    }
                    catch (...)
                    {
                        ROS_ERROR("CameraObject showVideo Error");
                    }
                }
			}
            else
            {
                if (m_isInfraredVideo)
                {
                    ROS_INFO("CameraObject:infrared: showVideo avcodec_decode_video2 false!");
                }
            }
		}
        
        av_free_packet(packet);
	}
	else
	{
        if (m_isInfraredVideo)
        {
            ROS_INFO("CameraObject:infrared: showVideo av_read_frame false!");
        }
// 		if (m_isMainCamera)
// 		{
//             ROBOTSTATUS.setHCNetCameraConnectionStatus(false);
// 		}
		m_connectAgain = false;
		m_isVideoConnectSucess = false;
		av_free(pFrameRGB);
		av_free(pFrame);
		avcodec_close(pCodecCtx);
		sws_freeContext(img_convert_ctx);
		avformat_close_input(&pFormatCtx);
		qDebug("av_read_frame error");
	}
}

/*
void CameraObject::OnVisibleZoomInClickedSlot(bool b)
{
	if (b)
	{
		m_HCNetCameraIntf->ZoomIn();
	}
	else
	{
		m_HCNetCameraIntf->ZoomStop();
	}
}

void CameraObject::OnVisibleZoomOutClickedSlot(bool b)
{
	QTime mmmm;
	mmmm.start();
	qDebug() << "OnVisibleZoomOutClickedSlot start";
	if (b)
	{
		m_HCNetCameraIntf->ZoomOut();
	}
	else
	{
		m_HCNetCameraIntf->ZoomStop();
	}
	qDebug() << "OnVisibleZoomOutClickedSlot end" << mmmm.elapsed();
}

void CameraObject::OnVisibleFocusNearClickedSlot(bool b)
{
	if (b)
	{
		m_HCNetCameraIntf->FocusNear();
	}
	else
	{
		m_HCNetCameraIntf->FocusStop();
	}
}

void CameraObject::OnVisibleFocusFarClickedSlot(bool b)
{
	if (b)
	{
		m_HCNetCameraIntf->FocusFar();
	}
	else
	{
		m_HCNetCameraIntf->FocusStop();
	}
}
*/

void CameraObject::OnVisibleZoomAbs(unsigned long zoom)
{
	NET_DVR_PTZPOS ptz;
	ptz.wAction = 4;
	ptz.wZoomPos = zoom;
	m_HCNetCameraIntf->setPtzPos(ptz);
}

void CameraObject::OnVisibleFocusAbs(unsigned long focus)
{
	m_HCNetCameraIntf->setFocus(focus);
}

void CameraObject::OnVisibleZoomAndFocus(unsigned long zoom, unsigned long focus)
{
    m_HCNetCameraIntf->setZoomAndFocus(zoom, focus);
}

int CameraObject::getCurrentZoomValue()
{
	if (m_HCNetCameraIntf != NULL && m_isVideoConnectSucess)
	{
		emit signalGetCameraZoomValue();
	//	m_cameraZoomValue = m_HCNetCameraIntf->getPtzPos().wZoomPos;
		m_cameraZoomValue = m_HCNetCameraIntf->getHCZoom();
		return m_cameraZoomValue;
	}

	return -1;
}

int CameraObject::getCurrentVideoFocusValue()
{
	if (m_HCNetCameraIntf != NULL && m_isVideoConnectSucess)
	{
		emit signalGetCameraFocusValue();
	//	m_cameraFocusValue = m_HCNetCameraIntf->getFocus();
		m_cameraFocusValue = m_HCNetCameraIntf->getHCFocus();
		return m_cameraFocusValue;
	}

	return -1;
}

bool CameraObject::cameraCaptureBool(QString fileName)
{
    return m_HCNetCameraIntf->onGetCameraImageBool(fileName);
}