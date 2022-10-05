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
		if ((av_gettime() - pThis->dwLastFrameRealtime) > 10 * 1000 * 1000) {//10s超时退出
			printf("主码流断开");
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
    // 初始化ffmpeg参数;
//     av_register_all(); //初始化FFMPEG  调用了这个才能正常使用编码器和解码器;
//     avformat_network_init();//初始化网络流格式,使用RTSP网络流时必须先执行;
    av_dict_set(&opts, "rtsp_transport", "tcp", 0);  //以udp方式打开，如果以tcp方式打开将udp替换为tcp
    av_dict_set(&opts, "stimeout", "5000000", 0);  //设置超时断开连接时间  5s  超过这个时间 av_read_frame avformat_open_input 断开
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

    // 如果之前是连接成功的，则需要释放内存;
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

// 开始播放视频;
void CameraObject::startPlay()
{
	this->start();
}

// 停止视频播放;
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
					// 防止打开界面之后立即关闭界面，导致m_HCNetCameraIntf被释放;
					if (m_HCNetCameraIntf != NULL && !m_isInfraredVideo)
					{
						m_HCNetCameraIntf->connect();
						m_HCNetCameraIntf->initCamera(m_HCNetCameraIntf->getCurrentChannel() + 1);
                        m_HCNetCameraIntf->startPlay(m_HCNetCameraIntf->getCurrentChannel(), (HWND)playHwnd->winId());
                    //    m_HCNetCameraIntf->startPlay(m_HCNetCameraIntf->getCurrentChannel(), m_hwnd);
						m_isFirstConnectSuccess = true;
						if (m_isMainCamera)
						{
							// 通知task初始化海康对象;
							emit signalNotifyCameraConnected();
						}
					}					
				}
			}
			else
			{
				// signalVideoConnectFailed();
				// qDebug() << "等待视频流...重试" << idx;
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


// 初始化FFmpeg;
bool CameraObject::initFFmpeg()
{
    int numBytes;
    int ret;
    m_iVideoStream = -1;

	av_register_all(); //初始化FFMPEG  调用了这个才能正常使用编码器和解码器
    if (m_isInfraredVideo)
    {
        ROS_INFO("CameraObject:infrared: showVideo av_register_all true!");
    }
	avformat_network_init();//初始化网络流格式,使用RTSP网络流时必须先执行
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

    // 判断是否是红外视频流;
    if (m_isInfraredVideo)
    {
        //strUrl = QString("rtsp://%1:%2/video").arg(m_cameraIp).arg(m_strRtspPort);//原端口为8554
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
	av_dict_set(&opts_, "rtsp_transport", "tcp", 0);  //以udp方式打开，如果以tcp方式打开将udp替换为tcp
	av_dict_set(&opts_, "stimeout", "5000000", 0);  //设置超时断开连接时间  2s  超过这个时间 av_read_frame avformat_open_input 断开
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

	// 打开视频解码器;
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
	packet = (AVPacket *)malloc(sizeof(AVPacket)); //分配一个packet;
//	av_new_packet(packet, y_size); //分配packet的数据;

								   //	av_dump_format(pFormatCtx, 0, file_path, 0); //输出视频信息;

	return true;
}

// 获取到视频数据，显示到界面;
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

				//把这个RGB数据 用QImage加载;
                
                if (!m_isPausePlay)
                {
                    try
                    {
                        m_connectAgain = true;
                        QImage tmpImg((uchar *)out_buffer, pCodecCtx->width, pCodecCtx->height, QImage::Format_RGB32);
                        emit signalGetCameraImage(QPixmap::fromImage(tmpImg));  //发送信号;
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