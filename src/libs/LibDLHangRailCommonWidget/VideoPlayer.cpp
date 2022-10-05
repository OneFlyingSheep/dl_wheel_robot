#include "VideoPlayer.h"
#include <QTime>
#include <QEvent>
#include <QMouseEvent>
#include <QHBoxLayout>
#include <QDebug>
#include <QApplication>
#include <QPainter>
#include <QAudioProbe>
#include "windows.h"

#include "LibDLNewInfraredSDK/InfraredChoose.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
// #include "LibDLInfraredTemperature/InfraredTemperature.h"
// #include "LibDLInfraredTemperature/TempChoose.h"
#define INFRADE_WIDTH 640			// 红外视频分辨率-宽;
#define INFRADE_HEIGHT 480			// 红外视频分辨率-高;

InfraredImageWidget::InfraredImageWidget(QWidget* parent /* = NULL */)
	: QWidget(parent)
	, m_infraredValue("")
	, m_isDrawInfraredValue(false)
{
//  	m_infraredTemperatureObject = TempChoose::getObject();
// 	m_infraredTemperatureObject->startlink();
//	m_infraredTemperatureObject->setInfroWid(INFRADE_WIDTH, INFRADE_HEIGHT);
// 	m_infraredTemperatureObject->startlink();
// 	QString filePath = QApplication::applicationDirPath() + "/InfraredImage/";
// 	setInfraredImagePath(filePath + "ff.jpg", filePath + "ff.ddt");
    if (WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().infraredManufacturer == 0)
    {
        INFRARED_BASE.setInfraredType(SDK_GUIDE_INFRARED);
    }
    else
    {
        INFRARED_BASE.setInfraredType(SDK_JUGE_INFRARED);
    }
    
	this->setMouseTracking(true);
}

InfraredImageWidget::~InfraredImageWidget()
{

}

void InfraredImageWidget::setInfraredImagePath(QString strBmpPath, QString strDdtPath)
{
	if (!QFile::exists(strBmpPath))
	{
// 		m_infraredTemperatureObject->getTempAnalysisFile(strDdtPath);
// 		QPixmap image(this->size());
// 		QPainter painter(&image);
// 		painter.fillRect(image.rect(), Qt::black);
// 		m_infraredImage = image;
	}
	else
	{
		m_infraredImage = QPixmap(strBmpPath);
	//	Sleep(1000);
	//	m_infraredTemperatureObject->setTempAnalysisFile(strBmpPath.replace("_result", "-temp"));
		m_strBmpPath = strBmpPath;
        if (WHEEL_ROBOT_BACKGROUND_CONFIG.getCoreCfg().infraredManufacturer >= 1)
        {
            INFRARED_OBJECT->setInfraredDDTFile(m_strBmpPath);
        }
	}
}

void InfraredImageWidget::mouseMoveEvent(QMouseEvent *event)
{
	
	m_mouseMovePoint = event->pos();
	int translateX = 1.0 * m_mouseMovePoint.x() * INFRADE_WIDTH / this->width();
	int translateY = INFRADE_HEIGHT - (1.0 * m_mouseMovePoint.y() * INFRADE_HEIGHT / this->height());
//     AnalysisCoordinates coor;
//     coor.x_AnCoorLeft = translateX;
//     coor.y_AnCoorLeft = translateY;
    float infraredValue = 0.0;
//     if (m_infraredTemperatureObject->getMouseMoveBmpTemp(coor, infraredValue))
//     {
//         m_infraredValue = QString::number(infraredValue);
//     }
    InfraredPointBase pointBase;
    pointBase.locate_x = translateX;
    pointBase.locate_y = translateY;

    INFRARED_OBJECT->getInfraredPointTemp(m_strBmpPath, pointBase, infraredValue);

    m_infraredValue = QString::number(infraredValue);
	m_isDrawInfraredValue = true;
	update();
}

void InfraredImageWidget::leaveEvent(QEvent *event)
{
	m_isDrawInfraredValue = false;
	update();
}

void InfraredImageWidget::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);
	QPainter painter(this);
	painter.drawPixmap(this->rect(), m_infraredImage.scaled(this->size()));

	if (m_isDrawInfraredValue)
	{
		QFont font;
		font.setPixelSize(20);
		painter.setFont(font);
		painter.setPen(Qt::green);
		painter.drawText(m_mouseMovePoint, m_infraredValue);
	}
}

// This function returns the maximum possible sample value for a given audio format
qreal getPeakValue(const QAudioFormat& format)
{
    // Note: Only the most common sample formats are supported
    if (!format.isValid())
        return qreal(0);

    if (format.codec() != "audio/pcm")
        return qreal(0);

    switch (format.sampleType()) {
    case QAudioFormat::Unknown:
        break;
    case QAudioFormat::Float:
        if (format.sampleSize() != 32) // other sample formats are not supported
            return qreal(0);
        return qreal(1.00003);
    case QAudioFormat::SignedInt:
        if (format.sampleSize() == 32)
            return qreal(INT_MAX);
        if (format.sampleSize() == 16)
            return qreal(SHRT_MAX);
        if (format.sampleSize() == 8)
            return qreal(CHAR_MAX);
        break;
    case QAudioFormat::UnSignedInt:
        if (format.sampleSize() == 32)
            return qreal(UINT_MAX);
        if (format.sampleSize() == 16)
            return qreal(USHRT_MAX);
        if (format.sampleSize() == 8)
            return qreal(UCHAR_MAX);
        break;
    }

    return qreal(0);
}

template <class T>
QVector<qreal> getBufferLevels(const T *buffer, int frames, int channels)
{
    QVector<qreal> max_values;
    max_values.fill(0, channels);

    for (int i = 0; i < frames; ++i) {
        for (int j = 0; j < channels; ++j) {
            qreal value = qAbs(qreal(buffer[i * channels + j]));
            if (value > max_values.at(j))
                max_values.replace(j, value);
        }
    }

    return max_values;
}

QVector<qreal> getBufferLevels(const QAudioBuffer& buffer)
{
    QVector<qreal> values;

    if (!buffer.isValid())
        return values;

    if (!buffer.format().isValid() || buffer.format().byteOrder() != QAudioFormat::LittleEndian)
        return values;

    if (buffer.format().codec() != "audio/pcm")
        return values;

    int channelCount = buffer.format().channelCount();
    values.fill(0, channelCount);
    qreal peak_value = getPeakValue(buffer.format());
    if (qFuzzyCompare(peak_value, qreal(0)))
        return values;

    switch (buffer.format().sampleType()) {
    case QAudioFormat::Unknown:
    case QAudioFormat::UnSignedInt:
        if (buffer.format().sampleSize() == 32)
            values = getBufferLevels(buffer.constData<quint32>(), buffer.frameCount(), channelCount);
        if (buffer.format().sampleSize() == 16)
            values = getBufferLevels(buffer.constData<quint16>(), buffer.frameCount(), channelCount);
        if (buffer.format().sampleSize() == 8)
            values = getBufferLevels(buffer.constData<quint8>(), buffer.frameCount(), channelCount);
        for (int i = 0; i < values.size(); ++i)
            values[i] = qAbs(values.at(i) - peak_value / 2) / (peak_value / 2);
        break;
    case QAudioFormat::Float:
        if (buffer.format().sampleSize() == 32) {
            values = getBufferLevels(buffer.constData<float>(), buffer.frameCount(), channelCount);
            for (int i = 0; i < values.size(); ++i)
                values[i] /= peak_value;
        }
        break;
    case QAudioFormat::SignedInt:
        if (buffer.format().sampleSize() == 32)
            values = getBufferLevels(buffer.constData<qint32>(), buffer.frameCount(), channelCount);
        if (buffer.format().sampleSize() == 16)
            values = getBufferLevels(buffer.constData<qint16>(), buffer.frameCount(), channelCount);
        if (buffer.format().sampleSize() == 8)
            values = getBufferLevels(buffer.constData<qint8>(), buffer.frameCount(), channelCount);
        for (int i = 0; i < values.size(); ++i)
            values[i] /= peak_value;
        break;
    }

    return values;
}

VideoPlayer::VideoPlayer(QWidget *parent /* = Q_NULLPTR */, PlayType playType /* = VideoPlay */)
	: QWidget(parent)
	, m_videoTotalTime(0)
	, m_mediaPlayer(NULL)
    , m_playType(playType)
    , m_videoWidget(NULL)
    , m_waveformWidget(NULL)
{
    switch (m_playType)
    {
    case AudioPlay:
        initAudioPlayerControl();
        break;
    case VideoPlay:
        initVideoPlayerControl();
        break;
    default:
        break;
    }

    m_audioRefreshTimer.setInterval(600);
    connect(&m_audioRefreshTimer, &QTimer::timeout, this, [=] {
        m_mediaPlayer->setPosition(m_mediaPlayer->position());
    });
}

void VideoPlayer::initMediaPlayer()
{
	if (m_mediaPlayer == NULL)
	{
		m_mediaPlayer = new QMediaPlayer(this);
		/*m_mediaPlayer->setAudioRole(QAudio::VideoRole);*/
		connect(m_mediaPlayer, &QMediaPlayer::durationChanged, this, &VideoPlayer::onVideoTotalTimeChanged);
		connect(m_mediaPlayer, &QMediaPlayer::positionChanged, this, &VideoPlayer::onVideoProcessChanged);
		connect(m_mediaPlayer, &QMediaPlayer::stateChanged, this, &VideoPlayer::onVideoStatusChanged);

        if (m_playType == AudioPlay)
        {
            QAudioProbe* audioProbe = new QAudioProbe(this);
            connect(audioProbe, SIGNAL(audioBufferProbed(QAudioBuffer)), this, SLOT(processBuffer(QAudioBuffer)));
            audioProbe->setSource(m_mediaPlayer);
        }
        else if (m_playType == VideoPlay)
        {
            // 必须加上下面代码,否则播放出错;
            m_videoProbe = new QVideoProbe(this);
            m_videoProbe->setSource(m_mediaPlayer);

            m_mediaPlayer->setVideoOutput(m_videoWidget);
        }
	}
}

void VideoPlayer::initVideoPlayerControl()
{
	m_videoWidget = new QVideoWidget(this);
	m_videoWidget->installEventFilter(this);
	m_videoWidget->setMouseTracking(true);

	m_videoWidget->setObjectName("VideoPlayer");
	m_videoWidget->setStyleSheet("QWidget{background:black;}");

	m_pButtonVideoPlay = new QPushButton(this);
	m_pButtonVideoPlay->setFixedSize(QSize(32, 32));
	m_pButtonVideoPlay->setCheckable(true);
	m_pButtonVideoPlay->setChecked(true);
	m_pButtonVideoPlay->setStyleSheet("QPushButton{border-image:url(:/Resources/VideoPlayer/videoStop.png);}\
										QPushButton:hover{border-image:url(:/Resources/VideoPlayer/videoStop_hover.png);}\
										QPushButton:pressed{margin:3px;}\
										QPushButton::checked{border-image:url(:/Resources/VideoPlayer/videoPlay.png);}\
										QPushButton::checked:hover{border-image:url(:/Resources/VideoPlayer/videoPlay_hover.png);}\
										QPushButton::checked:pressed{ margin:3px; }");
	
	m_pButtonVolumeSwitch = new QPushButton(this);
	m_pButtonVolumeSwitch->setFixedSize(QSize(32, 32));
	m_pButtonVolumeSwitch->setCheckable(true);
	m_pButtonVolumeSwitch->setStyleSheet("QPushButton{border-image:url(:/Resources/VideoPlayer/videoSoundOn.png);}\
										QPushButton:hover{border-image:url(:/Resources/VideoPlayer/videoSoundOn_hover.png);}\
										QPushButton:pressed{margin:3px;}\
										QPushButton::checked{border-image:url(:/Resources/VideoPlayer/videoSoundOff.png);}\
										QPushButton::checked:hover{border-image:url(:/Resources/VideoPlayer/videoSoundOff_hover.png);}\
										QPushButton::checked:pressed{ margin:3px; }");

	m_pButtonVideoFullScreen = new QPushButton(this);
	m_pButtonVideoFullScreen->setFixedSize(QSize(32, 32));
	m_pButtonVideoFullScreen->setCheckable(true);
	m_pButtonVideoFullScreen->setStyleSheet("QPushButton{border-image:url(:/Resources/VideoPlayer/fullScreen.png);}\
											QPushButton:hover{border-image:url(:/Resources/VideoPlayer/fullScreen_hover.png);}\
											QPushButton:pressed{ margin:3px; }\
											QPushButton::checked{border-image:url(:/Resources/VideoPlayer/restoreScreen.png);}\
											QPushButton::checked:hover{border-image:url(:/Resources/VideoPlayer/restoreScreen_hover.png);}\
											QPushButton::checked:pressed{ margin:3px; }");


	m_videoProcessTimeLabel = new QLabel(this);
	m_videoProcessTimeLabel->setText("0:00 / 0:00");
	m_videoProcessTimeLabel->setStyleSheet("color:#cdcdcd;font-size:13px;");

	m_videoVolumeSlider = new QSlider(Qt::Horizontal, this);
	m_videoVolumeSlider->setFixedWidth(100);
	m_videoVolumeSlider->setRange(0, 100);
	m_videoVolumeSlider->setValue(50);
	m_videoVolumeSlider->setStyleSheet("QSlider::groove:horizontal{border:1px solid white;height:1px;left:6px; right:6px;}\
							QSlider::handle:horizontal{border:0px;border-image:url(:/Resources/VideoPlayer/SliderButton.png);width:20px;margin:-9px -6px -9px -6px;}\
							QSlider::add-page:horizontal{background:white;}QSlider::sub-page:horizontal{background:rgb(33, 150, 243);}");


	m_videoPlayProcessSlider = new QSlider(Qt::Horizontal, this);
	m_videoPlayProcessSlider->setStyleSheet("QSlider::groove:horizontal{border:1px solid white;height:1px;left:6px; right:6px;}\
							QSlider::handle:horizontal{border:0px;border-image:url(:/Resources/VideoPlayer/SliderButton.png);width:20px;margin:-9px -6px -9px -6px;}\
							QSlider::add-page:horizontal{background:white;}QSlider::sub-page:horizontal{background:rgb(33, 150, 243);}");

	connect(m_pButtonVideoPlay, &QPushButton::clicked, this, &VideoPlayer::onVideoPlayButtonClicked);
	connect(m_pButtonVolumeSwitch, &QPushButton::clicked, this, &VideoPlayer::onVideoVoiceSwitchButtonClicked);
	connect(m_pButtonVideoFullScreen, &QPushButton::clicked, this, &VideoPlayer::onFullScreenButtonClicked);

	connect(m_videoVolumeSlider, &QSlider::valueChanged, this, &VideoPlayer::onVolumeChanged);
	connect(m_videoPlayProcessSlider, &QSlider::sliderMoved, this, &VideoPlayer::onChangeVideoProcess);

	m_videoControlWidget = new QWidget;
	m_videoControlWidget->setFixedHeight(60);
	m_videoControlWidget->setObjectName("VideoControlWidget");
	m_videoControlWidget->setStyleSheet("QWidget#VideoControlWidget{background:rgba(0, 0, 0, 180);}");

	QHBoxLayout* hVideoControlLayout = new QHBoxLayout();
	hVideoControlLayout->addWidget(m_pButtonVideoPlay);
	hVideoControlLayout->addWidget(m_videoProcessTimeLabel);
	hVideoControlLayout->addStretch();
	hVideoControlLayout->addWidget(m_pButtonVolumeSwitch);
	hVideoControlLayout->addWidget(m_videoVolumeSlider);
	hVideoControlLayout->addWidget(m_pButtonVideoFullScreen);
	hVideoControlLayout->setSpacing(10);
	hVideoControlLayout->setMargin(0);

	QVBoxLayout* vVideoControlWidgetLayout = new QVBoxLayout(m_videoControlWidget);
	vVideoControlWidgetLayout->addWidget(m_videoPlayProcessSlider);
	vVideoControlWidgetLayout->addLayout(hVideoControlLayout);
	vVideoControlWidgetLayout->setSpacing(0);
	vVideoControlWidgetLayout->setMargin(0);
	vVideoControlWidgetLayout->setContentsMargins(5, 0, 5, 5);

	QVBoxLayout* vVideoWidgetLayout = new QVBoxLayout(this);
//	vVideoWidgetLayout->addStretch();
	vVideoWidgetLayout->addWidget(m_videoWidget);
	vVideoWidgetLayout->addWidget(m_videoControlWidget);
	vVideoWidgetLayout->setMargin(0);
	vVideoWidgetLayout->setSpacing(0);

// 	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
// 	hMainLayout->addWidget(m_videoWidget);
// 	hMainLayout->setMargin(0);
// 	hMainLayout->setSpacing(0);
}

void VideoPlayer::initAudioPlayerControl()
{
    m_waveformWidget = new WaveformWidget;

    m_pButtonVideoPlay = new QPushButton(this);
    m_pButtonVideoPlay->setFixedSize(QSize(32, 32));
    m_pButtonVideoPlay->setCheckable(true);
    m_pButtonVideoPlay->setChecked(true);
    m_pButtonVideoPlay->setStyleSheet("QPushButton{border-image:url(:/Resources/VideoPlayer/videoStop.png);}\
										QPushButton:hover{border-image:url(:/Resources/VideoPlayer/videoStop_hover.png);}\
										QPushButton:pressed{margin:3px;}\
										QPushButton::checked{border-image:url(:/Resources/VideoPlayer/videoPlay.png);}\
										QPushButton::checked:hover{border-image:url(:/Resources/VideoPlayer/videoPlay_hover.png);}\
										QPushButton::checked:pressed{ margin:3px; }");

    m_pButtonVolumeSwitch = new QPushButton(this);
    m_pButtonVolumeSwitch->setFixedSize(QSize(32, 32));
    m_pButtonVolumeSwitch->setCheckable(true);
    m_pButtonVolumeSwitch->setStyleSheet("QPushButton{border-image:url(:/Resources/VideoPlayer/videoSoundOn.png);}\
										QPushButton:hover{border-image:url(:/Resources/VideoPlayer/videoSoundOn_hover.png);}\
										QPushButton:pressed{margin:3px;}\
										QPushButton::checked{border-image:url(:/Resources/VideoPlayer/videoSoundOff.png);}\
										QPushButton::checked:hover{border-image:url(:/Resources/VideoPlayer/videoSoundOff_hover.png);}\
										QPushButton::checked:pressed{ margin:3px; }");

   
    m_videoProcessTimeLabel = new QLabel(this);
    m_videoProcessTimeLabel->setText("0:00 / 0:00");
    m_videoProcessTimeLabel->setStyleSheet("color:#cdcdcd;font-size:13px;");

    m_videoVolumeSlider = new QSlider(Qt::Horizontal, this);
    m_videoVolumeSlider->setFixedWidth(100);
    m_videoVolumeSlider->setRange(0, 100);
    m_videoVolumeSlider->setValue(50);
    m_videoVolumeSlider->setStyleSheet("QSlider::groove:horizontal{border:1px solid white;height:1px;left:6px; right:6px;}\
							QSlider::handle:horizontal{border:0px;border-image:url(:/Resources/VideoPlayer/SliderButton.png);width:20px;margin:-9px -6px -9px -6px;}\
							QSlider::add-page:horizontal{background:white;}QSlider::sub-page:horizontal{background:rgb(33, 150, 243);}");


    m_videoPlayProcessSlider = new QSlider(Qt::Horizontal, this);
    m_videoPlayProcessSlider->setStyleSheet("QSlider::groove:horizontal{border:1px solid white;height:1px;left:6px; right:6px;}\
							QSlider::handle:horizontal{border:0px;border-image:url(:/Resources/VideoPlayer/SliderButton.png);width:20px;margin:-9px -6px -9px -6px;}\
							QSlider::add-page:horizontal{background:white;}QSlider::sub-page:horizontal{background:rgb(33, 150, 243);}");

    connect(m_pButtonVideoPlay, &QPushButton::clicked, this, &VideoPlayer::onVideoPlayButtonClicked);
    connect(m_pButtonVolumeSwitch, &QPushButton::clicked, this, &VideoPlayer::onVideoVoiceSwitchButtonClicked);

    connect(m_videoVolumeSlider, &QSlider::valueChanged, this, &VideoPlayer::onVolumeChanged);
    connect(m_videoPlayProcessSlider, &QSlider::sliderMoved, this, &VideoPlayer::onChangeVideoProcess);

    m_videoControlWidget = new QWidget;
    m_videoControlWidget->setFixedHeight(60);
    m_videoControlWidget->setObjectName("VideoControlWidget");
    m_videoControlWidget->setStyleSheet("QWidget#VideoControlWidget{background:rgba(0, 0, 0, 180);}");

    QHBoxLayout* hVideoControlLayout = new QHBoxLayout();
    hVideoControlLayout->addWidget(m_pButtonVideoPlay);
    hVideoControlLayout->addWidget(m_videoProcessTimeLabel);
    hVideoControlLayout->addStretch();
    hVideoControlLayout->addWidget(m_pButtonVolumeSwitch);
    hVideoControlLayout->addWidget(m_videoVolumeSlider);
    hVideoControlLayout->setSpacing(10);
    hVideoControlLayout->setMargin(0);

    QVBoxLayout* vVideoControlWidgetLayout = new QVBoxLayout(m_videoControlWidget);
    vVideoControlWidgetLayout->addWidget(m_videoPlayProcessSlider);
    vVideoControlWidgetLayout->addLayout(hVideoControlLayout);
    vVideoControlWidgetLayout->setSpacing(0);
    vVideoControlWidgetLayout->setMargin(0);
    vVideoControlWidgetLayout->setContentsMargins(5, 0, 5, 5);

    QVBoxLayout* vVideoWidgetLayout = new QVBoxLayout(this);
    vVideoWidgetLayout->addWidget(m_waveformWidget);
    vVideoWidgetLayout->addWidget(m_videoControlWidget);
    vVideoWidgetLayout->setMargin(0);
    vVideoWidgetLayout->setSpacing(0);
}

void VideoPlayer::setVideoPlayPath(QString filePath)
{
    if (m_playType == AudioPlay && m_waveformWidget != NULL)
    {
        m_waveformWidget->resetWaveform();
    }
	stopVideoPlay();
	if (QFile::exists(filePath))
	{
		m_strVideoFilePath = filePath;
		if (m_mediaPlayer != NULL)
		{
            qDebug() << "setMedia start";
            QTimer::singleShot(600, this, [=] {
                m_mediaPlayer->setMedia(QUrl::fromLocalFile(m_strVideoFilePath));
                qDebug() << "setMedia end";
            });
		}
	}
}

void VideoPlayer::stopVideoPlay()
{
	if (m_mediaPlayer != NULL)
	{
		m_mediaPlayer->stop();
		m_strVideoFilePath = "";
		m_pButtonVideoPlay->setChecked(true);
		m_videoProcessTimeLabel->setText("0:00 / 0:00");
	}
}

bool VideoPlayer::eventFilter(QObject *watched, QEvent *event)
{
	if (event->type() == QEvent::MouseButtonDblClick)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		// 鼠标左键双击;
		if (watched == m_videoWidget && mouseEvent->button() == Qt::LeftButton)
		{
			// 双击屏幕,全屏切换;
			if (watched == m_videoWidget)
			{
				m_videoWidget->setFullScreen(!m_videoWidget->isFullScreen());
				m_pButtonVideoFullScreen->setChecked(!m_pButtonVideoFullScreen->isChecked());
			}
		}
	}
	/*
	if (event->type() == QEvent::MouseMove)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		QRect bottomRect = QRect(0, this->height() - 60, this->width(), 60);
		if (bottomRect.contains(mouseEvent->pos()))
		{
		//	m_videoControlWidget->setVisible(true);
		}
		else
		{
		//	m_videoControlWidget->setVisible(false);
		}
	}

	if (event->type() == QEvent::Leave)
	{
	//	m_videoControlWidget->setVisible(false);
	}
	*/

	return __super::eventFilter(watched, event);
}

void VideoPlayer::onVideoTotalTimeChanged(qint64 videoTotalTime)
{
	m_videoTotalTime = videoTotalTime / 1000;
	m_videoPlayProcessSlider->setMaximum(m_videoTotalTime);
	updateVideoTimeInfo(0);
}

void VideoPlayer::onVideoProcessChanged(qint64 currentProcessTime)
{
	if (!m_videoPlayProcessSlider->isSliderDown())
		m_videoPlayProcessSlider->setValue(currentProcessTime / 1000);

	updateVideoTimeInfo(currentProcessTime / 1000);
}

void VideoPlayer::updateVideoTimeInfo(qint64 currentTimeInfo)
{
	QString tStr;
	if (currentTimeInfo || m_videoTotalTime) {
		QTime currentTime((currentTimeInfo / 3600) % 60, (currentTimeInfo / 60) % 60,
			currentTimeInfo % 60, (currentTimeInfo * 1000) % 1000);
		QTime totalTime((m_videoTotalTime / 3600) % 60, (m_videoTotalTime / 60) % 60,
			m_videoTotalTime % 60, (m_videoTotalTime * 1000) % 1000);
		QString format = "mm:ss";
		if (m_videoTotalTime > 3600)
			format = "hh:mm:ss";
		tStr = currentTime.toString(format) + " / " + totalTime.toString(format);
		m_videoProcessTimeLabel->setText(tStr);
	}
}

void VideoPlayer::onVideoPlayButtonClicked()
{
	if (!QFile::exists(m_strVideoFilePath))
	{
		m_pButtonVideoPlay->setChecked(true);
		return;
	}

	if (m_mediaPlayer == NULL)
	{
		this->setFixedSize(this->size());
		initMediaPlayer();
		m_mediaPlayer->setMedia(QUrl::fromLocalFile(m_strVideoFilePath));
		m_pButtonVolumeSwitch->setChecked(m_mediaPlayer->isMuted());
	}

	int mediaState = m_mediaPlayer->state();
	switch (m_mediaPlayer->state()) {
	case QMediaPlayer::PlayingState:
	{
		m_mediaPlayer->pause();
		m_pButtonVideoPlay->setChecked(true);
	}
		break;
	default:
	{
		m_mediaPlayer->play();
		m_pButtonVideoPlay->setChecked(false);
	}
		break;
	}
}

void VideoPlayer::onVideoVoiceSwitchButtonClicked()
{
	if (m_mediaPlayer != NULL)
	{
		m_mediaPlayer->setMuted(!m_mediaPlayer->isMuted());
	}
}

void VideoPlayer::onFullScreenButtonClicked()
{
	m_videoWidget->setFullScreen(!m_videoWidget->isFullScreen());
}

void VideoPlayer::onVolumeChanged()
{
	if (m_mediaPlayer != NULL)
	{
		qreal linearVolume = QAudio::convertVolume(m_videoVolumeSlider->value() / qreal(100),
			QAudio::LogarithmicVolumeScale,
			QAudio::LinearVolumeScale);

		m_mediaPlayer->setMuted(false);
		m_mediaPlayer->setVolume(qRound(linearVolume * 100));
	}
}

void VideoPlayer::onChangeVideoProcess(int seconds)
{
	if (m_mediaPlayer != NULL)
	{
		m_mediaPlayer->setPosition(seconds * 1000);
	}
}

void VideoPlayer::onVideoStatusChanged(QMediaPlayer::State state)
{
	switch (state)
	{
	case QMediaPlayer::StoppedState:
		m_pButtonVideoPlay->setChecked(true);
        if (m_playType == AudioPlay)
        {
            m_audioRefreshTimer.stop();
        }
		break;
	case QMediaPlayer::PlayingState:
        if (m_playType == AudioPlay)
        {
            m_audioRefreshTimer.start();
        }
		break;
	case QMediaPlayer::PausedState:
        if (m_playType == AudioPlay)
        {
            m_audioRefreshTimer.stop();
        }
		break;
	default:
		break;
	}
}

void VideoPlayer::processBuffer(QAudioBuffer buffer)
{
    QVector<qreal> levels = getBufferLevels(buffer);
    m_waveformWidget->addWaveformData(levels);
}

void VideoPlayer::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);
 	QPainter painter(this);
    if (m_videoWidget != NULL)
    {
        painter.fillRect(m_videoWidget->rect(), Qt::black);
    }
	return __super::paintEvent(event);
}

void VideoPlayer::showVideoWidget()
{
	this->setUpdatesEnabled(false);
	m_videoWidget->setFullScreen(true);
	m_videoWidget->setFullScreen(false);
	this->setUpdatesEnabled(true);
}