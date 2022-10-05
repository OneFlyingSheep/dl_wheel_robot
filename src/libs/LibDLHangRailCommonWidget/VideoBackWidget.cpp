#include "VideoBackWidget.h"
#include "LibDLHangRailConfigData/DLHangRailStationCfgData.h"
#include "CameraObject.h"
#include <QVBoxLayout>
#include <QDesktopWidget>
#include <QApplication>
#include <QStyleOption>
#include <QPainter>
#include <QMouseEvent>
#include <QButtonGroup>
#include <QDir>
#include <QStandardItemModel>
#include <QTableView>
#include <LibDLHangRailRobotDBOperation/LibDLHangRailRobotDBOperation.h>
#include <LibDLHangRailCommonWidget/QtDeviceCurve.h>
#include "LibDLHangRailCommonWidget/BaseWidget.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"
#include "LibDLHangRailRobotConfigData/DLHangRailRobotConfigData.h"

#define BORDER_HEIGHT	240

FullScreenVideoWidget::FullScreenVideoWidget(QWidget* parent /*= NULL*/)
	:QVideoWidget(parent)
{
	m_isSetPixmap = false;
	m_bIsCollectAim = false;

	//setFixedSize(100, 100);
}

void FullScreenVideoWidget::setPixmap(QPixmap videoImage)
{
	m_isSetPixmap = true;
	m_videoShowPixmap = videoImage;
	update();
}

void FullScreenVideoWidget::paintEvent(QPaintEvent *event)
{
	QPainter painter(this);
	if (m_isSetPixmap)
	{//绘制视频
		painter.drawPixmap(this->rect(), m_videoShowPixmap.scaled(this->rect().size()));
	}

	if (m_bIsCollectAim)
	{
		QRect rect = this->rect();
		qreal rWidth = rect.width();
		qreal rHeight = rect.height();

		//三分之一处绘制矩形
		qreal rSide = rHeight / (qreal)3;
		if (rWidth / (qreal)3 < rSide)
		{
			rSide = rWidth / (qreal)3;
		}
		painter.save();
		QPen pen;
		pen.setColor(Qt::red);
		pen.setWidth(2);
		painter.setPen(pen);
		QPoint ptCenter = rect.center();
		painter.drawRect((ptCenter.x() - qRound(rSide / 2)), (ptCenter.y() - qRound(rSide / 2)), rSide, rSide);
		painter.save();

		//六分之一绘制矩形
		qreal rFSide = rHeight / (qreal)6;
		if (rWidth / (qreal)6 < rFSide)
		{
			rFSide = rWidth / (qreal)6;
		}
		pen.setColor(Qt::blue);
		pen.setWidth(2);
		painter.setPen(pen);
		painter.drawRect((ptCenter.x() - qRound(rFSide / 2)), (ptCenter.y() - qRound(rFSide / 2)), rFSide, rFSide);
		painter.restore();

		//绘制虚线
		pen = painter.pen();
		pen.setStyle(Qt::DotLine);
		painter.setPen(pen);
		painter.drawLine(ptCenter.x() - qRound(rSide / 2), ptCenter.y(), ptCenter.x() + qRound(rSide / 2), ptCenter.y());
		painter.drawLine(ptCenter.x(), ptCenter.y() - qRound(rSide / 2), ptCenter.x(), ptCenter.y() + qRound(rSide / 2));
		painter.restore();
	}
// 	QRect rect = this->rect();
// 	rect.setWidth(rect.width() - 2);
// 	rect.setHeight(rect.height() - 2);
// 	painter.drawRect(rect);
}

VideoBackWidget::VideoBackWidget(VideoType videoType, bool isNeedSelect, QWidget *parent)
	: QWidget(parent)
	, m_videoType(videoType)
	, m_cameraObject(NULL)
	, m_isNeedSelect(isNeedSelect)
	, m_isDrawSelectRect(false)
//	, m_infraredTemperature(NULL)
	, m_videoBackWidget(NULL)
	, m_isStartRecordAudio(false)
	, m_pButtonGoBack(NULL)
{

	switch (m_videoType)
	{
	case CommonVideo:
		initCommonVideo();
		break;
	case AssistVideo:
		initAssistVideo();
		break;
	case VisibleLightControlVideo:
    case Wheel_Collect_VisibleLightControlVideo:
		initVisibleLightButton();
		break;
	case InfraredControlVideo:
    case Wheel_Collect_InfraredControlVideo:
		initInFraredButton();
		break;
    case Wheel_VisibleLightControlVideo:
        initWheelVisibleLightButton();
        break;
    case Wheel_InfraredControlVideo:
        initWheelInFraredButton();
        break;
	default:
		break;
	}

	if (m_videoType == VisibleLightControlVideo || m_videoType == Wheel_Collect_VisibleLightControlVideo || m_videoType == Wheel_Collect_InfraredControlVideo)
	{
		m_singleButtonClickTimer.setSingleShot(true);
		connect(&m_singleButtonClickTimer, &QTimer::timeout, this, [=] {
			emit signalSendMousePressPoint(m_mousePressPoint, m_videoBackWidget->size(), m_cameraObject->getCurrentZoomValue());
		});
	}
}

VideoBackWidget::~VideoBackWidget()
{

}

void VideoBackWidget::setVideoSize1080P()
{
	QDesktopWidget* desktopWidget = QApplication::desktop();
	QRect screenRect = desktopWidget->screenGeometry();
	// 长高 16:9;
	int height = (screenRect.height() - BORDER_HEIGHT) / 2;
	int width = height * 16 / 9;

	this->setFixedSize(QSize(width, height));
}

void VideoBackWidget::setVideoSizeScale4_3()
{
	QDesktopWidget* desktopWidget = QApplication::desktop();
	QRect screenRect = desktopWidget->screenGeometry();
	// 长高 4:3;
	int height = (screenRect.height() - BORDER_HEIGHT) / 2;
	int width = height * 4 / 3;

	this->setFixedSize(QSize(width, height));
}

void VideoBackWidget::initCommonVideo()
{
	m_videoBackWidget = new FullScreenVideoWidget;
	m_videoBackWidget->installEventFilter(this);

	QHBoxLayout* hMainBackLayout = new QHBoxLayout(this);
	hMainBackLayout->addWidget(m_videoBackWidget);

	if (m_isNeedSelect)
	{
		hMainBackLayout->setMargin(5);
	}
	else
	{
		hMainBackLayout->setMargin(0);
	}
}

void VideoBackWidget::initAssistVideo()
{
	for (int i = 0; i < 4; i++)
	{
		QWidget* widget = new QWidget;
		widget->setObjectName("ChildVideoWidget");
		m_videoWidgetList.append(widget);
	}

	QGridLayout* videoGridLayout = new QGridLayout(this);
	videoGridLayout->addWidget(m_videoWidgetList[0], 0, 0);
	videoGridLayout->addWidget(m_videoWidgetList[1], 0, 1);
	videoGridLayout->addWidget(m_videoWidgetList[2], 1, 0);
	videoGridLayout->addWidget(m_videoWidgetList[3], 1, 1);
	videoGridLayout->setSpacing(10);
	videoGridLayout->setMargin(10);

	this->setStyleSheet("QWidget#ChildVideoWidget{background:rgb(255,245,157);}");
}

void VideoBackWidget::initVisibleLightButton()
{
	m_videoBackWidget = new FullScreenVideoWidget(this);	
	m_videoBackWidget->setObjectName("fullScreenVideo");
	m_videoBackWidget->SetCollectAim(true);
	m_videoBackWidget->installEventFilter(this);


	QButtonGroup* buttonGroup = new QButtonGroup(this);

	m_buttonBackWidget = new QWidget(m_videoBackWidget);
	m_buttonBackWidget->setFixedHeight(60);
	m_buttonBackWidget->setObjectName("ButtonBackWidget");
	//m_buttonBackWidget->setStyleSheet("QWidget#ButtonBackWidget{background:rgba(0, 0, 0, 0);}");

	QHBoxLayout* vVisibleLightVideoLayout = new QHBoxLayout(m_buttonBackWidget);
	vVisibleLightVideoLayout->addStretch();
	for (int i = 0; i < 7; i++)
	{
		QPushButton* pButton = new QPushButton(m_buttonBackWidget);
		pButton->setFixedSize(QSize(40, 40));
		pButton->setIconSize(QSize(40, 40));
		//pButton->setStyleSheet("QPushButton{border:none;background:transparent;}QPushButton:pressed{padding-left:3px;padding-top:3px;}");
		pButton->setVisible(false);
		vVisibleLightVideoLayout->addWidget(pButton);
		m_VideoButtonList.append(pButton);
		buttonGroup->addButton(pButton, i);
	}
	m_VideoButtonList[0]->setIcon(QIcon(":/Resources/focusStrong.png"));
	m_VideoButtonList[1]->setIcon(QIcon(":/Resources/focusWeak.png"));
	m_VideoButtonList[2]->setIcon(QIcon(":/Resources/zoomOut.png"));
	m_VideoButtonList[3]->setIcon(QIcon(":/Resources/zoomIn.png"));
	m_VideoButtonList[4]->setIcon(QIcon(":/Resources/Capture.png"));
	m_VideoButtonList[5]->setIcon(QIcon(":/Resources/StartRecordAudio.png"));
	m_VideoButtonList[5]->setCheckable(true);
	m_VideoButtonList[5]->setToolTip("开始录音");
    m_VideoButtonList[6]->setIcon(QIcon(":/Resources/Restart.png"));
    m_VideoButtonList[6]->setToolTip("重启");

    if (Wheel_Collect_VisibleLightControlVideo != m_videoType)
    {
        m_VideoButtonList[6]->setVisible(false);
    }

	// 云台复位按钮;
	m_pButtonGoBack = new QPushButton(m_buttonBackWidget);
	m_pButtonGoBack->setFixedSize(QSize(40, 40));
	m_pButtonGoBack->setIconSize(QSize(40, 40));
	//m_pButtonGoBack->setStyleSheet("QPushButton{border:none;background:transparent;}QPushButton:pressed{padding-left:3px;padding-top:3px;}");
	m_pButtonGoBack->setVisible(false);
	m_pButtonGoBack->setEnabled(false);
	m_pButtonGoBack->setIcon(QIcon(":/Resources/Common/image/PtzReset.png"));
	vVisibleLightVideoLayout->addWidget(m_pButtonGoBack);
	m_VideoButtonList.append(m_pButtonGoBack);
	connect(m_pButtonGoBack, &QPushButton::clicked, this, &VideoBackWidget::signalPtzResetButtonClicked);

	//十字对焦的矩形框
	m_pBtnVisibleTenRect = new QPushButton(m_buttonBackWidget);
	m_pBtnVisibleTenRect->setToolTip("十字对焦的矩形框");
	m_pBtnVisibleTenRect->setCheckable(true);
	m_pBtnVisibleTenRect->setFixedSize(QSize(40, 40));
	m_pBtnVisibleTenRect->setIconSize(QSize(40, 40));
	//m_pBtnVisibleTenRect->setStyleSheet("");
	m_pBtnVisibleTenRect->setVisible(false);
	m_pBtnVisibleTenRect->setIcon(QIcon(":/Resources/Common/image/ten_rect.png"));
	vVisibleLightVideoLayout->addWidget(m_pBtnVisibleTenRect);
	m_VideoButtonList.append(m_pBtnVisibleTenRect);

	connect(m_pBtnVisibleTenRect, SIGNAL(clicked()), this, SLOT(VisibleTenRectSlot()));


	connect(buttonGroup, SIGNAL(buttonPressed(int)), this, SLOT(onButtonPressed(int)));
	connect(buttonGroup, SIGNAL(buttonReleased(int)), this, SLOT(onButtonReleased(int)));

	vVisibleLightVideoLayout->addStretch();
	vVisibleLightVideoLayout->setMargin(0);
	vVisibleLightVideoLayout->setSpacing(10);

	QVBoxLayout* hVideoBackLayout = new QVBoxLayout(m_videoBackWidget);
	hVideoBackLayout->addStretch();
	hVideoBackLayout->addWidget(m_buttonBackWidget);
	hVideoBackLayout->setMargin(5);

	QVBoxLayout* hMainLayout = new QVBoxLayout(this);
	hMainLayout->addWidget(m_videoBackWidget);
	//hMainLayout->addStretch();
	hMainLayout->setMargin(0);

	//QDesktopWidget* desktopWidget = QApplication::desktop();
	//QRect screenRect = desktopWidget->screenGeometry();
	//int height = (screenRect.height() - BORDER_HEIGHT) / 2;
	//this->setFixedHeight(315);
}

void VideoBackWidget::initInFraredButton()
{
	m_videoBackWidget = new FullScreenVideoWidget;
	m_videoBackWidget->installEventFilter(this);

	QButtonGroup* buttonGroup = new QButtonGroup(this);

	QWidget* buttonWidget = new QWidget;
	QVBoxLayout* vInFraredVideoLayout = new QVBoxLayout();
	vInFraredVideoLayout->addStretch();
	for (int i = 0; i < 5; i++)
	{
		QPushButton* pButton = new QPushButton;
		pButton->setFixedSize(QSize(40, 40));
		pButton->setIconSize(QSize(40, 40));
		pButton->setStyleSheet("QPushButton{border:none;background:transparent;}QPushButton:pressed{padding-left:3px;padding-top:3px;}");
		pButton->setVisible(false);
		vInFraredVideoLayout->addWidget(pButton);
		m_VideoButtonList.append(pButton);
		buttonGroup->addButton(pButton, i);
	}
	vInFraredVideoLayout->addStretch();
	vInFraredVideoLayout->setMargin(8);
	vInFraredVideoLayout->setSpacing(15);

	m_VideoButtonList[0]->setIcon(QIcon(":/Resources/focusStrong.png"));
	m_VideoButtonList[1]->setIcon(QIcon(":/Resources/focusWeak.png"));
	m_VideoButtonList[2]->setIcon(QIcon(":/Resources/autoFocus.png"));
	m_VideoButtonList[3]->setIcon(QIcon(":/Resources/Capture.png"));
    m_VideoButtonList[4]->setIcon(QIcon(":/Resources/Restart.png"));

    if (Wheel_Collect_InfraredControlVideo != m_videoType)
    {
        m_VideoButtonList[4]->setVisible(false);
    }

    connect(buttonGroup, SIGNAL(buttonPressed(int)), this, SLOT(onInfraredButtonPressed(int)));
    connect(buttonGroup, SIGNAL(buttonReleased(int)), this, SLOT(onInfraredButtonReleased(int)));

	QHBoxLayout* hVideoBackLayout = new QHBoxLayout(m_videoBackWidget);
	hVideoBackLayout->addStretch();
	hVideoBackLayout->addLayout(vInFraredVideoLayout);
	hVideoBackLayout->setMargin(0);
	hVideoBackLayout->setContentsMargins(0, 0, 10, 0);

	QHBoxLayout* hMainLayout = new QHBoxLayout(this);
	hMainLayout->addWidget(m_videoBackWidget);
	hMainLayout->setMargin(0);

// 	QDesktopWidget* desktopWidget = QApplication::desktop();
// 	QRect screenRect = desktopWidget->screenGeometry();
// 	int height = (screenRect.height() - BORDER_HEIGHT) / 2;
// 	this->setFixedHeight(height);
}

void VideoBackWidget::initWheelVisibleLightButton()
{
    initVideoPlayWindow(VideoPlay);

    m_videoBackWidget = new FullScreenVideoWidget;
    m_videoBackWidget->installEventFilter(this);

    QButtonGroup* buttonGroup = new QButtonGroup(this);

    m_buttonBackWidget = new QWidget;
    m_buttonBackWidget->setFixedHeight(60);
    m_buttonBackWidget->setObjectName("ButtonBackWidget");
    m_buttonBackWidget->setStyleSheet("QWidget#ButtonBackWidget{background:rgba(0, 0, 0, 0);}");

    QHBoxLayout* hHisibleLightVideoLayout = new QHBoxLayout(m_buttonBackWidget);
    hHisibleLightVideoLayout->addStretch();
    for (int i = 0; i < 4; i++)
    {
        QPushButton* pButton = new QPushButton;
        pButton->setFixedSize(QSize(35, 35));
        pButton->setIconSize(QSize(30, 30));
        pButton->setStyleSheet("QPushButton{border:none;background:transparent;}QPushButton:pressed{padding-left:3px;padding-top:3px;}");
        pButton->setVisible(false);
        hHisibleLightVideoLayout->addWidget(pButton);
        m_VideoButtonList.append(pButton);
        buttonGroup->addButton(pButton, i);
    }
    m_VideoButtonList[0]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Play.png"));
    m_VideoButtonList[1]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Pause.png"));
    m_VideoButtonList[2]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoPlayBack.png"));
    m_VideoButtonList[3]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Restart.png"));
    m_VideoButtonList[3]->setToolTip("重启");

    connect(buttonGroup, SIGNAL(buttonReleased(int)), this, SLOT(onWheelVisibleLightButtonReleased(int)));

    hHisibleLightVideoLayout->addStretch();
    hHisibleLightVideoLayout->setMargin(0);
    hHisibleLightVideoLayout->setSpacing(15);

    QVBoxLayout* vVideoBackLayout = new QVBoxLayout(m_videoBackWidget);
    vVideoBackLayout->addStretch();
    vVideoBackLayout->addWidget(m_buttonBackWidget);
    vVideoBackLayout->setContentsMargins(10, 0, 10, 10);

    QHBoxLayout* hMainLayout = new QHBoxLayout(this);
    hMainLayout->addWidget(m_videoBackWidget);
    hMainLayout->setMargin(0);
}

void VideoBackWidget::initWheelInFraredButton()
{
    initVideoPlayWindow(VideoPlay);

    m_videoBackWidget = new FullScreenVideoWidget;
    m_videoBackWidget->installEventFilter(this);

    QButtonGroup* buttonGroup = new QButtonGroup(this);
    buttonGroup->setExclusive(false);
    QWidget* buttonWidget = new QWidget;
    QHBoxLayout* hInFraredVideoLayout = new QHBoxLayout();
    hInFraredVideoLayout->addStretch();
    for (int i = 0; i < 7; i++)
    {
        QPushButton* pButton = new QPushButton;
        pButton->setFixedSize(QSize(35, 35));
        pButton->setIconSize(QSize(30, 30));
        pButton->setStyleSheet("QPushButton{border:none;background:transparent;}QPushButton:pressed{padding-left:3px;padding-top:3px;}");
        pButton->setVisible(false);
        hInFraredVideoLayout->addWidget(pButton);
        m_VideoButtonList.append(pButton);
        buttonGroup->addButton(pButton, i);
    }
    hInFraredVideoLayout->addStretch();
    hInFraredVideoLayout->setMargin(8);
    hInFraredVideoLayout->setSpacing(15);

    m_VideoButtonList[0]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Play.png"));
    m_VideoButtonList[1]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Pause.png"));
    m_VideoButtonList[2]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Capture.png"));
    m_VideoButtonList[3]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoRecord.png"));
    m_VideoButtonList[3]->setCheckable(true);
    m_VideoButtonList[4]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoPlayBack.png"));
    m_VideoButtonList[5]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Focus.png"));
    m_VideoButtonList[6]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/Restart.png"));
    m_VideoButtonList[6]->setToolTip("重启");

    connect(buttonGroup, SIGNAL(buttonReleased(int)), this, SLOT(onWheelInfraredButtonPressed(int)));

    // 默认不能使用，点击暂停之后使用;
    m_VideoButtonList[0]->setEnabled(false);

    QVBoxLayout* vVideoBackLayout = new QVBoxLayout(m_videoBackWidget);
    vVideoBackLayout->addStretch();
    vVideoBackLayout->addLayout(hInFraredVideoLayout);
    vVideoBackLayout->setMargin(0);
    vVideoBackLayout->setContentsMargins(10, 0, 10, 10);

    QHBoxLayout* hMainLayout = new QHBoxLayout(this);
    hMainLayout->addWidget(m_videoBackWidget);
    hMainLayout->setMargin(0);
}

void VideoBackWidget::initVideoPlayWindow(PlayType playType)
{
    // 视频播放窗口;
    m_videoBackWindow = new BaseWidget(NULL, BaseWidgetType::PopupWindow);
    m_videoBackWindow->setShowCloseButton();
    m_videoBackWindow->setFixedSize(QSize(600, 400));
    m_videoBackWindow->setWindowFlags(Qt::FramelessWindowHint);
    m_videoBackWindow->setTitleContent("视频回放");
    connect(m_videoBackWindow, &BaseWidget::signalCloseButtonClicked, this, [=] {
        m_videoPlayWidget->stopVideoPlay();
        m_videoBackWindow->hide();
    });
    m_videoPlayWidget = new VideoPlayer(NULL, playType);
    QHBoxLayout* hVideoLayout = new QHBoxLayout(m_videoBackWindow->getCenterWidget());
    hVideoLayout->addWidget(m_videoPlayWidget);
    hVideoLayout->setMargin(0);
}

void VideoBackWidget::paintEvent(QPaintEvent *event)
{
	Q_UNUSED(event);
	QStyleOption opt;
	opt.init(this);
	QPainter painter(this);
	style()->drawPrimitive(QStyle::PE_Widget, &opt, &painter, this);

	if (m_isNeedSelect && m_isDrawSelectRect)
	{
		QPen paintPen;
		paintPen.setWidth(10);
		paintPen.setColor(Qt::yellow);
		painter.setPen(paintPen);
		painter.drawRect(this->rect());
	}
}

void VideoBackWidget::setCameraObject(CameraObject* cameraObject)
{
	m_cameraObject = cameraObject;
	connect(m_cameraObject, SIGNAL(signalGetCameraImage(QPixmap)), this, SLOT(onReceiveInfraredVideoImage(QPixmap)));
}

// void VideoBackWidget::setInfraredObject(InfraredTemperature* infraredTemperature)
// {
// 	m_infraredTemperature = infraredTemperature;
// }

void VideoBackWidget::setPtzResetButtonVisible(bool isVisible)
{
	if (m_pButtonGoBack != NULL)
	{
		m_pButtonGoBack->setEnabled(isVisible);
	}	
}

void VideoBackWidget::cameraCapture(QString fileName)
{
	if (m_cameraObject != NULL)
	{
		m_cameraObject->signalVisibleCaptureClicked(fileName);
	}
}

bool VideoBackWidget::cameraCaptureBool(QString fileName)
{
    if (m_cameraObject != NULL)
    {
        return m_cameraObject->cameraCaptureBool(fileName);
    }
    return false;
}

void VideoBackWidget::infraredCapture(QString filePath, QString fileName)
{
// 	if (m_infraredTemperature != NULL)
// 	{
// 		m_infraredTemperature->getVideoImage(filePath + fileName);
// 	}
}

void VideoBackWidget::wheelRobotAudioRecord(bool isStart, QString fileName)
{
    if (isStart)
    {
        m_cameraObject->signalVisibleRecordAudio(true, fileName);
    }
    else
    {
        m_cameraObject->signalVisibleRecordAudio(false);
    }
}

HCNetCameraInterface* VideoBackWidget::getHCNetCameraInterface()
{
    if (m_cameraObject != NULL)
    {
        return m_cameraObject->getHCNetCameraInterface();
    }

    return NULL;
}

void VideoBackWidget::setVideoZoomAbs(unsigned long zoom)
{
	if (m_videoType == Wheel_Collect_VisibleLightControlVideo && m_cameraObject != NULL)
	{
		m_cameraObject->OnVisibleZoomAbs(zoom);
	}
}

void VideoBackWidget::setVideoFocusAbs(unsigned long focus)
{
	if (m_videoType == Wheel_Collect_VisibleLightControlVideo && m_cameraObject != NULL)
	{
		m_cameraObject->OnVisibleFocusAbs(focus);
	}
}

void VideoBackWidget::setVideoZoomAndFocus(unsigned long zoom, unsigned long focus)
{
    if (m_videoType == Wheel_Collect_VisibleLightControlVideo && m_cameraObject != NULL)
    {
        m_cameraObject->OnVisibleZoomAndFocus(zoom, focus);
    }
}

void VideoBackWidget::setInfraredVideoFocusAbs(unsigned long focus)
{
// 	if (m_infraredTemperature != NULL)
// 	{
// 		//m_infraredTemperature->setFocusLocation(focus);
// 	}	
}

void VideoBackWidget::onButtonPressed(int buttonId)
{
	if (m_cameraObject == NULL)
	{
		return;
	}

	VideoButtonType buttonType = VideoButtonType(buttonId);
	switch (buttonType)
	{
	case FocusStrong:
		m_cameraObject->signalVisibleFocusFarClickedSlot(true);
		break;
	case focusWeak:
		m_cameraObject->signalVisibleFocusNearClickedSlot(true);
		break;
	case ZoomOut:
		m_cameraObject->signalVisibleZoomInClickedSlot(true);
		break;
	case ZoomIn:
		m_cameraObject->signalVisibleZoomOutClickedSlot(true);
		break;
    case CameraRestart:
    {
        BlockMessageBox* messsageBox = new BlockMessageBox;
        messsageBox->setBlockTime(8);
        messsageBox->setAttribute(Qt::WA_DeleteOnClose);
        messsageBox->show();

        QTimer::singleShot(500, this, [=] {
            m_cameraObject->signalRestartCamera();
            messsageBox->close();
        });
    }
        break;
	case CameraRestoration:
	{
		// 复位;
		setVideoZoomAbs(ROBOTSTATIONCFG.getRestorationValueStruct().rest_camera_zoom);
		setVideoFocusAbs(ROBOTSTATIONCFG.getRestorationValueStruct().rest_camera_focal);
	}
	default:
		break;
	}
}

void VideoBackWidget::onButtonReleased(int buttonId)
{
	if (m_cameraObject == NULL)
	{
		return;
	}

	VideoButtonType buttonType = VideoButtonType(buttonId);
	switch (buttonType)
	{
	case FocusStrong:
		m_cameraObject->signalVisibleFocusFarClickedSlot(false);
		break;
	case focusWeak:
		m_cameraObject->signalVisibleFocusNearClickedSlot(false);
		break;
	case ZoomOut:
		m_cameraObject->signalVisibleZoomInClickedSlot(false);
		break;
	case ZoomIn:
		m_cameraObject->signalVisibleZoomOutClickedSlot(false);
		break;
	case Capture:
	{
        QString fileName;
        if (m_videoType == VisibleLightControlVideo)
        {
            fileName = getPatrolRootPath("可见光");
        }
        else if (m_videoType == Wheel_Collect_VisibleLightControlVideo)
        {
            fileName = getWheelRobotCaptureFileName();
        }
		
		//m_cameraObject->signalVisibleCaptureClicked(fileName);
        if (!m_cameraObject->cameraCaptureBool(fileName))
        {
            DLMessageBox* messageBox = new DLMessageBox();
            messageBox->setFixedWidth(280);
            messageBox->setMessageContent("拍照失败！");
            messageBox->setWindowModality(Qt::ApplicationModal);
            messageBox->show();
        }
	}
		break;
	case RecordAudio:
	{
		if (!m_isStartRecordAudio)
		{
			m_VideoButtonList[5]->setIcon(QIcon(":/Resources/StopRecordAudio.png"));
			m_VideoButtonList[5]->setToolTip("结束录音");
			m_VideoButtonList[5]->setChecked(false);

            QString fileName;
            if (m_videoType == VisibleLightControlVideo)
            {
                fileName = getPatrolRootPath("音频");
            }
            else if (m_videoType == Wheel_Collect_VisibleLightControlVideo)
            {
                fileName = getWheelRobotAudioFileName();
            }

            m_cameraObject->signalVisibleRecordAudio(true, fileName);
		}
		else
		{
			m_VideoButtonList[5]->setIcon(QIcon(":/Resources/StartRecordAudio.png"));
			m_VideoButtonList[5]->setToolTip("开始录音");
			m_cameraObject->signalVisibleRecordAudio(false);
		}
		m_isStartRecordAudio = !m_isStartRecordAudio;
	}
		break;
	default:
		break;
	}
}

void VideoBackWidget::onWheelVisibleLightButtonReleased(int buttonId)
{
    if (m_cameraObject == NULL)
    {
        return;
    }

    switch (buttonId)
    {
    // 视频播放;
    case 0:
    {
        m_cameraObject->setIsPausePlay(false);
    }
        break;
    // 视频暂停;
    case 1:
    {
        m_cameraObject->setIsPausePlay(true);
    }
        break;
    // 视频回放;
    case 2:
    {
        QString fileName = QFileDialog::getOpenFileName(this, ("Open File"), "", ("Video Files ()"));
        if (fileName.isEmpty())
        {
            return;
        }
        m_videoPlayWidget->setVideoPlayPath(fileName);
        m_videoBackWindow->activateWindow();
        m_videoBackWindow->show();
    }
        break;
    // 重启;
    case 3:
    {
        BlockMessageBox* messsageBox = new BlockMessageBox;
        messsageBox->setBlockTime(8);
        messsageBox->setAttribute(Qt::WA_DeleteOnClose);
        messsageBox->show();

        QTimer::singleShot(500, this, [=] {
            m_cameraObject->signalRestartCamera();
            messsageBox->close();
        });
        
    }
        break;
    default:
        break;
    }
}

void VideoBackWidget::onWheelInfraredButtonPressed(int buttonId)
{
//     if (m_infraredTemperature == NULL)
//     {
//         return;
//     }

    switch (buttonId)
    {
    // 视频播放;
    case 0:
    {
        m_VideoButtonList[1]->setEnabled(true);
        m_VideoButtonList[1]->setVisible(true);

        m_VideoButtonList[0]->setEnabled(false);
        m_VideoButtonList[0]->setVisible(false);

        m_cameraObject->setIsPausePlay(false);
    }
    break;
    // 视频暂停;
    case 1:
    {
        m_VideoButtonList[0]->setEnabled(true);
        m_VideoButtonList[0]->setVisible(true);

        m_VideoButtonList[1]->setEnabled(false);
        m_VideoButtonList[1]->setVisible(false);

        m_cameraObject->setIsPausePlay(true);
    }
    break;
    // 抓拍;
    case 2:
    {
		QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
		//WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_take_photo_req("infraredCapture", QString("%1.jpg").arg(strTime));
        WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_take_photo_req("infraredCapture", strTime, 0);
//         QString fileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/InfraredCapture/";
//         QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
//         QDir dir;
//         if (!dir.exists(fileName))
//         {
//             dir.mkpath(fileName);
//         }
//		m_infraredTemperature->emitGetImage(fileName + strTime);
//		m_infraredTemperature->getVideoImage(fileName + strTime);
// 		bool isSuccess = m_infraredTemperature->getVideoImage(fileName + strTime);
//         if (isSuccess)
//         {
//             DLMessageBox::showDLMessageBox(NULL, "提示", "抓图成功", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
//         }
//         else
//         {
//             DLMessageBox::showDLMessageBox(NULL, "提示", "抓图失败", MessageButtonType::BUTTON_OK, true, QSize(250, 180));
//         }        
    }
    break;
    // 录像;
    case 3:
    {       
        if (m_VideoButtonList[3]->isChecked())
        {
			WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_record_video_req("InfraredVideo", 0);
			m_VideoButtonList[3]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoRecordStop.png"));
//             QString fileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/InfraredVideo/";
//             QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
//             QDir dir;
//             if (!dir.exists(fileName))
//             {
//                 dir.mkpath(fileName);
//             }
// 			if (m_infraredTemperature->getVideoAviStart(fileName + strTime + ".avi"))
// 			{
// 				m_VideoButtonList[3]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoRecordStop.png"));
// 			}
// 			else
// 			{
// 				break;
// 			}
        }
        else
        {
			WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_record_video_req("InfraredVideo", 1);
//             m_infraredTemperature->getVideoAviStop();             
			m_VideoButtonList[3]->setIcon(QIcon(":/Resources/DLWheelRobotManager/Image/VideoRecord.png"));
        }
    }
    break;
    // 回放;
    case 4:
    {
        QString fileName = QFileDialog::getOpenFileName(this, ("Open File"), "", ("Video Files ()"));
        if (fileName.isEmpty())
        {
            return;
        }
        m_videoPlayWidget->setVideoPlayPath(fileName);
        m_videoBackWindow->activateWindow();
        m_videoBackWindow->show();
    }
    break;
    // 红外聚焦;
    case 5:
    {
        ROS_INFO("VideoBackWidget infrared :robot_cloud_infrared_auto_focus_req");
		WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_auto_focus_req(MODE_INFRA_AUTO_FOCUS);
 //       m_infraredTemperature->InfFocusAutomate();
    }
    break;
    // 红外重启;
    case 6:
    {
        BlockMessageBox* messsageBox = new BlockMessageBox;
        messsageBox->setBlockTime(8);
        messsageBox->setAttribute(Qt::WA_DeleteOnClose);
        messsageBox->show();

        QTimer::singleShot(500, this, [=] {
            m_cameraObject->signalRestartCamera();
            messsageBox->close();
        });
    }
    break;
    default:
        break;
    }
}

void VideoBackWidget::VisibleTenRectSlot()
{
	if (m_videoBackWidget)
	{
		m_videoBackWidget->SetCollectAim(!m_videoBackWidget->GetCollectAim());
		m_videoBackWidget->update();
	}
		
}

void VideoBackWidget::onInfraredButtonPressed(int buttonId)
{
    ROS_INFO("VideoBackWidget buttonId :%d", buttonId);
// 	if (m_infraredTemperature == NULL)
// 	{
// 		return;
// 	}
	InfraredButtonType buttonType = InfraredButtonType(buttonId);
    ROS_INFO("VideoBackWidget InfraredButtonType :%d", (int)buttonType);
	switch (buttonType)
	{
	case Infrared_FocusStrong:
		WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_auto_focus_req(MODE_INFRA_FAR_FOCUS);
		break;
	case Infrared_FocusWeak:
		WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_auto_focus_req(MODE_INFRA_NEAR_FOCUS);
		break;
	case Infrared_AutoFocus:
        ROS_INFO("VideoBackWidget infrared :robot_cloud_infrared_auto_focus_req");
		WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_auto_focus_req(MODE_INFRA_AUTO_FOCUS);
	//	m_infraredTemperature->InfFocusAutomate();
		break;
	case InfraredCapture:
	{
		QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
        WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_take_photo_req("infraredCapture", QString("%1").arg(strTime), 0);
//         QString filePath;
//         if (m_videoType == InfraredControlVideo)
//         {
//             filePath = getPatrolRootPath("红外", false);
//         }
//         else if (m_videoType == Wheel_Collect_InfraredControlVideo)
//         {
//             filePath = getWheelRobotInfraredFileName();
//         }
// 
// 		QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh-mm-ss");
// 		m_infraredTemperature->getVideoImage(filePath + strTime);
	}
		break;
    case InfraredRestart:
    {
        BlockMessageBox* messsageBox = new BlockMessageBox;
        messsageBox->setBlockTime(8);
        messsageBox->setAttribute(Qt::WA_DeleteOnClose);
        messsageBox->show();

        QTimer::singleShot(500, this, [=] {
            m_cameraObject->signalRestartCamera();
            messsageBox->close();
        });
    }
        break;
	default:
		break;
	}
}

void VideoBackWidget::onInfraredButtonReleased(int buttonId)
{
    ROS_INFO("VideoBackWidget buttonId :%d", buttonId);
    InfraredButtonType buttonType = InfraredButtonType(buttonId);
    ROS_INFO("VideoBackWidget InfraredButtonType :%d", (int)buttonType);
    switch (buttonType)
    {
    case Infrared_FocusStrong:
        WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_auto_focus_req(MODE_INFRA_STOP_FOCUS);
        break;
    case Infrared_FocusWeak:
        WHEEL_BACK_TO_CORE_SOCKET.robot_cloud_infrared_auto_focus_req(MODE_INFRA_STOP_FOCUS);
        break;

    default:
        break;
    }
}

int VideoBackWidget::getCurrentZoomValue()
{
    if (m_cameraObject != NULL)
    {
        switch (m_videoType)
        {
        case VisibleLightControlVideo:
        case Wheel_Collect_VisibleLightControlVideo:
        case Wheel_VisibleLightControlVideo:
            return m_cameraObject->getCurrentZoomValue();
            break;
        default:
            break;
        }
    }

	return -1;
}

int VideoBackWidget::getCurrentVideoFocusValue()
{
    if (m_cameraObject != NULL)
    {
        switch (m_videoType)
        {
        case VisibleLightControlVideo:
        case Wheel_Collect_VisibleLightControlVideo:
        case Wheel_VisibleLightControlVideo:
            return m_cameraObject->getCurrentVideoFocusValue();
            break;
        default:
            break;
        }
    }

    return -1;
}

bool VideoBackWidget::eventFilter(QObject *watched, QEvent *event)
{
	if (event->type() == QEvent::MouseButtonDblClick)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		if (mouseEvent->button() == Qt::LeftButton && watched == m_videoBackWidget)
		{
			m_singleButtonClickTimer.stop();

            if (app_type_ == APP_TYPE_MAIN_CLIENT)
            {
                m_videoBackWidget->setFullScreen(!m_videoBackWidget->isFullScreen());
            }
            if (app_type_ == APP_TYPE_COLLECT_CLIENT)
            {
                if (m_videoType == VideoType::Wheel_Collect_VisibleLightControlVideo)
                {
                    if (m_videoBackWidget->size().height() > 400)
                    {
                        emit signalTreeText(false, VideoType::Wheel_Collect_VisibleLightControlVideo);
                    //    emit MoveCollectEquipmentPosSignal(true);

                    }
                    else
                    {
                        emit signalTreeText(true, VideoType::Wheel_Collect_VisibleLightControlVideo);
                    //    emit MoveCollectEquipmentPosSignal(false);
                    }
                }
                if (m_videoType == VideoType::Wheel_Collect_InfraredControlVideo)
                {
                    if (m_videoBackWidget->size().height() > 500)
                    {
                        emit signalTreeText(false, VideoType::Wheel_Collect_InfraredControlVideo);
                    }
                    else
                    {
                        emit signalTreeText(true, VideoType::Wheel_Collect_InfraredControlVideo);
                    }
                }
            } 
		}
	}
	else if (event->type() == QEvent::MouseButtonRelease)
	{
		QMouseEvent* mouseEvent = static_cast<QMouseEvent*>(event);
		
    //    if (watched == m_videoBackWidget && mouseEvent->button() == Qt::LeftButton)
    //    if (watched == m_videoBackWidget && mouseEvent->button() == Qt::RightButton && m_bVideoHeightFull)
        if (watched == m_videoBackWidget && mouseEvent->button() == Qt::RightButton)// && m_bVideoHeightFull)
		{
			m_mousePressPoint = mouseEvent->pos();
			m_singleButtonClickTimer.start(200);
		}
	}

	if (event->type() == QEvent::MouseButtonPress)
	{
		if (m_isNeedSelect && watched == m_videoBackWidget)
		{
			m_isDrawSelectRect = true;
			if (!m_videoBackWidget->isFullScreen())
			{
				update();
			}
			emit signalVideoWidgetSelect();
		}
	}
	if (event->type() == QEvent::Enter)
	{
		for (int i = 0; i < m_VideoButtonList.count(); i++)
		{
			if (m_VideoButtonList[i]->isEnabled())
			{
				m_VideoButtonList[i]->setVisible(true);
			}
		}
	}
	else if (event->type() == QEvent::Leave)
	{
		for (int i = 0; i < m_VideoButtonList.count(); i++)
		{
			m_VideoButtonList[i]->setVisible(false);
		}
	}

	if (event->type() == QEvent::KeyPress)
	{
		if (m_videoBackWidget->isFullScreen())
		{
			QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
			if (!keyEvent->isAutoRepeat())
			{
				switch (keyEvent->key())
				{
				case Qt::Key_A:
				case Qt::Key_S:
				case Qt::Key_D:
				case Qt::Key_W:
				case Qt::Key_Q:
				case Qt::Key_E:
				case Qt::Key_Z:
				case Qt::Key_X:
					emit signalOperatePtz(keyEvent->key(), false);
					qDebug() << "VisibleVideo keyPressEvent " << Qt::Key(keyEvent->key());
					break;
				default:
					break;
				}
			}
			return true;
		}		
	}
	else if (event->type() == QEvent::KeyRelease)
	{
		if (m_videoBackWidget->isFullScreen())
		{
			QKeyEvent* keyEvent = static_cast<QKeyEvent*>(event);
			if (!keyEvent->isAutoRepeat())
			{
				switch (keyEvent->key())
				{
				case Qt::Key_A:
				case Qt::Key_S:
				case Qt::Key_D:
				case Qt::Key_W:
				case Qt::Key_Q:
				case Qt::Key_E:
				case Qt::Key_Z:
				case Qt::Key_X:
					emit signalOperatePtz(keyEvent->key(), true);
					qDebug() << "VisibleVideo keyReleaseEvent " << Qt::Key(keyEvent->key());
					break;
				default:
					break;
				}
			}
			return true;
		}		
	}
	else
	{
		return false;
	}

	return __super::eventFilter(watched, event);
}

void VideoBackWidget::setIsDrawSelectRect(bool isDrawSelectRect /* = false */)
{
	m_isDrawSelectRect = isDrawSelectRect;
	update();
}

void VideoBackWidget::onReceiveInfraredVideoImage(QPixmap image)
{
	if (m_videoBackWidget != NULL)
	{
		m_videoBackWidget->setPixmap(image);
	}
}

int VideoBackWidget::getInfraredVideoFocus()
{
// 	if (m_infraredTemperature != NULL)
// 	{
// 		int focus = 0;
// //		m_infraredTemperature->getFocusAbsolute(focus);
// 		return focus;
// 	}

	return -1;
}

QString VideoBackWidget::getPatrolRootPath(QString index, bool isNeedTime)
{
//	stationCfg cameraConfig = ROBOTSTATIONCFG.getStationCfg();
	QString filePath = HANG_RAIL_ROBOT_BACKGROUND_CONFIG.getCfg().strRootPath;
	if (!filePath.endsWith('/'))
	{
		filePath.append('/');
	}
	QString fileName = QString("%1%2/%3/").arg(filePath).arg("手动巡检").arg(index);
	QDir dir;
	if (!dir.exists(fileName))
	{
		dir.mkpath(fileName);
	}
	if (isNeedTime)
	{
		QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh-mm-ss");
		fileName.append(strTime);
	}
	return fileName;
}

QString VideoBackWidget::getWheelRobotCaptureFileName()
{
    QString fileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/VisibleLightCapture/";
    QDir dir;
    if (!dir.exists(fileName))
    {
        dir.mkpath(fileName);
    }
    QString strTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
    fileName.append(strTime);
    return fileName;
}

QString VideoBackWidget::getWheelRobotAudioFileName()
{
    QString strCurrentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd_hh-mm-ss");
    QString strRecordFilePath = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/AudioRecord/";
    QDir dir;
    if (!dir.exists(strRecordFilePath))
    {
        dir.mkpath(strRecordFilePath);
    }

    QString strRecordFileName = strRecordFilePath + strCurrentTime;
    return strRecordFileName;
}

QString VideoBackWidget::getWheelRobotInfraredFileName()
{
    QString fileName = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg().rootPath + "/InfraredCapture/";
    QDir dir;
    if (!dir.exists(fileName))
    {
        dir.mkpath(fileName);
    }

    return fileName;
}

void FullScreenVideoWidget::SetCollectAim(bool bIsCollectAim)
{
	m_bIsCollectAim = bIsCollectAim;
}

bool FullScreenVideoWidget::GetCollectAim()
{
	return m_bIsCollectAim;
}
