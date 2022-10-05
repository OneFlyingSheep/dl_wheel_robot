#include "DLBackStageMapWidget.h"
#include <QtWidgets>
#include <fstream>
#include <iostream>
#include <QTime>
#include "LibDlToolItems/DLPlatformItem.h"
#include "LibDlToolItems/DLBezierItem.h"
#include "LibDlToolItems/DLLandmarkItem.h"
#include "LibDlToolItems/DLCoordinateItem.h"
#include "LibDlToolItems/DLPathItem.h"
#include "LibDlToolItems/DLDeviceAreaItem.h"
#include "LibDlToolItems/DLRobotItem.h"
#include "LibDlToolItems/DLPathItem.h"
#include "LibDlToolItems/DLLaserItem.h"
#include "LibMapReader/MapReader.h"
#include "LibMapReaderInfoWidget/MapReaderInfoWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "LibDLDeviceViewWidget/LibDLDeviceViewWidget.h"
#include "common/DLRobotCommonDef.h"
#include "common/DLWheelRobotGlobalDef.hpp"

#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

#include "LibDLSceneView/DLCustomScene.h"


#define CATCH_FACTOR (5)
#define PI (3.14159)
#define VIEW_CENTER (viewport()->rect().center())
#define VIEW_WIDTH  (viewport()->rect().width())
#define VIEW_HEIGHT (viewport()->rect().height())



DLBackStageMapView::DLBackStageMapView(QWidget *parent)
	: QGraphicsView(parent),
	translate_button_(Qt::MidButton),
	scale_(1.0),
	zoom_delta_(0.1),
	translate_speed_(1.0),
	bMouse_translate_(false),
	viewPort_mode_(false)
{

	initView();
	setFocusPolicy(Qt::StrongFocus);
	setCacheMode(CacheBackground);
	setViewportUpdateMode(BoundingRectViewportUpdate);
	setRenderHint(QPainter::Antialiasing);
	setTransformationAnchor(AnchorUnderMouse);
	//setMatrix(QMatrix(1.0, 0.0, 0.0, -1.0, 0.0, 0.0), true);
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	centerOn(0, 0);

	//OpenDefaultMap();
}


void DLBackStageMapView::initView()
{
	follow_pushbutton_ = new QPushButton;
	follow_pushbutton_->setCheckable(true);
	open_pushbutton_ = new QPushButton;
	center_pushbutton_ = new QPushButton;
	center_pushbutton_->setFixedSize(30, 30);
	follow_pushbutton_->setFixedSize(30, 30);
	open_pushbutton_->setFixedSize(30, 30);

	m_pDrawPathBtn = new QPushButton(this);
	connect(m_pDrawPathBtn, SIGNAL(clicked(bool)), this, SLOT(DrawPathBtnSlot(bool)));
	m_pDrawPathBtn->setFixedSize(30, 30);
	m_pDrawPathBtn->setToolTip("绘制机器人行走路径");
	m_pDrawPathBtn->setCheckable(true);

// 	m_pCutBgBtn = new QPushButton(this);
// 	connect(m_pCutBgBtn, SIGNAL(clicked(bool)), this, SLOT(CutBgBtnSlot(bool)));
// 	m_pCutBgBtn->setFixedSize(30, 30);
// 	m_pCutBgBtn->setToolTip("切换背景");
// 	m_pCutBgBtn->setCheckable(true);

	QPixmap pic_follow, pic_open, pic_center, pic_draw, pic_change;
	pic_follow.load(":/Resources/Common/image/manual.png");
	pic_open.load(":/Resources/Common/image/open.png");
	pic_center.load(":/Resources/Common/image/center.png");
	pic_draw.load(":/Resources/Common/image/draw.png");
	pic_change.load(":/Resources/Common/image/change.png");

	pic_follow.scaled(30, 30);
	pic_open.scaled(30, 30);
	pic_center.scaled(30, 30);
	pic_draw.scaled(30, 30);
	pic_change.scaled(30, 30);

	follow_pushbutton_->setIcon(QIcon(pic_follow));
	follow_pushbutton_->setIconSize(QSize(30, 30));
	open_pushbutton_->setIcon(QIcon(pic_open));
	open_pushbutton_->setIconSize(QSize(30, 30));
	center_pushbutton_->setIcon(QIcon(pic_center));
	center_pushbutton_->setIconSize(QSize(30, 30));
	m_pDrawPathBtn->setIcon(QIcon(pic_draw));
	m_pDrawPathBtn->setIconSize(QSize(30, 30));
// 	m_pCutBgBtn->setIcon(QIcon(pic_change));
// 	m_pCutBgBtn->setIconSize(QSize(30, 30));


	QVBoxLayout *main_layout = new QVBoxLayout;
	QHBoxLayout *top_layout = new QHBoxLayout;
	top_layout->addWidget(center_pushbutton_);
	top_layout->addStretch();

	//top_layout->addWidget(m_pCutBgBtn);
	top_layout->addWidget(m_pDrawPathBtn);
	top_layout->addWidget(follow_pushbutton_);
	top_layout->addWidget(open_pushbutton_);


	main_layout->addLayout(top_layout);
	main_layout->addStretch();
	this->setLayout(main_layout);

	connect(follow_pushbutton_, SIGNAL(clicked(bool)), this, SLOT(slot_on_viewPortMode(bool)));
	connect(open_pushbutton_, SIGNAL(clicked()), this, SLOT(slot_on_open_map()));
	connect(center_pushbutton_, SIGNAL(clicked()), this, SLOT(slot_on_center()));
}


void DLBackStageMapView::slot_on_center()
{

	QPointF center_point = ((DLCustomScene*)this->scene())->GetCenter();
	centerOn(center_point);

}


void DLBackStageMapView::slot_on_open_map()
{
// 	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
// 	QString strDir = stMapInfo.rootPath + "/map/";
// 	QString fileName = QFileDialog::getOpenFileName(this, ("Open Map"), strDir, ("Map Files (*.smap)"));
// 	((DLCustomScene*)this->scene())->load_json_map(fileName);
// 	QPointF center_point = ((DLCustomScene*)this->scene())->GetCenter();
// 	centerOn(center_point);
	QString strMapDir = QFileDialog::getExistingDirectory(this, tr("选择地图"), "", QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
	((DLCustomScene*)this->scene())->removeall();
	((DLCustomScene*)this->scene())->load_json_map(strMapDir);
	QPointF center_point = ((DLCustomScene*)this->scene())->GetCenter();
	centerOn(center_point);
}


void DLBackStageMapView::ExitDrawStateSlot()
{
	m_pDrawPathBtn->setEnabled(true);
	m_pDrawPathBtn->setChecked(false);
	this->setCursor(Qt::ArrowCursor);
}

void DLBackStageMapView::DrawPathBtnSlot(bool bIsChecked)
{
	if (bIsChecked)
	{
		QCursor myCursor(QPixmap(":/Resources/Common/image/draw.png"));    //-1,-1表示热点位于图片中心
		this->setCursor(myCursor);
		m_pDrawPathBtn->setEnabled(false);

		DLCustomScene *pCustomScene = dynamic_cast<DLCustomScene *>(scene());
		if (pCustomScene)
		{
			//fitInView(pCustomScene->GetPathItem());
			//fitInView(pCustomScene->GetPathRect());
			QPointF center_point = ((DLCustomScene*)this->scene())->GetCenter();
			centerOn(center_point);
		}
	}
	else
	{
		ExitDrawStateSlot();
	}


	emit DrawStateChangedSignal(bIsChecked);
}

void DLBackStageMapView::CutBgBtnSlot(bool bIsChecked)
{
	if (bIsChecked)
	{
		DLCustomScene *pDLCustomScene = dynamic_cast<DLCustomScene *>(this->scene());
		if (NULL != pDLCustomScene)
		{
			pDLCustomScene->SetIsShowBgPixmap(false);
		}
	}
	else
	{
		DLCustomScene *pDLCustomScene = dynamic_cast<DLCustomScene *>(this->scene());
		if (NULL != pDLCustomScene)
		{
			pDLCustomScene->SetIsShowBgPixmap(true);
		}
	}
}

void DLBackStageMapView::slot_on_flush_viewPort(QPointF point)
{
	if (viewPort_mode_) {
		centerOn(point);
	}
}


void DLBackStageMapView::slot_on_viewPortMode(bool mode)
{
	viewPort_mode_ = mode;
	QPixmap pic_viewPort;
	if (mode) {
		pic_viewPort.load(":/Resources/Common/image/auto.png");
	}
	else {
		pic_viewPort.load(":/Resources/Common/image/manual.png");
	}
	pic_viewPort.scaled(30, 30);
	follow_pushbutton_->setIcon(QIcon(pic_viewPort));
	follow_pushbutton_->setIconSize(QSize(30, 30));
}


void DLBackStageMapView::setTranslateSpeed(double speed)
{
	// 建议速度范围
	Q_ASSERT_X(speed >= 0.0 && speed <= 2.0,
		"InteractiveView::setTranslateSpeed", "Speed should be in range [0.0, 2.0].");
	translate_speed_ = speed;
}


double DLBackStageMapView::translateSpeed()
{
	return translate_speed_;
}


void DLBackStageMapView::setZoomDelta(double delta)
{
	// 建议增量范围
	Q_ASSERT_X(delta >= 0.0 && delta <= 1.0,
		"InteractiveView::setZoomDelta", "Delta should be in range [0.0, 1.0].");
	zoom_delta_ = delta;
}


double DLBackStageMapView::zoomDelta()
{
	return zoom_delta_;
}


void DLBackStageMapView::keyPressEvent(QKeyEvent *event)
{
	switch (event->key()) {
	case Qt::Key_Plus:  // 放大
		zoomIn();
		break;
	case Qt::Key_Minus:  // 缩小
		zoomOut();
		break;
	default:
		QGraphicsView::keyPressEvent(event);
	}
}


void DLBackStageMapView::mouseMoveEvent(QMouseEvent *event)
{
	if (bMouse_translate_) 
	{
		QPointF mouseDelta = mapToScene(event->pos()) - mapToScene(last_mousePos_);
		translate(mouseDelta);
	}
	
	last_mousePos_ = event->pos();
	return QGraphicsView::mouseMoveEvent(event);
}


void DLBackStageMapView::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == translate_button_) 
	{
		QPointF point = mapToScene(event->pos());
		//if (scene()->itemAt(point, transform()) == NULL) 
		{
			setCursor(Qt::SizeAllCursor);
			bMouse_translate_ = true;
			last_mousePos_ = event->pos();
		}
	}
	QGraphicsView::mousePressEvent(event);
}


void DLBackStageMapView::mouseReleaseEvent(QMouseEvent *event)
{
	if (event->button() == translate_button_) 
	{
		bMouse_translate_ = false;
		setCursor(Qt::ArrowCursor);
	}
	
	QGraphicsView::mouseReleaseEvent(event);
}


void DLBackStageMapView::wheelEvent(QWheelEvent *event)
{
	// 滚轮的滚动量
	QPoint scrollAmount = event->angleDelta();
	// 正值表示滚轮远离使用者（放大），负值表示朝向使用者（缩小）
	scrollAmount.y() > 0 ? zoomIn() : zoomOut();
}


void DLBackStageMapView::OpenDefaultMap()
{
	WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();

	QString fileName = stMapInfo.rootPath + "/map/"+ stMapInfo.strDefaultMapName;
	//((DLCustomScene*)this->scene())->load_backstage_map(fileName);
	((DLCustomScene*)this->scene())->load_json_map(fileName);
	QPointF center_point = ((DLCustomScene*)this->scene())->GetCenter();
	centerOn(center_point);
}

void DLBackStageMapView::zoomIn()
{

	zoom(1 + zoom_delta_);

}


void DLBackStageMapView::zoomOut()
{
	zoom(1 - zoom_delta_);
}


void DLBackStageMapView::zoom(float scaleFactor)
{
	// 防止过小或过大
	qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
	if (factor < 0.07 || factor > 100)
		return;

	scale(scaleFactor, scaleFactor);
	scale_ *= scaleFactor;
}


void DLBackStageMapView::translate(QPointF delta)
{
	// 根据当前 zoom 缩放平移数
	delta *= scale_;
	delta *= translate_speed_;

	// view 根据鼠标下的点作为锚点来定位 scene
	setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	QPoint newCenter(VIEW_WIDTH / 2 - delta.x(), VIEW_HEIGHT / 2 - delta.y());
	centerOn(mapToScene(newCenter));

	// scene 在 view 的中心点作为锚点
	setTransformationAnchor(QGraphicsView::AnchorViewCenter);
}
