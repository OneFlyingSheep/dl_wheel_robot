#include "DLCollectMap.h"
#include <QtWidgets>
#include <fstream>
#include "LibMapReader/MapReader.h"
#include "LibDlToolItems/DLDeviceItem.h"
#include "LibDlToolItems/DLPixmapItem.h"
#include "LibDlToolItems/DLBezierItem.h"
#include "LibDlToolItems/DLLandmarkItem.h"
#include "LibDlToolItems/DLCoordinateItem.h"
#include "LibDlToolItems/DLMultiPointItem.h"
#include "LibDlToolItems/DLSegmentItem.h"
#include "LibDlToolItems/DLPolygonItem.h"
#include "LibDLToolItems/DLAdvancedAreaItem.h"
#include "LibDlToolItems/DLPathItem.h"
#include "LibDlToolItems/DLDeviceAreaItem.h"
#include "LibDlToolItems/DLGridItem.h"
#include "LibDlToolItems/DLRobotItem.h"
#include "LibDlToolItems/DLPathItem.h"
#include "LibDlToolItems/DLLaserItem.h"
#include "LibMapReaderInfoWidget/MapReaderInfoWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
//#include "LibDL3DRobotViewer/DL3DRobotViewer.h"
#include "LibDLWheelCustomWidget/CustomTableWidget.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include <iostream>
#include <QTime>
#include <windows.h> 

#include "LibDLSceneView/DLCustomScene.h"

#define CATCH_FACTOR (5)
#define PI (3.14159)
#define VIEW_CENTER (viewport()->rect().center())
#define VIEW_WIDTH  (viewport()->rect().width())
#define VIEW_HEIGHT (viewport()->rect().height())
#define FLUSH_TIME (33)
#define INSERTE_NUMBER (10)
#define RESOLUTION_NAME_WIDTH (80)


DLPointInfoWidget::DLPointInfoWidget(QWidget *parent)
{
	initWidget();
}


void DLPointInfoWidget::initWidget()
{
	this->setWindowTitle("巡检点位设备列表");
	setFixedSize(600, 800);
	tableWidget_ = new QTableWidget;
	tableWidget_->setColumnCount(4);
	tableWidget_->setHorizontalHeaderLabels(QStringList() << "设备uuid" << "电压等级" << "设备间隔" << "设备名称");
	tableWidget_->setShowGrid(false);
	tableWidget_->horizontalHeader()->setStretchLastSection(true);
	tableWidget_->setFrameShape(QFrame::NoFrame); 
	tableWidget_->verticalHeader()->setVisible(false); 
	tableWidget_->setSelectionMode(QAbstractItemView::ExtendedSelection);
	tableWidget_->setSelectionBehavior(QAbstractItemView::SelectRows); 
	tableWidget_->setEditTriggers(QAbstractItemView::NoEditTriggers); 
	tableWidget_->horizontalHeader()->resizeSection(0, 150);
	tableWidget_->horizontalHeader()->setFixedHeight(25); 
	tableWidget_->horizontalHeader()->setStyleSheet("QHeaderView::section{background:skyblue;}");

	tableWidget_->setStyleSheet("QTableWidget{border:none; alternate-background-color:rgb(240, 250, 247);}\
		QTableWidget::item{padding:5px;background:transparent;}\
        QTableWidget::item:selected{ background:rgb(106, 194, 172);}");


	//设置水平、垂直滚动条样式
	tableWidget_->horizontalScrollBar()->setStyleSheet("QScrollBar{background:transparent; height:10px;}"
		"QScrollBar::handle{background:lightgray; border:2px solid transparent; border-radius:5px;}"
		"QScrollBar::handle:hover{background:gray;}"
		"QScrollBar::sub-line{background:transparent;}"
		"QScrollBar::add-line{background:transparent;}");

	tableWidget_->verticalScrollBar()->setStyleSheet("QScrollBar{background:transparent; width: 10px;}"
		"QScrollBar::handle{background:lightgray; border:2px solid transparent; border-radius:5px;}"
		"QScrollBar::handle:hover{background:gray;}"
		"QScrollBar::sub-line{background:transparent;}"
		"QScrollBar::add-line{background:transparent;}");
	

	QHBoxLayout *layout = new QHBoxLayout;
	layout->addWidget(tableWidget_);
	setLayout(layout);
}


void DLPointInfoWidget::loadData(QList<DeviceDetail> data)
{
	tableWidget_->clearContents();
	tableWidget_->setRowCount(data.size());

	for (int i = 0; i < data.size(); i++)
	{
		tableWidget_->setItem(i, 0, new QTableWidgetItem(data[i].deviceUUid));
		tableWidget_->setItem(i, 1, new QTableWidgetItem(data[i].VoltageLevel));
		tableWidget_->setItem(i, 2, new QTableWidgetItem(data[i].equipmentInterval));
		tableWidget_->setItem(i, 3, new QTableWidgetItem(data[i].devicePointType));
	}

}



///////////////////////////////


DLCollectMapView::DLCollectMapView(QWidget *parent)
	: QGraphicsView(parent),
	translate_button_(Qt::MidButton),
	scale_(1.0),
	zoom_delta_(0.1),
	translate_speed_(1.0),
	bMouse_translate_(false), 
	viewPort_mode_(false)
	, m_rResolution(0.1)
	, m_bIsFirstIn(true)
{

	initView();
// 	setFocusPolicy(Qt::StrongFocus);
// 	setCacheMode(CacheBackground);
// 	setViewportUpdateMode(BoundingRectViewportUpdate);
// 	setRenderHint(QPainter::Antialiasing);
// 	setTransformationAnchor(AnchorUnderMouse);
// 	//setMatrix(QMatrix(1.0, 0.0, 0.0, -1.0, 0.0, 0.0), true);
// 	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
// 	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
// 	centerOn(0, 0);

	setFocusPolicy(Qt::StrongFocus);
	setCacheMode(CacheBackground);
	//setDragMode(QGraphicsView::RubberBandDrag);
	setViewportUpdateMode(QGraphicsView::SmartViewportUpdate);

	setRenderHint(QPainter::Antialiasing);
	setTransformationAnchor(AnchorUnderMouse);
	//setMatrix( QMatrix(1.0, 0.0 , 0.0 , -1.0 , 0.0 , 0.0) , true );//镜像变换													
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);			   // 去掉滚动条
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	centerOn(0, 0);
}


void DLCollectMapView::initView()
{
	zoom_in_pushbutton_ = new QPushButton;
	zoom_out_pushbutton_ = new QPushButton;
	center_pushbutton_ = new QPushButton;
    add_patrol_point_pushbutton_ = new QPushButton;
	viewPort_mode_pushbutton_ = new QPushButton;
	three_d_view_pushbutton_ = new QPushButton;
	viewPort_mode_pushbutton_->setCheckable(true);

	zoom_in_pushbutton_->setFixedSize(40, 40);
	zoom_out_pushbutton_->setFixedSize(40, 40);
	center_pushbutton_->setFixedSize(40, 40);
    add_patrol_point_pushbutton_->setFixedSize(40, 40);
	viewPort_mode_pushbutton_->setFixedSize(40, 40);

    zoom_in_pushbutton_->setFocusPolicy(Qt::NoFocus);
    zoom_out_pushbutton_->setFocusPolicy(Qt::NoFocus);
    center_pushbutton_->setFocusPolicy(Qt::NoFocus);
    add_patrol_point_pushbutton_->setFocusPolicy(Qt::NoFocus);
    viewPort_mode_pushbutton_->setFocusPolicy(Qt::NoFocus);

	QPixmap pic_zoom_in, pic_zoom_out, pic_center, pic_add_patrol, pic_viewPort, pic_three_d_view;
	pic_zoom_in.load(":/Resources/Common/image/zoom_in.png");
	pic_zoom_out.load(":/Resources/Common/image/zoom_out.png");
	pic_center.load(":/Resources/Common/image/center.png");
    pic_add_patrol.load(":/Resources/map_edite/add_advanced_point.png");
	pic_viewPort.load(":/Resources/Common/image/manual.png");
	pic_three_d_view.load(":/Resources/Common/image/3D_view.png");
	pic_zoom_in.scaled(40, 40);
	pic_zoom_out.scaled(40, 40);
	pic_center.scaled(40, 40);
    pic_add_patrol.scaled(40, 40);
	pic_viewPort.scaled(40, 40);
	pic_three_d_view.scaled(40, 40);

	zoom_in_pushbutton_->setIcon(QIcon(pic_zoom_in));
	zoom_in_pushbutton_->setIconSize(QSize(40, 40));
	zoom_out_pushbutton_->setIcon(QIcon(pic_zoom_out));
	zoom_out_pushbutton_->setIconSize(QSize(40, 40));
	center_pushbutton_->setIcon(QIcon(pic_center));
	center_pushbutton_->setIconSize(QSize(40, 40));
    add_patrol_point_pushbutton_->setIcon(QIcon(pic_add_patrol));
    add_patrol_point_pushbutton_->setIconSize(QSize(40, 40));
	viewPort_mode_pushbutton_->setIcon(QIcon(pic_viewPort));
	viewPort_mode_pushbutton_->setIconSize(QSize(40, 40));
	three_d_view_pushbutton_->setIcon(QIcon(pic_three_d_view));
	three_d_view_pushbutton_->setIconSize(QSize(40, 40));


	QHBoxLayout *main_layout = new QHBoxLayout;
	QVBoxLayout *bottom_layout = new QVBoxLayout;
    bottom_layout->addWidget(add_patrol_point_pushbutton_);
	bottom_layout->addWidget(viewPort_mode_pushbutton_);
	bottom_layout->addStretch();
	bottom_layout->addWidget(zoom_in_pushbutton_);
	bottom_layout->addWidget(zoom_out_pushbutton_);
	bottom_layout->addWidget(center_pushbutton_);
	bottom_layout->addWidget(three_d_view_pushbutton_);

	main_layout->addStretch();
	main_layout->addLayout(bottom_layout);
	this->setLayout(main_layout);

	connect(zoom_in_pushbutton_, SIGNAL(clicked()), this, SLOT(zoomIn()));
	connect(zoom_out_pushbutton_, SIGNAL(clicked()), this, SLOT(zoomOut()));
	connect(center_pushbutton_, SIGNAL(clicked()), this, SLOT(slot_on_center()));
    connect(add_patrol_point_pushbutton_, SIGNAL(clicked()), this, SIGNAL(sig_add_patrolPoint()));
	connect(viewPort_mode_pushbutton_, SIGNAL(clicked(bool)), this, SLOT(slot_on_viewPortMode(bool)));
	connect(three_d_view_pushbutton_, SIGNAL(clicked()), this, SLOT(slot_on_view3d()));



	m_pResolutionLbl = new QLabel(this);
	m_pResolutionLbl->setObjectName("resolution");
	m_pResolutionLbl->setFixedHeight(50);
	//m_pResolutionLbl->setFixedWidth(50);
	m_pResolutionLbl->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	QFont font = m_pResolutionLbl->font();
	font.setPointSize(20);
	m_pResolutionLbl->setFont(font);
	//m_pResolutionLbl->setPalette(QPalette(QPalette::WindowText, Qt::red));
// 	qreal rTemp = m_rResolution;
// 	int iValue = 1;
// 	while (rTemp < 1.0)
// 	{
// 		rTemp = rTemp * 10;
// 		iValue = iValue * 10;
// 	}
// 
// 	m_pResolutionLbl->setText(QString("%1:%2").arg(rTemp).arg(iValue));

	m_pResolutionNameLbl = new QLabel(this);
	m_pResolutionNameLbl->setObjectName("resolutionname");
	m_pResolutionNameLbl->setFixedHeight(50);
	m_pResolutionNameLbl->setFixedWidth(RESOLUTION_NAME_WIDTH);
	QSize size(RESOLUTION_NAME_WIDTH/2, 25);
	QPixmap pixmap;
	pixmap.load(":/Resources/Common/image/scale.png");
	pixmap = pixmap.scaled(size, Qt::KeepAspectRatio);
	m_pResolutionNameLbl->setPixmap(pixmap);
	m_pResolutionNameLbl->setAlignment(Qt::AlignCenter);
}


void DLCollectMapView::SetResolution(qreal rResolution)
{
	m_rResolution = rResolution;
	qreal rTemp = m_rResolution;
	int iValue = 1;
	while (rTemp < 1.0)
	{
		rTemp = rTemp * 10;
		iValue = iValue * 10;
	}

	m_pResolutionLbl->setText(QString("%1:%2").arg(rTemp).arg(iValue));
	m_pResolutionLbl->update();
}

void DLCollectMapView::slot_on_center()
{

	centerOn(0, 0);

}


void DLCollectMapView::slot_on_flush_viewPort(QPointF point)
{
	if (viewPort_mode_) {
		centerOn(point);
	}
}


void DLCollectMapView::slot_on_viewPortMode(bool mode)
{
	viewPort_mode_ = mode;
	QPixmap pic_viewPort;
	if (mode) {
		pic_viewPort.load(":/Resources/Common/image/auto.png");
		pic_viewPort.scaled(40, 40);
		viewPort_mode_pushbutton_->setIcon(QIcon(pic_viewPort));
		viewPort_mode_pushbutton_->setIconSize(QSize(40, 40));
	}
	else{
		pic_viewPort.load(":/Resources/Common/image/manual.png");
		pic_viewPort.scaled(40, 40);
		viewPort_mode_pushbutton_->setIcon(QIcon(pic_viewPort));
		viewPort_mode_pushbutton_->setIconSize(QSize(40, 40));
	}
}


void DLCollectMapView::slot_on_view3d()
{	
	//((DLCollectMapScene*)scene())->show_3d_viewer();
}


void DLCollectMapView::setTranslateSpeed(double speed)
{
	// 建议速度范围
	Q_ASSERT_X(speed >= 0.0 && speed <= 2.0,
		"InteractiveView::setTranslateSpeed", "Speed should be in range [0.0, 2.0].");
	translate_speed_ = speed;
}


double DLCollectMapView::translateSpeed()
{
	return translate_speed_;
}


void DLCollectMapView::setZoomDelta(double delta)
{
	// 建议增量范围
	Q_ASSERT_X(delta >= 0.0 && delta <= 1.0,
		"InteractiveView::setZoomDelta", "Delta should be in range [0.0, 1.0].");
	zoom_delta_ = delta;
}


double DLCollectMapView::zoomDelta()
{
	return zoom_delta_;
}


void DLCollectMapView::keyPressEvent(QKeyEvent *event)
{
	switch (event->key()) {
	case Qt::Key_Plus:  // 放大
		zoomIn();
		break;
	case Qt::Key_Minus:  // 缩小
		zoomOut();
		break;
	case Qt::Key_Escape:
	case Qt::Key_Delete:
		QGraphicsView::keyPressEvent(event);
		break;
	default:break;
	}
}




void DLCollectMapView::mouseMoveEvent(QMouseEvent *event)
{

	if (bMouse_translate_) {
		QPointF mouseDelta = mapToScene(event->pos()) - mapToScene(last_mousePos_);
		translate(mouseDelta);
		//emit sig_viewRect_changed(QRectF(mapToScene(viewport()->rect()).boundingRect()));
	}

	last_mousePos_ = event->pos();
	QGraphicsView::mouseMoveEvent(event);
}


void DLCollectMapView::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == translate_button_) {

		// 当光标底下没有 item 时，才能移动
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


void DLCollectMapView::mouseReleaseEvent(QMouseEvent *event)
{
	if (event->button() == translate_button_) {
		bMouse_translate_ = false;
		setCursor(Qt::ArrowCursor);
	}
	QGraphicsView::mouseReleaseEvent(event);
}


void DLCollectMapView::wheelEvent(QWheelEvent *event)
{
	// 滚轮的滚动量
	QPoint scrollAmount = event->angleDelta();
	// 正值表示滚轮远离使用者（放大），负值表示朝向使用者（缩小）
	scrollAmount.y() > 0 ? zoomIn() : zoomOut();
	emit sig_viewRect_changed(QRectF(mapToScene(viewport()->rect()).boundingRect()));
}


void DLCollectMapView::showEvent(QShowEvent *event)
{
	if (m_bIsFirstIn)
	{
		m_bIsFirstIn = false;
		QRect rect = this->rect();
		m_pResolutionNameLbl->move(rect.bottomLeft() - QPoint(0, 50));
		m_pResolutionLbl->move(rect.bottomLeft() - QPoint(-RESOLUTION_NAME_WIDTH, 50));
		centerOn(0, 0);
	}
}

void DLCollectMapView::zoomIn()
{

	zoom(1 + zoom_delta_);

}


void DLCollectMapView::zoomOut()
{
	zoom(1 - zoom_delta_);
}


void DLCollectMapView::zoom(float scaleFactor)
{
	// 防止过小或过大
	qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
	if (factor < 0.07 || factor > 100)
		return;

	scale(scaleFactor, scaleFactor);
	scale_ *= scaleFactor;

	((DLCustomScene*)scene())->UpdateNormalBackground();
}


void DLCollectMapView::translate(QPointF delta)
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
	((DLCustomScene*)scene())->UpdateNormalBackground();
}



//////////////////////////////////////////////////////////////////

int last_start_index_ = 0;
int last_end_index_ = 0;

void set_3d_map(WheelRobotTaskCurrentPointStatus data)
{
	last_start_index_ = data.start_point;
	last_end_index_ = data.end_point;
}

