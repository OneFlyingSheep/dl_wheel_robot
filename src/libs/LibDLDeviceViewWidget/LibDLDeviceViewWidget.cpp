#include "LibDLDeviceViewWidget.h"
#include <QtWidgets>
#include "LibDlToolItems/DLDeviceItem.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "common/DLRobotCommonDef.h"


#define CATCH_FACTOR (5)
#define PI (3.14159)
#define VIEW_CENTER (viewport()->rect().center())
#define VIEW_WIDTH  (viewport()->rect().width())
#define VIEW_HEIGHT (viewport()->rect().height())


DLDeviceViewer::DLDeviceViewer(QWidget *parent)
	: QWidget(parent)
{
	view_ = new DLDeviceGraphcisView;
	scene_ = new DLDeviceGraphicsScene;
	view_->setScene(scene_);
	view_->setSceneRect(-10000, -10000, 20000, 20000);
	QHBoxLayout *main_layout = new QHBoxLayout;
	main_layout->addWidget(view_);
	main_layout->setContentsMargins(0, 0, 0, 0);
	this->setLayout(main_layout);
	this->setWindowFlags(Qt::FramelessWindowHint);

}


void DLDeviceViewer::loadDevice()
{
	scene_->clear();
	scene_->loadDevice();
}


void DLDeviceViewer::set_device_propertry(WheelInspectResultStruct current_task_info)
{
	
	scene_->setDevice(current_task_info);
}



DLDeviceGraphcisView::DLDeviceGraphcisView(QWidget *parent)
	: QGraphicsView(parent),
	translate_button_(Qt::LeftButton),
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
	setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	centerOn(0, 0);

}

void DLDeviceGraphcisView::initView()
{
	this->setFixedSize(640, 480);
	this->setStyleSheet("background-color:rgb(229,231,218);");
}


void DLDeviceGraphcisView::setTranslateSpeed(double speed)
{
	// 建议速度范围
	Q_ASSERT_X(speed >= 0.0 && speed <= 2.0,
		"InteractiveView::setTranslateSpeed", "Speed should be in range [0.0, 2.0].");
	translate_speed_ = speed;
}


double DLDeviceGraphcisView::translateSpeed()
{
	return translate_speed_;
}


void DLDeviceGraphcisView::setZoomDelta(double delta)
{
	// 建议增量范围
	Q_ASSERT_X(delta >= 0.0 && delta <= 1.0,
		"InteractiveView::setZoomDelta", "Delta should be in range [0.0, 1.0].");
	zoom_delta_ = delta;
}


double DLDeviceGraphcisView::zoomDelta()
{
	return zoom_delta_;
}


void DLDeviceGraphcisView::mouseMoveEvent(QMouseEvent *event)
{

	if (bMouse_translate_) {
		QPointF mouseDelta = mapToScene(event->pos()) - mapToScene(last_mousePos_);
		translate(mouseDelta);
	}

	last_mousePos_ = event->pos();
	QGraphicsView::mouseMoveEvent(event);
}


void DLDeviceGraphcisView::mousePressEvent(QMouseEvent *event)
{
	if (event->button() == translate_button_) {

		// 当光标底下没有 item 时，才能移动
		QPointF point = mapToScene(event->pos());
		if (scene()->itemAt(point, transform()) == NULL) {
			setCursor(Qt::SizeAllCursor);
			bMouse_translate_ = true;
			last_mousePos_ = event->pos();
		}
	}
	QGraphicsView::mousePressEvent(event);
}


void DLDeviceGraphcisView::mouseReleaseEvent(QMouseEvent *event)
{
	if (event->button() == translate_button_) {
		bMouse_translate_ = false;
		setCursor(Qt::ArrowCursor);
	}
	QGraphicsView::mouseReleaseEvent(event);
}


void DLDeviceGraphcisView::wheelEvent(QWheelEvent *event)
{
	// 滚轮的滚动量
	QPoint scrollAmount = event->angleDelta();
	// 正值表示滚轮远离使用者（放大），负值表示朝向使用者（缩小）
	scrollAmount.y() > 0 ? zoomIn() : zoomOut();
}


void DLDeviceGraphcisView::zoomIn()
{

	zoom(1 + zoom_delta_);

}


void DLDeviceGraphcisView::zoomOut()
{
	zoom(1 - zoom_delta_);
}


void DLDeviceGraphcisView::zoom(float scaleFactor)
{
	// 防止过小或过大
	qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
	if (factor < 0.07 || factor > 100)
		return;

	scale(scaleFactor, scaleFactor);
	scale_ *= scaleFactor;
}


void DLDeviceGraphcisView::translate(QPointF delta)
{

	// 根据当前 zoom 缩放平移数
	delta *= scale_;
	delta *= translate_speed_;

	// view 根据鼠标下的点作为锚点来定位 scene
	setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
	QPoint newCenter(VIEW_WIDTH / 2 - delta.x(), VIEW_HEIGHT / 2 + delta.y());
	centerOn(mapToScene(newCenter));

	// scene 在 view 的中心点作为锚点
	setTransformationAnchor(QGraphicsView::AnchorViewCenter);
}



//////////////////////////////////////////////////////////////////



DLDeviceGraphicsScene::DLDeviceGraphicsScene(QObject *parent) : QGraphicsScene(parent)
{

	setItemIndexMethod(QGraphicsScene::NoIndex);
	
}


void DLDeviceGraphicsScene::loadDevice()
{
	std::map<QString, QStringList> device_map;
	WHEEL_ROBOT_DB.searchDeviceUUidWithDeviceAreaDB(device_map);
	//QStringList list1, list2, list3;
	//list1 << "1" << "2" << "3" << "4" << "5" << "6" << "7";

	//list2 << "1" << "2" << "3" << "4" << "5" << "6" << "7";

	//list3 << "1" << "2" << "3" << "4" << "5" << "6" << "7";

	//device_map["220Kv"] = list1;
	//device_map["110Kv"] = list2;
	//device_map["550Kv"] = list3;

	std::map<QString, QStringList>::iterator itor = device_map.begin();
	int  device_area_colomn = 0;
	while (itor != device_map.end())
	{

		QStringList device_list = itor->second;
		int count = device_list.size();
		
		int row = 0;
		int colomn = 0;
		for (int i = 0; i < device_list.size(); i++)
		{
			HShape::Rectangle pro;
			pro.center_.x_ = 30 + colomn * 50 + device_area_colomn*180;
			pro.center_.y_ = 25 + row * 40;
			pro.width_ = 40;
			pro.height_ = 30;

			if ((i+1) % 3 == 0)
			{
				row++;
				colomn = 0;
			}
			else {
				colomn++;
			}

			DLDeviceItem *rect = new DLDeviceItem(pro, device_list[i]);
			rect->setZValue(10);
			rect->setPos(pro.center_.x_, pro.center_.y_);
			rect->setState(3);
			device_list_[device_list[i]] = rect;
			rect->setMatrix(QMatrix(1.0, 0.0, 0.0, -1.0, 0.0, 0.0), true);
			addItem(rect);
			rect->setToolTip(device_list[i]);
		}

		QGraphicsRectItem *item = new QGraphicsRectItem;
		item->setZValue(1);
		item->setRect(QRect(0 + device_area_colomn * 180, 0, 160, (row+1)*45));
		item->setOpacity(0.3);
		item->setBrush(Qt::lightGray);
		item->setPen(Qt::NoPen);
		this->addItem(item);


		HShape::Rectangle area_label_pro;
		area_label_pro.center_.x_ = 80 + device_area_colomn * 180;
		area_label_pro.center_.y_ = -45;
		area_label_pro.width_ = 160;
		area_label_pro.height_ = 50;
		DLDeviceItem *area_label_item = new DLDeviceItem(area_label_pro, itor->first);
		area_label_item->setPos(area_label_pro.center_.x_, area_label_pro.center_.y_);
		area_label_item->setMatrix(QMatrix(1.0, 0.0, 0.0, -1.0, 0.0, 0.0), true);
		this->addItem(area_label_item);

	
		device_area_colomn++;
		itor++;
	}
	//ROS_INFO("DLDeviceGraphicsScene device taotal size:%d", device_map.size());
}


void DLDeviceGraphicsScene::setDevice(WheelInspectResultStruct current_task_info)
{
	QMap<QString, QGraphicsItem*>::iterator itor = device_list_.find(current_task_info.device_uuid);
	if (itor != device_list_.end()) {
		int state = 0;
		switch (current_task_info.alarm_level_id)
		{
		case Alarm_NONE:
			state = 0;
			break;
		case Alarm_Normal:
			state = 1;
			break;
		case Alarm_Waring:
			state = 2;
			break;
		case Alarm_Common:
			state = 3;
			break;
		case Alarm_Serious:
			state = 4;
			break;
		case Alarm_Dangerous:
			state = 5;
			break;
		case Alarm_NoIdentifyAbnormal:
			state = 6;
			break;
		default:
			break;
		}
		((DLDeviceItem*)itor.value())->setState(state);
	}
}

DLDeviceGraphicsScene::~DLDeviceGraphicsScene()
{

}


void DLDeviceGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	
}


void DLDeviceGraphicsScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	
}


void DLDeviceGraphicsScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	
}






