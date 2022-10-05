#include <QtWidgets>
#include "DLView.h"
#include "DLCustomScene.h"
#include "DLOperator.h"
#include <QDebug>

#define HLINECOUNT (500+1)
#define VLINECOUNT (500+1)

#define VIEW_CENTER (viewport()->rect().center())
#define VIEW_WIDTH  (viewport()->rect().width())
#define VIEW_HEIGHT (viewport()->rect().height())
#define VIEW_MARGIN	(30)


DLView::DLView(QWidget *parent)
	: QGraphicsView(parent),
	translate_button_(Qt::LeftButton),
	scale_(1.0),
	zoom_delta_(0.1),
	translate_speed_(1.0),
	bMouse_translate_(false)
	, m_bIsCutoutState(false)
	, m_bIsPressed(false)
	,m_ptStartPoint(0,0)
	, m_ptEndPoint(0,0)
{

	initView();
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


void DLView::initView()
{ 
	zoom_in_pushbutton_ = new QPushButton;
	zoom_out_pushbutton_ = new QPushButton;
	center_pushbutton_ = new QPushButton;
	//three_d_view_pushbutton_ = new QPushButton;
	zoom_in_pushbutton_->setFixedSize(40, 40);
	zoom_out_pushbutton_->setFixedSize(40, 40);
	center_pushbutton_->setFixedSize(40, 40);
	//three_d_view_pushbutton_->setFixedSize(40, 40);

	QPixmap pic_zoom_in, pic_zoom_out, pic_center, three_d_view;
	pic_zoom_in.load(":/Resources/Common/image/zoom_in.png");
	pic_zoom_out.load(":/Resources/Common/image/zoom_out.png");
	pic_center.load(":/Resources/Common/image/center.png");
	//bool ret = three_d_view.load(":/Resources/Common/image/3D_view.png");
	pic_zoom_in.scaled(40, 40);
	pic_zoom_out.scaled(40, 40);
	pic_center.scaled(40, 40);
	//three_d_view.scaled(40, 40);

	zoom_in_pushbutton_->setIcon(QIcon(pic_zoom_in));
	zoom_out_pushbutton_->setIcon(QIcon(pic_zoom_out));
	center_pushbutton_->setIcon(QIcon(pic_center));
	//three_d_view_pushbutton_->setIcon(QIcon(three_d_view));

	QHBoxLayout *main_layout = new QHBoxLayout;
	QVBoxLayout *bottom_layout = new QVBoxLayout;
	//bottom_layout->addWidget(three_d_view_pushbutton_);
	bottom_layout->addStretch();
	bottom_layout->addWidget(zoom_in_pushbutton_);
	bottom_layout->addWidget(zoom_out_pushbutton_);
	bottom_layout->addWidget(center_pushbutton_);

	main_layout->addStretch();
	main_layout->addLayout(bottom_layout);
	this->setLayout(main_layout);

	connect(zoom_in_pushbutton_, SIGNAL(clicked()), this, SLOT(zoomIn()));
	connect(zoom_out_pushbutton_, SIGNAL(clicked()), this, SLOT(zoomOut()));
	connect(center_pushbutton_, SIGNAL(clicked()), this, SLOT(slot_on_center()));
	//connect(three_d_view_pushbutton_, SIGNAL(clicked()), this, SLOT(sh()));
}


void DLView::slot_on_center()
{
	
	centerOn(0, 0);

}


void DLView::drawBackground(QPainter *painter, const QRectF &exposed_rect)
{
	horizon_vec_.clear();
	vertical_vec_.clear();


	QPoint p1(this->rect().topLeft().x(), this->rect().topLeft().y());
	QPoint p2(this->rect().bottomRight().x(), this->rect().bottomRight().y());
	QPoint p3(this->rect().topRight().x(), this->rect().topRight().y());
	QPoint p4(this->rect().bottomLeft().x(), this->rect().bottomLeft().y());

	QPointF topLeft = mapToScene(p1);
	QPointF bottomRight = mapToScene(p2);
	QPointF topRight = mapToScene(p3);
	QPointF bottomLeft = mapToScene(p4);

	double exposed_min_horizon = topLeft.x();
	double exposed_max_horizon = bottomRight.x();
	double exposed_min_vertical = bottomRight.y();
	double exposed_max_vertical = topLeft.y();
	
	QRectF rect = sceneRect();
	QPointF center_pos = rect.center();
	QPointF top_left = rect.topLeft();
	QPointF bottom_left = rect.bottomLeft();
	QPointF top_right = rect.topRight();
	QPointF bottom_right = rect.bottomRight();

	double h_offset = abs( (top_left - top_right).x() ) / (HLINECOUNT - 1);
	double v_offset = abs( (bottom_left - top_left).y() ) / (VLINECOUNT - 1);

	//计算水平线
	for (int i = 0; i < HLINECOUNT; ++i)
	{
		double temp = top_left.y() + i * v_offset;
		QPointF left_pos(top_left.x(), temp);
		QPointF right_pos(top_right.x(), temp);
		QLineF line(left_pos, right_pos);

		
		if (temp > exposed_min_vertical && temp < exposed_max_vertical) {

			bool isSpecial = false;
			if ((i % 5) == 0) {
				isSpecial = true;
			}

			ScaleInfo info;
			info.point_ = QPointF(exposed_min_horizon, temp);
			info.flag_ = isSpecial;
			vertical_vec_.push_back(info);
		}
	}

	//计算垂直线
	for (int i = 0; i < VLINECOUNT; ++i)
	{
		double temp = top_left.x() + i * h_offset;
		QPointF top_pos(temp, top_left.y());
		QPointF bottom_pos(temp, bottom_left.y());
		QLineF line(top_pos, bottom_pos);

		if (temp > exposed_min_horizon && temp < exposed_max_horizon) {

			bool isSpecial = false;
			if ((i % 5) == 0) {
				isSpecial = true;
			}

			ScaleInfo info;
			info.point_ = QPointF(temp, exposed_max_vertical);
			info.flag_ = isSpecial;
			horizon_vec_.push_back(info);
		}
	}
	
	emit sig_changed();
}


void DLView::paintEvent(QPaintEvent *event)
{
	QGraphicsView::paintEvent(event);
	if (m_bIsCutoutState && m_bIsPressed)
	{
		QPainter painter(this);
		QPen pen(Qt::blue, 3, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);
		painter.setPen(pen);
		painter.setBrush(Qt::blue);
		painter.setOpacity(0.5);
		painter.drawRect(QRect(m_ptStartPoint, m_ptEndPoint));
	}
}

std::vector<ScaleInfo> DLView::get_horizon()
{
	return horizon_vec_;
}


std::vector<ScaleInfo> DLView::get_vertical()
{
	return vertical_vec_;
}


void DLView::SetCutOutState(bool bIsCutOutState)
{
	m_bIsCutoutState = bIsCutOutState;
}

void DLView::setTranslateSpeed(double speed)
{
	// 建议速度范围
	Q_ASSERT_X(speed >= 0.0 && speed <= 2.0,
		"InteractiveView::setTranslateSpeed", "Speed should be in range [0.0, 2.0].");
	translate_speed_ = speed;
}


double DLView::translateSpeed()
{
	return translate_speed_;
}


void DLView::setZoomDelta(double delta)
{
	// 建议增量范围
	Q_ASSERT_X(delta >= 0.0 && delta <= 1.0,
		"InteractiveView::setZoomDelta", "Delta should be in range [0.0, 1.0].");
	zoom_delta_ = delta;
}


double DLView::zoomDelta()
{
	return zoom_delta_;
}


void DLView::keyPressEvent(QKeyEvent *event)
{
	switch (event->key()) {
	case Qt::Key_Up:
		translate(QPointF(0, -20));  // 上移
		break;
	case Qt::Key_Down:
		translate(QPointF(0, 20));  // 下移
		break;
	case Qt::Key_Left:
		translate(QPointF(-20, 0));  // 左移
		break;
	case Qt::Key_Right:
		translate(QPointF(20, 0));  // 右移
		break;
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


void DLView::mouseMoveEvent(QMouseEvent *event)
{
	if (m_bIsCutoutState && m_bIsPressed)
	{
		m_ptEndPoint = event->pos();
	}
	else
	{
		if (((DLCustomScene*)scene())->GetOperate()->type() != DLOperator::TYPE_MOVE) {
			QGraphicsView::mouseMoveEvent(event);
			return;
		}

		if (bMouse_translate_) {
			QPointF mouseDelta = mapToScene(event->pos()) - mapToScene(last_mousePos_);
			translate(QPointF(mouseDelta.x(), -mouseDelta.y()));
		}
		if (bMouse_translate_) {
			emit sig_viewRect_changed(QRectF(mapToScene(viewport()->rect()).boundingRect()));
		}
		last_mousePos_ = event->pos();
	}
	QGraphicsView::mouseMoveEvent(event);
}


void DLView::mousePressEvent(QMouseEvent *event)
{
	if (m_bIsCutoutState)
	{
		m_ptStartPoint = event->pos();
		m_ptEndPoint = event->pos();
		m_bIsPressed = true;
	}
	else
	{
		if (event->button() == translate_button_) {

			// 当光标底下没有 item 时，才能移动
			QPointF point = mapToScene(event->pos());
			if (scene()->itemAt(point, transform()) == NULL) {
				//setCursor(Qt::SizeAllCursor);
				bMouse_translate_ = true;
				last_mousePos_ = event->pos();
			}
		}
	}
	

	QGraphicsView::mousePressEvent(event);
}


void DLView::mouseReleaseEvent(QMouseEvent *event)
{
	if (m_bIsCutoutState && m_bIsPressed)
	{
		m_ptEndPoint = event->pos();
		m_bIsPressed = false;
		m_bIsCutoutState = false;
		//截图
		QPixmap pixmap = this->grab(QRect(m_ptStartPoint, m_ptEndPoint));
		pixmap.save("D:/test.png");
	}
	else
	{
		if (event->button() == translate_button_) {
			bMouse_translate_ = false;
			//setCursor(Qt::ArrowCursor);
		}
	}
	
	
	//qDebug() << scale_ << ", scene size = " << mapToScene(this->rect()).boundingRect() << " sceneRect() = " << scene()->itemsBoundingRect();
	QGraphicsView::mouseReleaseEvent(event);
}


void DLView::wheelEvent(QWheelEvent *event)
{
	// 滚轮的滚动量
	QPoint scrollAmount = event->angleDelta();
	// 正值表示滚轮远离使用者（放大），负值表示朝向使用者（缩小）
	scrollAmount.y() > 0 ? zoomIn() : zoomOut();
	emit sig_viewRect_changed(QRectF(mapToScene(viewport()->rect()).boundingRect()));
	//qDebug() << "rect=" << this->rect() << " scene= " << mapToScene(this->rect()).boundingRect() << " viewprot() = " << viewport()->rect();
}


void DLView::zoomIn()
{

	zoom(1 + zoom_delta_);

}


void DLView::zoomOut()
{
	zoom(1 - zoom_delta_);
}


void DLView::zoom(float scaleFactor)
{
	// 防止过小或过大
	qreal factor = transform().scale(scaleFactor, scaleFactor).mapRect(QRectF(0, 0, 1, 1)).width();
	if (factor < 0.07 || factor > 100)
		return;

	scale(scaleFactor, scaleFactor);
	scale_ *= scaleFactor;
}


void DLView::translate(QPointF delta)
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


/////////////////////////////////////////////


DLFrameView::DLFrameView()
{
	view_ = new DLView;
	
	view_->setStyleSheet("background-color:rgb(229,231,218);");

	QHBoxLayout *layout = new QHBoxLayout;
	layout->setContentsMargins(VIEW_MARGIN, VIEW_MARGIN, 0, 0);
	layout->addWidget(view_);
	this->setLayout(layout);
	connect(view_, SIGNAL(sig_changed()), this, SLOT(update()));

}


void DLFrameView::setScene(DLCustomScene* scene)
{
	view_->setScene(scene);
	connect(view_, SIGNAL(sig_viewRect_changed(const QRectF&)), scene, SIGNAL(sig_sceneRect_changed(const QRectF&)));
}


void DLFrameView::SetCutOutState(bool bIsCutOutState)
{
	view_->SetCutOutState(bIsCutOutState);
	
}

DLFrameView::~DLFrameView()
{
	//if (view_ != NULL) {
	//	delete view_;
	//	view_ = NULL;
	//}
}


void DLFrameView::paintEvent(QPaintEvent *e)
{
	QPainter painter(this);
	
	painter.save();
	painter.setPen(QPen(Qt::lightGray, 3));
	painter.drawRect(this->rect());
	painter.restore();

	std::vector<ScaleInfo> horizon_vec = view_->get_horizon();
	std::vector<ScaleInfo> vertical_vec = view_->get_vertical();
	

	//绘制垂直刻度线
	int count_horizon = horizon_vec.size();
	for (int i = 0; i < horizon_vec.size(); ++i)
	{
		QPointF start_point = view_->mapFromScene(horizon_vec[i].point_) + QPointF(0, VIEW_MARGIN);
		QPointF end_point;
		end_point.setX(start_point.x());
		if (horizon_vec[i].flag_) {
			end_point.setY(start_point.y() - 12);
			QRectF rect(start_point.x(), start_point.y() - VIEW_MARGIN, 60, 18);
			if (count_horizon < 100) {
				painter.drawText(rect, Qt::AlignCenter, QString::number(((int)horizon_vec[i].point_.x())));
			}
			else{
				int value = horizon_vec[i].point_.x();
				if ((value / 400) % 2 == 0) {
					painter.drawText(rect, Qt::AlignCenter, QString::number(((int)horizon_vec[i].point_.x())));
				}
			}

		}
		else {
			end_point.setY(start_point.y() - 6);
		}
		painter.drawLine(QLineF(start_point + QPointF(VIEW_MARGIN, 0), end_point + QPointF(VIEW_MARGIN, 0)));
	}
	
	//绘制水平刻度线
	int count_vertical = vertical_vec.size();
	for (int i = 0; i < vertical_vec.size(); ++i)
	{
		QPointF start_point = view_->mapFromScene(vertical_vec[i].point_) + QPointF(VIEW_MARGIN, 0);
		QPointF end_point;
		end_point.setY(start_point.y());
		if (vertical_vec[i].flag_) {
			end_point.setX(start_point.x() - 12);
			//QRectF rect(start_point.y(), start_point.x() - 60 + 12, 60, 18);
			QRectF rect(-start_point.y() - 60, -start_point.x() + 60 - 30, 60, 18);

			if (count_vertical < 75) {
				painter.save();
				painter.rotate(-90);
				//painter.rotate(90);
				painter.drawText(rect, Qt::AlignCenter, QString::number(((int)vertical_vec[i].point_.y())));
				painter.restore();
			}
			else {
				int value = vertical_vec[i].point_.y();
				if ((value / 400) % 2 == 0) {
					painter.save();
					painter.rotate(-90);
					//painter.rotate(90);
					painter.drawText(rect, Qt::AlignCenter, QString::number(((int)vertical_vec[i].point_.y())));
					painter.restore();
				}
			}
		}
		else {
			end_point.setX(start_point.x() - 6);
		}
		painter.drawLine(QLineF(start_point + QPointF(0, VIEW_MARGIN), end_point + QPointF(0, VIEW_MARGIN)));
	}
	
	


}
