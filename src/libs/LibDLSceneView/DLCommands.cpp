#include <QtWidgets>
#include <QGraphicsItem>
#include <iostream>
#include <math.h>
#include <QDebug>
#include "DLCustomScene.h"
#include "DLOperator.h"
#include "DLCommands.h"
#include "LibDLToolItems/HShape.h"
#include "LibDLToolItems/DLLandmarkItem.h"
#include "LibDLToolItems/DLBezierItem.h"
#include "LibDLToolItems/DLShapeItem.h"
#include "LibDLToolItems/DLDeviceItem.h"
#include "LibDLToolItems/DLPolygonItem.h"
#include "LibDLToolItems/DLAdvancedAreaItem.h"
#include "LibDLToolItems/DLDeviceAreaItem.h"
#include "LibDLToolItems/DLSegmentItem.h"
#include "LibDLToolItems/DLRelocateItem.h"
#include "LibDLRelocateProcesser/DLRelocateProcesser.h"




DLUndoCommandInterface::DLUndoCommandInterface(QGraphicsScene *scene):scene_(scene)
{
	
}


DLUndoCommandInterface::DLUndoCommandInterface(QGraphicsScene *scene, int type):scene_(scene), type_(type)
{

}


void DLUndoCommandInterface::mousePressEvent(QGraphicsSceneMouseEvent *event)
{

}


void DLUndoCommandInterface::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{

}


void DLUndoCommandInterface::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{

}


/////////////////////////////////////////////////////////


DLAddLandmarkCommand::DLAddLandmarkCommand(QGraphicsScene *scene):DLUndoCommandInterface(scene)
{

	item_=NULL;

}


void DLAddLandmarkCommand::undo()
{
	if((scene()!=NULL)&&(item_!=NULL))
	{
		//scene_->removeItem(item_);
		//scene_->update();
		DLCustomScene *s=(DLCustomScene*)scene();
		s->remove_advancedpoint(item_->id());

	}
}


void DLAddLandmarkCommand::redo()
{
	if((scene()!=NULL)&&(item_!=NULL))
	{
		//scene_->addItem(item_);
		//scene_->update();
		DLCustomScene *s=(DLCustomScene*)scene();
		s->add_advancedPoint(item_);

	}
}


bool DLAddLandmarkCommand::is_useful()
{
	if(item_!=NULL){
		return true;
	}
	else{
		return false;
	}
}


void DLAddLandmarkCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{

}


void DLAddLandmarkCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{

}


void DLAddLandmarkCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{

	if(event->button()==Qt::LeftButton)
	{
		QPointF pos=event->scenePos();

		HShape::Ellipse pro;
		pro.center_.x_=pos.x();
		pro.center_.y_=pos.y();
		pro.width_=20;
		pro.height_=20;
		pro.angle_=0;

		DLCustomScene *s=(DLCustomScene*)scene();
		int id = s->GetLandmarkId()+1;
		QString name = /*"LM" +*/ QString::number(id);
		item_ = new DLLandmarkItem(pro, id, name.toStdString());
	}
	

}


/////////////////////////////////////////////////////////


DLRelocationCommand::DLRelocationCommand(QGraphicsScene *scene)
	: DLUndoCommandInterface(scene)
{
	relocate_item_ = NULL;
	isPress = false;
	//relocateProcesser_ = NULL;

	relocate_tip_widget_ = new QWidget;
	QVBoxLayout *main_layout = new QVBoxLayout;
	relocate_tip_widget_->setWindowFlags(Qt::FramelessWindowHint);
	relocate_tip_widget_->setStyleSheet("background-color: rgb(229,231,200);");

	confirm_relocate_button_ = new QPushButton(QStringLiteral("确认"));
	
	relocate_tip_label_ = new QLabel(QStringLiteral("正在执行重定位"));
	relocate_tip_label_->setAlignment(Qt::AlignCenter);
	confirm_relocate_button_->hide();
	confirm_relocate_button_->setStyleSheet("background: rgb(229,231,200);");

	main_layout->addWidget(relocate_tip_label_);
	main_layout->addWidget(confirm_relocate_button_);

	relocate_tip_widget_->setLayout(main_layout);
	relocate_tip_widget_->setFixedSize(200, 150);
	relocate_tip_widget_->setWindowModality(Qt::ApplicationModal);
	
	connect(confirm_relocate_button_, SIGNAL(clicked()), this, SLOT(slot_confirm_relocate()));
}


void DLRelocationCommand::RobotRelcation(QPointF &ptRobotPos, double dAngle)
{
	qreal rResolution = 0.0;
	DLCustomScene *pScene = dynamic_cast<DLCustomScene *>(scene());
	if (NULL != pScene)
	{
		rResolution = pScene->GetResolution();
	}

	//relocateProcesser_ = new DLRelocateProcesser((DLCustomScene*)scene(), start_point_.x()*rResolution, start_point_.y()*rResolution, point_angle);
    relocateProcesser_ = new DLRelocateProcesser((DLCustomScene*)scene(), ptRobotPos.x()*rResolution, ptRobotPos.y()*rResolution, dAngle);
//    relocateProcesser_ = new DLRelocateProcesser((DLCustomScene*)scene(), ptRobotPos.x()*rResolution, ptRobotPos.y()*rResolution, dAngle);
	relocateProcesser_->start();

	relocate_tip_label_->setText(QStringLiteral("正在进行重定位"));
	//relocate_tip_widget_->show();

	connect(relocateProcesser_, SIGNAL(signal_relocation_finished(int)), this, SLOT(slot_get_relocate_state(int)));
	connect(this, SIGNAL(signal_confirm_relocation()), relocateProcesser_, SLOT(slot_confirm_relocation()));
	connect(relocateProcesser_, SIGNAL(finished()), this, SLOT(slot_release_thread()));
}

void DLRelocationCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	relocateProcesser_ = NULL;
	isPress = true;
	QPointF pos = event->scenePos();
	start_point_ = pos;
}


void DLRelocationCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	QPointF pos = event->scenePos();
	end_point_ = pos;
	
	if(isPress){
		if(relocate_item_ == NULL){
			relocate_item_ = new DLRelocateItem;
			relocate_item_->setPos(start_point_);
			scene()->addItem(relocate_item_);
		}
		else{
			relocate_item_->updateItem(start_point_, end_point_);
			relocate_item_->setRotation(QLineF(start_point_, end_point_).angleTo(QLineF(0, 0, 0, 100)));
		}
	}	
}


void DLRelocationCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    end_point_ = event->scenePos();
	scene()->removeItem(relocate_item_);
	point_angle = QLineF(start_point_, end_point_).angleTo(QLineF(QPointF(0, 0), QPointF(100, 0)));
	point_angle = point_angle * (3.1415926) / 180;

	if(((DLCustomScene*)scene_)->RobotItem() == NULL)
	{
		isPress = false;
		return;
	}

	//如果线程处理对象不为空不让其再次进行重定位
	if (relocateProcesser_ != NULL){
		isPress = false;
		return;
	}
	RobotRelcation(start_point_, point_angle);

	isPress = false;
	 
}


void DLRelocationCommand::slot_release_thread()
{
	
	//if(relocateProcesser_->isFinished())
	//{
	//	delete relocateProcesser_;
	//	relocateProcesser_ = NULL;
	//}
	//else{
	//	relocateProcesser_->stopThread();
	//}

}


void DLRelocationCommand::slot_get_relocate_state(int state)
{
	if (-1 == state) {
		relocate_tip_label_->setText(QStringLiteral("重定位失败，请调整机器人位置"));
	}
	if(0 == state) {
		relocate_tip_label_->setText(QStringLiteral("重定位失败"));
	}
	else if(1 == state) {
		relocate_tip_label_->setText(QStringLiteral("重定位成功"));
		return;
	}
	else if(2 == state) {
		relocate_tip_label_->setText(QStringLiteral("重定位中"));
	}
	else if(3 == state) {
		relocate_tip_label_->setText(QStringLiteral("重定位完成"));
		//relocateProcesser_->lockThread();
	}
	else {
		std::cout << "other state...." << std::endl;
	}

	confirm_relocate_button_->show();
}


void DLRelocationCommand::slot_confirm_relocate()
{
	relocate_tip_widget_->hide();

	emit signal_confirm_relocation();
	((DLCustomScene*)scene_)->SetRobotItemPropertry(start_point_.x()/100, start_point_.y()/100, point_angle);
	((DLCustomScene*)scene_)->SetOperateType(DLOperator::TYPE_MOVE);
}


int DLRelocationCommand::getRelocateRetVal() 
{
    return ((DLCustomScene*)scene())->GetRelocateRetVal();
}


/////////////////////////////////////////////////////////


DLCoordinateTransformCommand::DLCoordinateTransformCommand(QGraphicsScene *scene)
	: DLUndoCommandInterface(scene)
{
	//isPress_ = false;
}


void DLCoordinateTransformCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	
	//isPress_ = true;
	//start_line_.setP2(event->scenePos());
	//start_line_.setP1(QPointF(0, 0));
	//start_line_.setP2(event->scenePos());

	//end_line_.setP1(QPointF(0, 0));
}


void DLCoordinateTransformCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	//if (isPress_) {
	//	if (isPress_ == true && ((DLCustomScene*)scene_)->button() == Qt::RightButton) {
	//		end_line_.setP2(event->scenePos());

	//		double degree_angle = end_line_.angleTo(start_line_);
	//		double rad_angle = degree_angle * 3.14159 / 180.0;
	//		((DLCustomScene*)scene())->set_transform(degree_angle);
	//	}

	//}
	//
}


void DLCoordinateTransformCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	//isPress_ = false;
}


/////////////////////////////////////////////////////////


DLAddDeviceCommand::DLAddDeviceCommand(QGraphicsScene *scene)
: DLUndoCommandInterface(scene)
{
}


void DLAddDeviceCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	
}


void DLAddDeviceCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	
}


void DLAddDeviceCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{

	//if(event->button()==Qt::LeftButton)
	//{
	//	QPointF pos=event->scenePos();

	//	HShape::Ellipse pro;
	//	pro.center_.x_=pos.x();
	//	pro.center_.y_=pos.y();
	//	pro.width_=10;
	//	pro.height_=10;
	//	pro.angle_=0;

	//	//std::cout << "x = " << pos.x() << " y = " << pos.y() << std::endl;
	//	DLCustomScene *s=(DLCustomScene*)scene();
	//	QGraphicsObject *item_ = new DLDeviceItem(pro);
	//	item_->setToolTip(QObject::tr("device"));
	//	s->add_deviceItem(item_);
	//	QObject::connect(item_, SIGNAL(sig_edit(FPatrolInfo&)), scene(), SIGNAL(sig_scene_edit(FPatrolInfo&)));
	//	QObject::connect(scene(), SIGNAL(sig_info_ok()), item_, SLOT(slot_info_ok()));
	//}

}


/////////////////////////////////////////////////////////


DLAddArcCommand::DLAddArcCommand(QGraphicsScene *scene)
: DLUndoCommandInterface(scene)
{
	item_ = NULL;
}


void DLAddArcCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{

}


void DLAddArcCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{

}


void DLAddArcCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{

	if(event->button()==Qt::LeftButton)
	{
		//QPointF pos=event->scenePos();

		//Segment pro;
		//pro.start_.x-=pos.x();
		//pro.start_.y_=pos.y();
		//pro.end_.x-=10;
		//pro.height_=10;
		//pro.angle_=0;


		//DLCustomScene *s=(DLCustomScene*)scene();
		//item_ = new HQShapeArcItem(pro);
		//scene()->addItem(item_);
		//s->add_deviceItem(item_);
	}

}


/////////////////////////////////////////////////////////


DLAddEdgeCommand::DLAddEdgeCommand(QGraphicsScene *scene):DLUndoCommandInterface(scene)
{
	//printf("DLAddEdgeCommand::DLAddEdgeCommand\n");
	virtual_item_=NULL;
}


bool DLAddEdgeCommand::is_useful()
{
	if(bezier_list_.isEmpty()){
		return false;
	}
	else{
		return true;
	}
}


void DLAddEdgeCommand::undo()
{
	printf("DLAddEdgeCommand::undo\n");
	foreach(QGraphicsItem *item,bezier_list_){
		//scene_->removeItem((QGraphicsItem*)item);
		//scene_->update();
		DLCustomScene *s=(DLCustomScene*)scene();
		s->remove_advancedcurve(((DLBezierItem*)item)->id());
	}

}


void DLAddEdgeCommand::redo()
{

	printf("DLAddEdgeCommand::redo\n");
	foreach(QGraphicsItem*item,bezier_list_){
		//scene_->addItem((QGraphicsItem*)item);
		//scene_->update();
		DLCustomScene *s=(DLCustomScene*)scene();
		//int id = ((DLBezierItem*)item)->id();
		s->add_advancedCurve(item);

	}

}


void DLAddEdgeCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{

	printf("DLAddEdgeCommand::mousePressEvent\n");

	//QList<QGraphicsView*> view_list=scene_->views();

	if(event->button()==Qt::LeftButton)
	{
		start_point_=event->scenePos();
		end_point_=event->scenePos();

		virtual_item_=new QGraphicsLineItem(QLineF(start_point_,end_point_));
		scene()->addItem(virtual_item_);

	}


}
	

void DLAddEdgeCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{

	if(virtual_item_!=NULL)
	{

		end_point_=event->scenePos();
		virtual_item_->setLine(QLineF(start_point_,end_point_));

		DLCustomScene *s=(DLCustomScene*)scene();
		QGraphicsItem *start_item=s->FindItem(start_point_,HShape::LANDMARKITEM);
		QGraphicsItem *end_item=s->FindItem(end_point_, HShape::LANDMARKITEM);

		if((start_item!=NULL)&&(end_item!=NULL))
		{
			if(start_item!=end_item)
			{

				DLCustomScene *s=(DLCustomScene*)scene();
				DLBezierItem *item=new DLBezierItem((DLShapeItem*)start_item,(DLShapeItem*)end_item,s->GetBezierId()+1);
				bezier_list_.push_back(item);

				{
					start_point_=start_item->scenePos();
					end_point_=end_item->scenePos();
					virtual_item_->setLine(QLineF(start_item->scenePos(),end_item->scenePos()));	
					virtual_list_.push_back(virtual_item_);

					start_point_=end_point_;
					virtual_item_=new QGraphicsLineItem(QLineF(start_point_,end_point_));
					scene()->addItem(virtual_item_);
				}
			}
		}
	}


}


void DLAddEdgeCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{

	printf("DLAddEdgeCommand::mouseReleaseEvent\n");

	if(event->button()==Qt::LeftButton)
	{
		end_point_=event->scenePos();
		if(virtual_item_!=NULL)
		{

			DLCustomScene *s=(DLCustomScene*)scene();
			QGraphicsItem *start_item=s->FindItem(start_point_, HShape::LANDMARKITEM);
			QGraphicsItem *end_item=s->FindItem(end_point_, HShape::LANDMARKITEM);

			if((start_item!=NULL)&&(end_item!=NULL))
			{
				int start_id = ((DLLandmarkItem*)start_item)->id();
				int end_id = ((DLLandmarkItem*)end_item)->id();
				int id = (start_id << 16) + end_id;

				if(start_item!=end_item)
				{
					
					DLCustomScene *s=(DLCustomScene*)scene();
					DLBezierItem *item=new DLBezierItem((DLShapeItem*)start_item,(DLShapeItem*)end_item, id);

					bezier_list_.push_back(item);
					start_point_=end_point_;

				}

			}

			//清除虚拟线
			foreach(QGraphicsItem*item,virtual_list_)
			{
				scene()->removeItem(item);
				delete item;
				item=NULL;
			}

			{
				scene()->removeItem(virtual_item_);
				delete virtual_item_;
				virtual_item_=NULL;	
			}
		}
	}
	

}


/////////////////////////////////////////////////////////


DLSelectCommand::DLSelectCommand(QGraphicsScene *scene):DLUndoCommandInterface(scene)
{

	rect_item_=NULL;

}


void DLSelectCommand::undo()
{


}


void DLSelectCommand::redo()
{


}


bool DLSelectCommand::is_useful()
{
	return true;
}


void DLSelectCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		QList<QGraphicsItem*> item_list = scene()->items(event->scenePos());
		topleft_pos_ = event->scenePos();

		if ((item_list.size()<1) || ((item_list.size() == 1) && (item_list[0]->type() == DLShapeItem::TYPE_MULTI_POINT)))
		{ 
			rect_item_ = new QGraphicsRectItem(QRectF(event->scenePos(), QSizeF(0, 0)).normalized());
			rect_item_->setPen(Qt::NoPen);
			rect_item_->setOpacity(0.5);
			rect_item_->setBrush(QColor(133, 177, 222));
			rect_item_->setZValue(50);
			scene()->addItem(rect_item_);
		}
	}

}


void DLSelectCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	QPointF p=event->scenePos()-topleft_pos_;
	
	QRectF rect(topleft_pos_,event->scenePos());
	if(rect_item_!=NULL){
		rect_item_->setRect(rect);
	}

}


void DLSelectCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	QRectF rect;
	if (rect_item_ !=  NULL) {
		rect = rect_item_->rect();
	}

// 	DLCustomScene *pScene = (DLCustomScene*)scene();
// 	if (pScene != NULL) {
// 		std::map<int, QPointF> points;
// 		pScene->GetNormalPosList(points);
// 		
// 		for (auto it = points.begin(); it != points.end();) {
// 			if (rect.contains(it->second))
// 			{
// 				it++;
// 			}
// 			else {
// 				points.erase(it++);
// 			}
// 		}
// 		pScene->SetMutilPointItem(points);
		//pScene->SetMutilSelectRect(rect);
//	}

	if(rect_item_!=NULL){
		QList<QGraphicsItem*> items=rect_item_->collidingItems();
		foreach(QGraphicsItem*item,items){
			item->setSelected(true);
		}
		delete rect_item_;
		rect_item_=NULL;
	}



}


/////////////////////////////////////////////////////////


DLDeleteCommand::DLDeleteCommand(QGraphicsScene *scene):DLUndoCommandInterface(scene)
{

	QList<QGraphicsItem*> item_list=scene_->items();
	foreach(QGraphicsItem*item,item_list){
		if(item->type()==HShape::LANDMARKITEM){
			item_list_.push_back(item);
		}
		else if(item->type()== HShape::BEZIERITEM){
			item_list_.push_back(item);
		}
	}


}


void DLDeleteCommand::undo()
{

	DLCustomScene *s=(DLCustomScene*)scene();
	foreach(QGraphicsItem*item,item_list_){
		if(item->type()== HShape::LANDMARKITEM){
			s->add_advancedPoint(item);
		}
		else if(item->type()== HShape::BEZIERITEM){
			s->add_advancedCurve(item);
		}
	}
}


void DLDeleteCommand::redo()
{

	//remove
	DLCustomScene *s=(DLCustomScene*)scene();
	foreach(QGraphicsItem *item,item_list_){
		
		if(item->type()== HShape::LANDMARKITEM){
			s->remove_advancedpoint(((DLLandmarkItem*)item)->id());
		}
		else if(item->type()== HShape::BEZIERITEM){
			s->remove_advancedcurve(((DLLandmarkItem*)item)->id());
		}
	}

}


bool DLDeleteCommand::is_useful()
{
	if(item_list_.isEmpty()){
		return false;
	}
	else{
		return true;
	}
}


void DLDeleteCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{

}


void DLDeleteCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{

}


void DLDeleteCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{

}


//void DLDeleteCommand::keyPressEvent(QKeyEvent *event)
//{
//	if (event->key() == Qt::Key_Delete)
//	{
//		foreach(QGraphicsItem *item, item_list_) {
//			int type = item->type();
//			if (type == HShape::LANDMARKITEM) {
//				item_list_.removeOne(item);
//				scene()->removeItem(item);
//			}
//		}
//	}
//}


/////////////////////////////////////////////////////////


DLAddPictureCommand::DLAddPictureCommand( QGraphicsScene *scene )
: DLUndoCommandInterface(scene)
{
	item_=NULL;
}


void DLAddPictureCommand::undo()
{
	//if((scene()!=NULL)&&(item_!=NULL))
	//{
	//	scene()->removeItem(item_);
	//}
}


void DLAddPictureCommand::redo()
{

	//if((scene()!=NULL)&&(item_!=NULL))
	//{
	//	scene()->addItem(item_);
	//}

}


void DLAddPictureCommand::mousePressEvent( QGraphicsSceneMouseEvent *event )
{

}


void DLAddPictureCommand::mouseMoveEvent( QGraphicsSceneMouseEvent *event )
{

}


void DLAddPictureCommand::mouseReleaseEvent( QGraphicsSceneMouseEvent *event )
{

	//if(event->button()==Qt::LeftButton)
	//{
	//	QPointF pos=event->scenePos();

	//	HShape::Rectangle pro;
	//	pro.center_.x_=pos.x();
	//	pro.center_.y_=pos.y();
	//	pro.width_=20;
	//	pro.height_=20;

	//	DLCustomScene *s=(DLCustomScene*)scene();
	//	item_ = new DLPictureItem(pro);

	//}

}


bool DLAddPictureCommand::is_useful()
{
	//if(item_!=NULL){
	//	return true;
	//}
	//else{
		return false;
	//}
}


/////////////////////////////////////////////////////////


DLAddLineCommand::DLAddLineCommand(QGraphicsScene *scene, int type)
	: DLUndoCommandInterface(scene)
{
	item_ = NULL;
	type_ = type;
}


void DLAddLineCommand::undo()
{
	if ((scene() != NULL) && (item_ != NULL))
	{
		DLCustomScene *s = (DLCustomScene*)scene();

	}
}


void DLAddLineCommand::redo()	
{
	if ((scene() != NULL) && (item_ != NULL))
	{
		DLCustomScene *s = (DLCustomScene*)scene();
		s->add_normalLine(item_);

	}
}


bool DLAddLineCommand::is_useful()
{
	return true;
}


void DLAddLineCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		start_point_ = event->scenePos();
		end_point_ = event->scenePos();

		virtual_item_ = new QGraphicsLineItem(QLineF(start_point_, end_point_));
		scene()->addItem(virtual_item_);
	}
}


void DLAddLineCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	if (virtual_item_ != NULL)
	{
		end_point_ = event->scenePos();
		virtual_item_->setLine(QLineF(start_point_, end_point_));
	}
}


void DLAddLineCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		end_point_ = event->scenePos();
		HShape::Segment pro;
		pro.start_.x_ = start_point_.x();
		pro.start_.y_ = start_point_.y();
		pro.end_.x_ = end_point_.x();
		pro.end_.y_ = end_point_.y();

		DLSegmentItem::LineProperty type = (DLSegmentItem::LineProperty)type_;
		item_ = new DLSegmentItem(pro, 0, type);

		scene()->removeItem(virtual_item_);
		delete virtual_item_;
		virtual_item_ = NULL;
	}
}


/////////////////////////////////////////////////////////


DLAddPolygonCommand::DLAddPolygonCommand(QGraphicsScene *scene)
	: DLUndoCommandInterface(scene)
{
	item_ = NULL;;
}


void DLAddPolygonCommand::undo()
{
	if ((scene() != NULL) && (item_ != NULL))
	{
		DLCustomScene *s = (DLCustomScene*)scene();

	}
}


void DLAddPolygonCommand::redo()
{
	if ((scene() != NULL) && (item_ != NULL))
	{
		DLCustomScene *s = (DLCustomScene*)scene();
		s->add_advancedArea(item_);

	}
}


bool DLAddPolygonCommand::is_useful()
{
	return true;
}


void DLAddPolygonCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		topLeft_point_ = event->scenePos();
		bottomRight_point_ = event->scenePos();

		virtual_item_ = new QGraphicsRectItem();
		scene()->addItem(virtual_item_);
	}
}


void DLAddPolygonCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	if (virtual_item_ != NULL)
	{
		bottomRight_point_ = event->scenePos();
		virtual_item_->setRect(QRectF(topLeft_point_, bottomRight_point_));
	}
}


void DLAddPolygonCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		bottomRight_point_ = event->scenePos();
		QLineF line(topLeft_point_, bottomRight_point_);
		if (line.length() > 14) {
			//QVector<QPointF> points;
			//points.push_back(QPointF(topLeft_point_.x(), bottomRight_point_.y()));		//左下
			//points.push_back(topLeft_point_);											// 左上
			//points.push_back(QPointF(bottomRight_point_.x(), topLeft_point_.y()));		// 右上
			//points.push_back(bottomRight_point_);	//右下

			HShape::Rectangle pro;
			pro.center_.x_ = (topLeft_point_.x() + bottomRight_point_.x()) / 2;
			pro.center_.y_ = (topLeft_point_.y() + bottomRight_point_.y()) / 2;
			pro.width_ = abs(topLeft_point_.x() - bottomRight_point_.x());
			pro.height_ = abs(topLeft_point_.y() - bottomRight_point_.y());
			DLCustomScene *s = (DLCustomScene*)scene();
			item_ = new DLAdvancedAreaItem(pro, s->GetAdvancedAreaId()+1, DLAdvancedAreaItem::Forbidden);

			//设置高级区域的线不可通行
			QRectF rect(topLeft_point_, bottomRight_point_);
			if (NULL != s)
			{
				QList<QGraphicsItem *> pItems = s->items(rect);
				for (int index = 0; index < pItems.size(); ++index)
				{
					DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(pItems[index]);
					if (NULL != pBezierItem)
					{
						s->AddAdvanceAreaItemMap(item_, pBezierItem);    //添加到高级区域映射中
						//pBezierItem->setPassing(false);
						pBezierItem->SetIsAdvanceArea(true);
					}
				}
			}

			s->SetOperateType(DLOperator::TYPE_SELECT);
			emit s->ChangeTypeSignal(tr("选择"), DLOperator::TYPE_SELECT);
		}
		

		scene()->removeItem(virtual_item_);
		delete virtual_item_;
		virtual_item_ = NULL;

	}
}


/////////////////////////////////////////////////////////


DLAddDevAreaCommand::DLAddDevAreaCommand(QGraphicsScene *scene)
	: DLUndoCommandInterface(scene)
{
	item_ = NULL;
}


void DLAddDevAreaCommand::undo()
{
	if ((scene() != NULL) && (item_ != NULL))
	{
		DLCustomScene *s = (DLCustomScene*)scene();

	}
}


void DLAddDevAreaCommand::redo()
{
	if ((scene() != NULL) && (item_ != NULL))
	{
		DLCustomScene *s = (DLCustomScene*)scene();
		s->add_deviceArea(item_);

	}
}


bool DLAddDevAreaCommand::is_useful()
{
	return true;
}


void DLAddDevAreaCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		topLeft_point_ = event->scenePos();
		bottomRight_point_ = event->scenePos();

		virtual_item_ = new QGraphicsRectItem();
		scene()->addItem(virtual_item_);
	}
}


void DLAddDevAreaCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	if (virtual_item_ != NULL)
	{
		bottomRight_point_ = event->scenePos();
		virtual_item_->setRect(QRectF(topLeft_point_, bottomRight_point_));
	}
}


void DLAddDevAreaCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		bottomRight_point_ = event->scenePos();
		QLineF line(topLeft_point_, bottomRight_point_);
		if (line.length() > 14) {
			QPointF offset = bottomRight_point_ - topLeft_point_;
			HShape::Rectangle pro;
			pro.center_.x_ = (bottomRight_point_.x() + topLeft_point_.x()) / 2;
			pro.center_.y_ = (bottomRight_point_.y() + topLeft_point_.y()) / 2;
			pro.width_ = abs(offset.x());
			pro.height_ = abs(offset.y());

			item_ = new DLDeviceAreaItem(pro, 0, DLDeviceAreaItem::DeviceArea);
		}

		scene()->removeItem(virtual_item_);
		delete virtual_item_;
		virtual_item_ = NULL;
	}
}


/////////////////////////////////////////////////////////


DLAddSTationAreaCommand::DLAddSTationAreaCommand(QGraphicsScene *scene)
	: DLUndoCommandInterface(scene)
{
	item_ = NULL;
}


void DLAddSTationAreaCommand::undo()
{
	if ((scene() != NULL) && (item_ != NULL))
	{
		DLCustomScene *s = (DLCustomScene*)scene();

	}
}


void DLAddSTationAreaCommand::redo()
{
	if ((scene() != NULL) && (item_ != NULL))
	{
		DLCustomScene *s = (DLCustomScene*)scene();
		s->add_deviceArea(item_);

	}
}


bool DLAddSTationAreaCommand::is_useful()
{
	return true;
}


void DLAddSTationAreaCommand::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		topLeft_point_ = event->scenePos();
		bottomRight_point_ = event->scenePos();

		virtual_item_ = new QGraphicsRectItem();
		scene()->addItem(virtual_item_);
	}
}


void DLAddSTationAreaCommand::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	if (virtual_item_ != NULL)
	{
		bottomRight_point_ = event->scenePos();
		virtual_item_->setRect(QRectF(topLeft_point_, bottomRight_point_));
	}
}


void DLAddSTationAreaCommand::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	if (event->button() == Qt::LeftButton)
	{
		bottomRight_point_ = event->scenePos();
		QLineF line(topLeft_point_, bottomRight_point_);
		if (line.length() > 14) {
			QPointF offset = bottomRight_point_ - topLeft_point_;
			HShape::Rectangle pro;
			pro.center_.x_ = (bottomRight_point_.x() + topLeft_point_.x()) / 2;
			pro.center_.y_ = (bottomRight_point_.y() + topLeft_point_.y()) / 2;
			pro.width_ = abs(offset.x());
			pro.height_ = abs(offset.y());

			item_ = new DLDeviceAreaItem(pro, 0, DLDeviceAreaItem::Station);
		}

		scene()->removeItem(virtual_item_);
		delete virtual_item_;
		virtual_item_ = NULL;
	}
}



