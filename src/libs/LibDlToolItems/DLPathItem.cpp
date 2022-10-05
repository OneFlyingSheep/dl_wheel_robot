#include "DLPathItem.h"
#include "DLLandmarkItem.h"
#include "DLBezierItem.h"
#include <QPainter>
#include <QDebug>
#include "common/DLRobotCommonDef.h"

DLPathItem::DLPathItem()
{
	m_bIsWork = false;
	patrol_point_index_ = 0;
	path_width_ = 10;
	percentage_ = 0;
	currentPath_start_index_ = 0;
	currentPath_end_index_ = 0;
	setFlag(QGraphicsItem::ItemIsMovable, false);
	setFlag(QGraphicsItem::ItemIsSelectable, false);
	this->setZValue(20);
}


QGraphicsItem* DLPathItem::find_bezier(int start_id, int end_id)
{
	if (m_lstBeziers.size() <= 0) return NULL;

	std::map<int, QGraphicsItem*>::iterator it = m_lstBeziers.begin();
	while (it != m_lstBeziers.end())
	{
		DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(it->second);
		if (NULL != pBezierItem && ((pBezierItem->start_id() == start_id && pBezierItem->end_id() == end_id) || (pBezierItem->end_id() == start_id && pBezierItem->start_id() == end_id)))
		{
			return pBezierItem;
		}
		++it;

	}
	return NULL;




	
	//巡检路线可能是反向的，搜寻两次确保一定找到

// 	int id = (start_id << 16) + end_id;//潜规则
 //	std::map<int, QGraphicsItem*>::iterator it = bezier_list_.find(id);
// 	if (it != bezier_list_.end() )
// 	{
// 		//DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(it->second);
// 		//if (pBezierItem->start_id() == start_id && pBezierItem->end_id() == end_id)
// 		{
// 			return it->second;
// 		}
// 	}
// 
// 	id += 1;
// 	it = bezier_list_.find(id);
// 	if (it != bezier_list_.end())
// 	{
// // 		DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(it->second);
// // 		if (pBezierItem->start_id() == start_id && pBezierItem->end_id() == end_id)
// // 		{
// 			return it->second;
// 	//	}
// 	}
// 
// 
// 	int _id = (end_id << 16) + start_id;//潜规则.
// 	std::map<int, QGraphicsItem*>::iterator _it = bezier_list_.find(_id);
// 	if (_it != bezier_list_.end()) 
// 	{
// // 		DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(_it->second);
// // 		if (pBezierItem->start_id() == start_id && pBezierItem->end_id() == end_id)
// // 		{
// 			return _it->second;
// 		//}
// 	}
// 
// 	id += 1;//潜规则.
// 	_it = bezier_list_.find(_id);
// 	if (_it != bezier_list_.end())
// 	{
// // 		DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(_it->second);
// // 		if (pBezierItem->start_id() == start_id && pBezierItem->end_id() == end_id)
// // 		{
// 			return _it->second;
// 		//}
// 	}


	return NULL;
}


void DLPathItem::initPath(std::map<int, QGraphicsItem*> &bezier_list)
{
	m_lstBeziers = bezier_list;
	auto iter = bezier_list.begin();
	while (iter != bezier_list.end()) 
	{
		int start_index = ((DLBezierItem*)iter->second)->start_id();
		int end_index = ((DLBezierItem*)iter->second)->end_id();
		QPoint index(start_index, end_index);
		m_lstInvalidLines.push_back(index);
		iter++;
	}
	UpdatePath(0, 0, 0, 0);
}


void DLPathItem::initUpdateRect(const QRectF &rect)
{
	update_rect_ = rect;
}


QRectF DLPathItem::boundingRect() const
{

	return shape().boundingRect();

}


QPainterPath DLPathItem::shape() const
{

	QPainterPath path;
	path.addRect(update_rect_);
	QPainterPathStroker path_stroker;

	path_stroker.setWidth(path_width_);
	return path_stroker.createStroke(path);

}


void DLPathItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
	painter->setRenderHint(QPainter::Antialiasing, true);
	QPen pen(Qt::blue, path_width_, Qt::SolidLine);

	
	
	//绘制巡检路径，最底层
	painter->save();
	pen.setStyle(Qt::SolidLine);
	pen.setWidth(path_width_);
	pen.setBrush(Qt::lightGray);
	painter->setPen(pen);
	for (int i = 0; i < m_lstInvalidBezierPaths.size(); ++i) {
		painter->drawPath(m_lstInvalidBezierPaths[i]);
	}
	painter->restore();

	/////////////分别绘制完成、未完成和非工作路径/////////////////
	painter->save();
	pen.setStyle(Qt::SolidLine);
	pen.setWidth(path_width_);
	pen.setBrush(Qt::red);
	painter->setPen(pen);
	for (int i = 0; i < m_lstUnfinishedBezierPaths.size(); ++i) {
		painter->drawPath(m_lstUnfinishedBezierPaths[i]);
	}
	painter->restore();

	if (percentage_ < 1 && m_bIsWork) {

		//获取当前路径的path的属性,并绘制当前路径
		//QPointF startPos, c1Pos, c2Pos, endPos;
		QPointF startPos, endPos;
		DLBezierItem *bezier_item = (DLBezierItem*)find_bezier(currentPath_start_index_, currentPath_end_index_);
		if (bezier_item != NULL) {

			int bezier_start_index = ((DLBezierItem*)bezier_item)->start_id();
			int bezier_end_index = ((DLBezierItem*)bezier_item)->end_id();

			if (currentPath_start_index_ == bezier_start_index && currentPath_end_index_ == bezier_end_index) {
				startPos = ((DLBezierItem*)bezier_item)->start_pos();
				//c1Pos = ((DLBezierItem*)bezier_item)->c1_pos();
				//c2Pos = ((DLBezierItem*)bezier_item)->c2_pos();
				endPos = ((DLBezierItem*)bezier_item)->end_pos();
			}
			else {
				endPos = ((DLBezierItem*)bezier_item)->start_pos();
				//c2Pos = ((DLBezierItem*)bezier_item)->c1_pos();
				///c1Pos = ((DLBezierItem*)bezier_item)->c2_pos();
				startPos = ((DLBezierItem*)bezier_item)->end_pos();
			}

			QPointF point = current_bezier_path_.pointAtPercent(percentage_);
			QLineF line_1, line_2;
			line_1.setP1(startPos);
			line_1.setP2(point);
			line_2.setP1(point);
			line_2.setP2(endPos);

			painter->save();
			painter->setPen(QPen(Qt::blue, path_width_, Qt::SolidLine));
			painter->drawLine(line_1);
			painter->restore();

			painter->save();
			painter->setPen(QPen(Qt::red, path_width_, Qt::SolidLine));
			painter->drawLine(line_2);
			painter->restore();
		}
	}

	painter->save();
	pen.setBrush(Qt::blue);
	painter->setPen(pen);
	for (int i = 0; i < m_lstFinishedBezierPaths.size(); ++i) {
		painter->drawPath(m_lstFinishedBezierPaths[i]);
			}
	painter->restore();


}


void DLPathItem::SetWorkPath(std::vector<QPoint> vWorkPaths, std::vector<int> patrol_points, bool is_work)
{
	ROS_INFO("+++++++++++is work:%d", is_work);
	ROS_INFO("+++++++++work path begin:");

	std::vector<QPoint>::iterator it = vWorkPaths.begin();
	while (it != vWorkPaths.end())
	{
		ROS_INFO("start:%d, end:%d", it->x(), it->y());
		++it;
	}
	ROS_INFO("+++++++++work path end");

	ROS_INFO("+++++++++Points begin:");
	std::vector<int>::iterator itInt = patrol_points.begin();
	while (itInt != patrol_points.end())
	{
		ROS_INFO("index:%d", *itInt);
		++itInt;
	}
	ROS_INFO("+++++++++Points end");

	//qDebug() << "+++++++++work path:" << vWorkPaths;
	//qDebug() << "+++++++++points:" << patrol_points;

	m_iBeginIndex = 0;

	m_bIsWork = is_work;
	m_vPatrolPoints.clear();
	m_lstWorkPath.clear();
	m_vFinishedPath.clear();
	m_vUnfinishedPath.clear();
	m_lstInvalidLines.clear();
	currentPath_start_index_ = 0;
	currentPath_end_index_ = 0;
	patrol_point_index_ = 0;

	m_lstWorkPath = vWorkPaths;
	m_vPatrolPoints = patrol_points;

	for (int i = 0; i < vWorkPaths.size(); i++)
	{
		QGraphicsItem *item = find_bezier(vWorkPaths[i].x(), vWorkPaths[i].y());
		if ( NULL != item){
			m_vUnfinishedPath.push_back(vWorkPaths[i]);
		}
	}
	for(auto it = m_lstBeziers.begin(); it != m_lstBeziers.end(); it++)
	{
		bool isExist = false;
		int start_index = ((DLBezierItem*)it->second)->start_id();
		int end_index = ((DLBezierItem*)it->second)->end_id();
		
		for (int i = 0; i < m_vUnfinishedPath.size(); i++)
		{
			//如果存在就跳出循环
			if ( (start_index == m_vUnfinishedPath[i].x() && end_index == m_vUnfinishedPath[i].y()) || (start_index == m_vUnfinishedPath[i].y() && end_index == m_vUnfinishedPath[i].x()) )
			{
				isExist = true;
				break;
			}
		}
		//if (!isExist) {
			m_lstInvalidLines.push_back(QPoint(start_index, end_index));
		//}
	}
	UpdatePath(0, 0, 0, 0);
}


QPainterPath DLPathItem::cal_bezier_path(QGraphicsItem *bezier_item, int start_index, int end_index)
{
	QPainterPath path;

	//QPointF startPos, c1Pos, c2Pos, endPos;
	QPointF startPos, endPos;
	if (bezier_item != NULL) {

		int bezier_start_index = ((DLBezierItem*)bezier_item)->start_id();
		int bezier_end_index = ((DLBezierItem*)bezier_item)->end_id();

		if (start_index == bezier_start_index && end_index == bezier_end_index) {
			startPos = ((DLBezierItem*)bezier_item)->start_pos();
			//c1Pos = ((DLBezierItem*)bezier_item)->c1_pos();
			//c2Pos = ((DLBezierItem*)bezier_item)->c2_pos();
			endPos = ((DLBezierItem*)bezier_item)->end_pos();
		}
		else {
			endPos = ((DLBezierItem*)bezier_item)->start_pos();
			//c2Pos = ((DLBezierItem*)bezier_item)->c1_pos();
			//c1Pos = ((DLBezierItem*)bezier_item)->c2_pos();
			startPos = ((DLBezierItem*)bezier_item)->end_pos();
		}
	}
	path.moveTo(startPos);
	//path.cubicTo(QPointF(c1Pos), QPointF(c2Pos), QPointF(endPos));
	path.lineTo(QPointF(endPos));
	return path;
}


std::vector<QPoint> DLPathItem::cal_unfinished_path(int start_index, int end_index)
{

	std::vector<QPoint> work_paths;
	if (m_vPatrolPoints.size() < 2 || abs(start_index) > m_vPatrolPoints.size() || abs(end_index) > m_vPatrolPoints.size()) {
		return work_paths;
	}
	else {
		for (int i = start_index; i < end_index; ++i)
		{
			//当迭代到倒数第二个数值时候退出循环
			if (i < end_index - 1) {
				int start_index = m_vPatrolPoints[i];
				int end_index = m_vPatrolPoints[i + 1];
				QPoint path_index(start_index, end_index);
				work_paths.push_back(path_index);
			}

		}
		return work_paths;
	}

}


std::vector<QPoint> DLPathItem::cal_finished_path(int start_index, int end_index)
{
	std::vector<QPoint> work_paths;
	if (m_vPatrolPoints.size() < 2 || abs(start_index) > m_vPatrolPoints.size() || abs(end_index) > m_vPatrolPoints.size()) {
		return work_paths;
	}
	else {
		for (int i = start_index; i < end_index; ++i)
		{
			if (i <= end_index - 1)
			{
				int start_index = m_vPatrolPoints[i];
				int end_index = m_vPatrolPoints[i + 1];
				QPoint path_index(start_index, end_index);
				work_paths.push_back(path_index);
			}
		}
		return work_paths;
	}

}


void DLPathItem::UpdatePath(int start_index, int end_index, double percentage, int patrol_task_index)
{
	//qDebug() << "start index:" << start_index << "; end index: " << end_index << ";percentage:" << percentage << "; patrol task index:" << patrol_task_index;
	
	ROS_INFO("++++++++start index:%d", start_index);
	ROS_INFO("++++++++end index:%d", end_index);
	ROS_INFO("++++++++percentage:%lf", percentage);
	ROS_INFO("+++++++patrol_task_index:%d", patrol_task_index);

	//初始化路径和清空路径
	m_lstFinishedBezierPaths.clear();
	m_lstUnfinishedBezierPaths.clear();
	m_lstInvalidBezierPaths.clear();

	QPainterPath path;
	finished_bezier_path_ = path;
	unfinished_bezier_path_ = path;
	current_bezier_path_ = path;

	ROS_INFO("+++++++++++patrol points begin");
	for (int index= 0; index < m_vPatrolPoints.size(); ++index)
	{
		ROS_INFO("+++++++point:%d", m_vPatrolPoints[index]);
	}
	ROS_INFO("+++++++++++patrol points end");
	//qDebug() << "+++++++++++patrol points:" << m_vPatrolPoints;
	currentPath_start_index_ = start_index;
	///////////////calculate path index////////////
	if (start_index == end_index) {
		m_vUnfinishedPath = cal_unfinished_path(0, m_vPatrolPoints.size());
	}
	else {
		//m_vFinishedPath = cal_finished_path(0, patrol_task_index);

		//找到完成的路径索引号
 		int iEndIndex = 0;
 		for (iEndIndex; iEndIndex < m_vPatrolPoints.size(); ++iEndIndex)
 		{
 			if (percentage > 0.99)
 			{
 				if (end_index == m_vPatrolPoints[iEndIndex] && iEndIndex >= m_iBeginIndex)
 				{
					m_iBeginIndex = iEndIndex;
					currentPath_start_index_ = end_index;
 					break;
 				}
 			}
 			else
 			{
 				if (start_index == m_vPatrolPoints[iEndIndex] && iEndIndex >= m_iBeginIndex)
 				{
					m_iBeginIndex = iEndIndex;
					currentPath_start_index_ = start_index;
 					break;
 				}
 			}
 			
 		}
 
		ROS_INFO("+++++++end index:%d", iEndIndex);
 		if (iEndIndex != m_vPatrolPoints.size())
 		{//找到
			ROS_INFO("+++++++point:%d", m_vPatrolPoints[iEndIndex]);
			m_vFinishedPath = cal_finished_path(0, iEndIndex);
			m_vUnfinishedPath = cal_unfinished_path(iEndIndex, m_vPatrolPoints.size());
		}

		//m_vUnfinishedPath = cal_unfinished_path(patrol_task_index, m_vPatrolPoints.size());
	}

	///////////////calculate path////////////
	{
		QGraphicsItem *item = find_bezier(start_index, end_index);
		if (item != NULL) {
			current_bezier_path_ = cal_bezier_path(item, start_index, end_index);
		}
	}
	

	for (int i = 0; i < m_vFinishedPath.size(); ++i)
	{
		QGraphicsItem *item = find_bezier(m_vFinishedPath[i].x(), m_vFinishedPath[i].y());
		if (item != NULL) {
			QPainterPath finished_bezier_path = cal_bezier_path(item, m_vFinishedPath[i].x(), m_vFinishedPath[i].y());
			m_lstFinishedBezierPaths.push_back(finished_bezier_path);
		}
	}

	for (int i = 0; i < m_vUnfinishedPath.size(); ++i)
	{
		QGraphicsItem *item = find_bezier(m_vUnfinishedPath[i].x(), m_vUnfinishedPath[i].y());
		if (item != NULL) {
			QPainterPath unfinished_bezier_path = cal_bezier_path(item, m_vUnfinishedPath[i].x(), m_vUnfinishedPath[i].y());
			m_lstUnfinishedBezierPaths.push_back(unfinished_bezier_path);
		}
	}

	for (int i = 0; i < m_lstInvalidLines.size(); ++i)
	{

		QGraphicsItem *item = find_bezier(m_lstInvalidLines[i].x(), m_lstInvalidLines[i].y());
		if (item != NULL) 
		{
			QPainterPath invalid_bezier_path = cal_bezier_path(item, m_lstInvalidLines[i].x(), m_lstInvalidLines[i].y());
			m_lstInvalidBezierPaths.push_back(invalid_bezier_path);
		}
	
	}

	//qDebug() << "===================invalid list size:" << invalid_list_.size();

	//更新当前的变量值
	percentage_ = percentage;
	currentPath_end_index_ = end_index;

	update();
}





