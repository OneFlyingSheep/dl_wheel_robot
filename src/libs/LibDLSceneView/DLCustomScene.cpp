#include <QtWidgets>
#include <fstream>
#include <iostream>
#include <QTime>
#include <QGraphicsPixmapItem>
#include <QListWidget>
#include <QDebug>
#include "DLCustomScene.h"
#include "DLOperator.h"
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
#include "LibDlToolItems/LoadPixmapItem.h"
#include "LibMapReaderInfoWidget/MapReaderInfoWidget.h"
#include "LibDLWheelRobotCoreSocket/LibDLWheelRobotCoreSocket.h"
#include "LibDlToolItems/DLRobotItem.h"
#include "LibDlToolItems/DLPathItem.h"
#include "LibDlToolItems/DLLaserItem.h"
//////////////////////////////////////////////////////////////////////////
#include "LibDlToolItems/DLPlatformItem.h"
#include "LibDLDeviceViewWidget/LibDLDeviceViewWidget.h"

//////////////////////////////////////////////////////////////////////////
#include "LibDLCollectMap/DLCollectMap.h"
#include "LibDLWheelRobotDBOperation/LibDLWheelRobotDBOperation.h"
#include "LibDLWheelRobotConfigData/DLWheelRobotBackgroundConfig.h"

#include "DLViewGrabDlg.h"
#include "DLOverfittingDlg.h"

#include "LibMapReader/ReadBgMapThread.h"
#include "LibMapReader/MapData.h"


#define CATCH_FACTOR (5)
#define PI (3.14159)

#define DRAW_PATH_Z_VALUE 42
#define DRAW_AREA_WIDTH 20
#define DRAW_SELECT_AREA 51

#define ROUND_DISTANCE qreal(0.1)					//判断加点范围值，单位m

double rad2angle(double rad)
{
	//（-π，π）转换到（0， 2π）
	double angle = 0;
	if (rad > 0) {
		angle = rad * 180 / 3.14159;
	}
	else {
		angle = ((2 * 3.14159 + rad) * 180) / 3.14159;
	}

	//逆时针转换到顺时针
	return (360 - angle);
}

double angle2rad(double angle)
{
	double rad = 0;
	if (angle > 0 && angle < 180) {
		rad = (0 - angle) * 3.14159 / 180;
	}
	else if (angle > 180 && angle < 360) {
		rad = (360 - angle) * 3.14159 / 180;
	}
	else if (abs(angle - 180) < 0.00001) {
		rad = 3.14159;
	}

	return rad;
}

CheckDeviceDlg::CheckDeviceDlg(QWidget *parent)
	:QDialog(parent)
{
	QVBoxLayout *pVMainLayout = new QVBoxLayout(this);
	m_pContextListWgt = new QListWidget(this);
	m_pContextListWgt->setAlternatingRowColors(true);
	pVMainLayout->addWidget(m_pContextListWgt);

	//QString strStyle = "QListWidget{gridline-color:red;}";
	//this->setStyleSheet(strStyle);


	setWindowModality(Qt::ApplicationModal);        //设置阻塞类型

	resize(400, 550);
}

CheckDeviceDlg::~CheckDeviceDlg()
{

}


void CheckDeviceDlg::LoadData(const QStringList &lstEquipmentInfos)
{
	m_pContextListWgt->clear();
	foreach(QString strEquipmentInfo, lstEquipmentInfos)
	{
		QListWidgetItem *pItem = new QListWidgetItem(strEquipmentInfo);
		pItem->setTextAlignment(Qt::AlignCenter);
		m_pContextListWgt->addItem(pItem);
	}
}

DLCustomScene::DLCustomScene(int iSceneType, QObject *parent)
	: QGraphicsScene(parent)
	, m_iCustomSceneType(iSceneType)
	, m_bIsDrawState(false)
	, m_pDrawPathItem(NULL)
	, m_pCurSelectItem(NULL)
	, m_pCheckDeviceDlg(NULL)
	, m_bIsCutoutState(false)
	, m_ptStartPoint(0, 0)
	, m_ptEndPoint(0, 0)
	, m_pViewGrabDlg(NULL)
//	, m_strBgPixmap("")
	//, m_pBgPixmapItem(NULL)
	, m_rResolution(0.1)				//默认地图分辨率
	, m_ptBgCenterPoint(0.0, 0.0)		//背景图片中心点
	//, m_strBgImagePath("")				//背景图片路径
	, m_pOverFittingDlg(NULL)			//拟合窗口
	, m_pOverFittingLineItem(NULL)
	//, m_pReadBgMapThread(NULL)
	//, m_pMutilNormalPointItem(NULL)
	, m_pLoadPixmapItem(NULL)
	,m_pPathItem(NULL)

{
	qRegisterMetaType<WheelRobotSwitchRunningStatus>("WheelRobotSwitchRunningStatus");
	qRegisterMetaType<WheelRobotTaskCurrentPointStatus>("WheelRobotTaskCurrentPointStatus");
	qRegisterMetaType<WheelRobotRealtimeStatus>("WheelRobotRealtimeStatus");
	qRegisterMetaType<QRectF>("QRectF");
	qRegisterMetaType<QRectF>("QPointF");
	qRegisterMetaType<QRectF>("QVector<QVector<AreaInfo>>");

	m_bIsStart = false;
	m_iTaskPatrolPoint = 0;
	m_bPressDown = false;
	m_pSelectItem = NULL;
	m_pPlatformItem = NULL;

	m_dRobotCurrentX = 0;
	m_dRobotCurrentY = 0;

	m_pRobotItem = NULL;
	//m_pMutilNormalPointItem = NULL;
	m_pLaserItem = NULL;
	m_pLandmarkPropertyWidget = NULL;
	m_pDeviceAreaPropertyWidget = NULL;

	m_bIsPress = false;
	m_iBezierId = -1;
	m_iAdvancedAreaId = -1;
	m_dCoordinateAngle = 0;
	m_ptTransformPoint = QPointF(0.0, 0.0);


	//初始化
	m_dRobotCurrentAngle = 0;

	if (DL_BROWSE_TYPE == m_iCustomSceneType)
	{//客户端
		create_pathItem();
		create_platformItem();
		ConnectSock();
	}


	//////////////////////////////////////////////////////////////////////////

	CreateRobotItem();
	m_pWorldCoordinateItem = new DLCoordinateItem;
	m_pGridItem = new DLGridItem(QRect(-20000, -20000, 40000, 40000));
	this->addItem(m_pGridItem);

	m_pHLineItem = new QGraphicsLineItem;
	m_pVLineItem = new QGraphicsLineItem;
	m_pHLineItem->setPen(QPen(Qt::black, 3, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin));
	m_pVLineItem->setPen(QPen(Qt::black, 3, Qt::DotLine, Qt::RoundCap, Qt::RoundJoin));
	m_pHLineItem->setZValue(20);
	m_pVLineItem->setZValue(20);
	m_pHLineItem->hide();
	m_pVLineItem->hide();
	this->addItem(m_pHLineItem);
	this->addItem(m_pVLineItem);
	this->addItem(m_pWorldCoordinateItem);
	m_pWorldCoordinateItem->setZValue(1);
	m_pOperator = new DLOperator(this);
	m_pReaderThread = new MapReader(this);
	m_pReaderInfoWidget = new MapReaderInfoWidget();
	m_pLandmarkPropertyWidget = new LandMarkPropertyWidget;
	m_pDeviceAreaPropertyWidget = new DeviceAreaPropertyWidget;
	m_pBezierPropertyWidget = new BezierPropertyWidget;
	m_pAdvancedAreaPropertyWidget = new AdvancedAreaPropertyWidget;

	//m_pReadBgMapThread = new ReadBgMapThread(this);
	//connect(m_pReadBgMapThread, SIGNAL(SendAreaPointsSignal(QPointF, QPointF)), this, SLOT(SendAreaPointsSlot(QPointF, QPointF)));


	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotCurrentLoc2CollectMapTable.connect(boost::bind(&DLCustomScene::SetRobotItemPropertry, this, _1, _2, _3));    //实时更新机器人的位置
	QTimer *timer_ = new QTimer(this);
	timer_->start(100);
	connect(timer_, SIGNAL(timeout()), this, SLOT(UpdateRobotItem()));
	if (DL_BROWSE_TYPE != m_iCustomSceneType)
	{//不是客户端
		create_laserItem();
		// 绑定激光数据;
		WHEEL_BACK_TO_CORE_SOCKET.wheelRobotLaserData2CollectMapTable.connect(boost::bind(&DLCustomScene::set_laserItem_propertry, this, _1));
	
		connect(timer_, SIGNAL(timeout()), this, SLOT(update_laserItem()));
	}
	connect(m_pReaderThread, SIGNAL(FinishedSignals(int, bool)), this, SLOT(ReadFinishedSlot(int, bool)));
	setItemIndexMethod(QGraphicsScene::NoIndex);
	m_iLandmarkId = 0;
	m_pPointInfoViewer = new DLPointInfoWidget;
	QPixmapCache::setCacheLimit(1);
}


DLCustomScene::~DLCustomScene()
{
	if (m_pReaderInfoWidget != NULL)
	{
		delete m_pReaderInfoWidget;
		m_pReaderInfoWidget = NULL;
	}
	if (m_pLandmarkPropertyWidget != NULL)
	{
		delete m_pLandmarkPropertyWidget;
		m_pLandmarkPropertyWidget = NULL;
	}
	if (m_pDeviceAreaPropertyWidget != NULL)
	{
		delete m_pDeviceAreaPropertyWidget;
		m_pDeviceAreaPropertyWidget = NULL;
	}
	if (m_pBezierPropertyWidget != NULL)
	{
		delete m_pBezierPropertyWidget;
		m_pBezierPropertyWidget = NULL;
	}
	if (m_pAdvancedAreaPropertyWidget != NULL)
	{
		delete m_pAdvancedAreaPropertyWidget;
		m_pAdvancedAreaPropertyWidget = NULL;
	}
	// 	if (m_pDeviceViewer != NULL)
	// 	{
	// 		delete m_pDeviceViewer;
	// 		m_pDeviceViewer = NULL;
	// 	}
	if (m_pPointInfoViewer != NULL)
	{
		m_pPointInfoViewer->hide();
		delete m_pPointInfoViewer;
		m_pPointInfoViewer = NULL;
	}
}


void DLCustomScene::SetOperateType(int type)
{

// 	if (m_mapNormalPos.size() != 0)
// 	{
// 		if (type == DLOperator::TYPE_COORDINATE_TRANSFORM) {
// 			m_pMutilNormalPointItem->setTransformAble(true);
// 		}
// 		else {
// 			m_pMutilNormalPointItem->setTransformAble(false);
// 		}
// 	}

	if (DLOperator::TYPE_SELECT == type)
	{//添加巡检路径
		SetLandmarkItemMoved(true);
	}
	else
	{
		SetLandmarkItemMoved(false);
	}

	return m_pOperator->set_type(type);
}


QUndoStack* DLCustomScene::UndoStack()
{
	return m_pOperator->undo_stack();
}


DLOperator* DLCustomScene::GetOperate()
{
	return m_pOperator;
}


QPointF DLCustomScene::GetCenter()
{
	QPointF center_point(0, 0);

	if (m_mapDeviceAreas.size() != 0) {
		for (auto it = m_mapDeviceAreas.begin(); it != m_mapDeviceAreas.end(); ++it)
		{
			if (((DLDeviceAreaItem*)it->second)->className().compare("STATION") == 0)
			{
				HShape::Rectangle rect = ((DLDeviceAreaItem*)it->second)->get_pro();
				center_point.setX(rect.center_.x_);
				center_point.setY(rect.center_.y_);
			}
		}
	}
	return center_point;
}

void DLCustomScene::AddPatrolPoint()
{
// 	HShape::Ellipse pro;
// 	double angle_value = m_dRobotCurrentAngle * 180 / PI;
// 
// 	pro.angle_ = rad2angle(angle_value);
// 	pro.center_.x_ = m_dRobotCurrentX / m_rResolution;
//     pro.center_.y_ = m_dRobotCurrentY / m_rResolution;
// //    pro.center_.y_ = m_dRobotCurrentY / m_rResolution;
// 	pro.width_ = 20;
// 	pro.height_ = 20;
// 
// 	int id = GetLandmarkId() + 1;
// 	QString name = QString::number(id);
// 	DLLandmarkItem* item = new DLLandmarkItem(pro, id, name.toStdString());
// 	add_advancedPoint(item);
}


void DLCustomScene::TaskBegin(WheelRobotTaskBegin task_begin)
{
	m_bIsStart = false;
	std::vector<int> patrol_points;
	std::vector<QPoint> work_path;
	m_iTaskPatrolPoint = 0;
	SetWorkPath(work_path, patrol_points, false);

	if ("ChargingTask" == task_begin.task_uuid) {
		ROS_INFO("DLBackStageMapScene ChargingTask");
		return;
	}


	m_bIsStart = true;
	QString task_uuid = task_begin.task_uuid;
	QString start_time = task_begin.start_time;
	int  predict_duration = task_begin.predict_duration;
	QStringList points = task_begin.points;


	for (int i = 0; i < points.size(); i++)
	{
		int index = points[i].toInt();
		patrol_points.push_back(index);

		if (points.size() == 1) {
			QPoint path_index(index, index);
			work_path.push_back(path_index);
		}
		else {
			if (i < points.size() - 1) {
				int start_index = points[i].toInt();
				int end_index = points[i + 1].toInt();
				QPoint path_index(start_index, end_index);
				work_path.push_back(path_index);
			}
		}
	}

	//初始化机器人和云台位置
	if (work_path.size() > 0) {
		if (work_path[0].x() == work_path[0].y())
		{
			QGraphicsItem *item = find_advancedpoint(work_path[0].x());
			if (item != NULL) {
				QPointF point = item->scenePos();
				//qDebug() << "==================机器人的坐标:" << point;
				m_pRobotItem->setPos(point);
			}
		}
	}

	//生成巡检路径
	SetWorkPath(work_path, patrol_points, true);
	for (int i = 0; i < points.size(); ++i)
	{
		ROS_INFO("DLBackStageMapScene index list:%d", points[i].toInt());
	}
	ROS_INFO("DLBackStageMapScene task_begin task_uuid: %s, patrol_id size:%d, work_path size:%d", task_uuid.toLocal8Bit().constData(), patrol_points.size(), work_path.size());

}

void DLCustomScene::SetCurrentPathIndex(int path_index)
{
	m_mutexPatrolPoint.lock();
	m_iTaskPatrolPoint = path_index;
	m_mutexPatrolPoint.unlock();
	ROS_INFO("DLBackStageMapScene current_path_index:%d", path_index);
}

void DLCustomScene::TaskEnd(QString task_uuid, QString task_time, int ret_code)
{
	m_bIsStart = false;
	std::vector<int> patrol_points;
	std::vector<QPoint> work_path;
	m_iTaskPatrolPoint = 0;
	SetWorkPath(work_path, patrol_points, false);
	ROS_INFO("DLBackStageMapScene task_end:%d", ret_code);
}

void DLCustomScene::FlushCarItemPropertry(double pos_x, double pos_y, double angle)
{
	m_mutexLocation.lock();
	//qDebug() << "==============刷新机器人的角度：" << angle * 180 / PI <<  ";原始角度：" << angle;
	m_dRobotCurrentAngle = angle;
	m_mutexLocation.unlock();
}

QRectF DLCustomScene::GetPathRect()
{
	qreal rTop = 0.0;
	qreal rLeft = 0.0;
	qreal rBottom = 0.0;
	qreal rRight = 0.0;
	bool bFirstIn = true;
	foreach(QPainterPath painterPath, m_lstPainterPaths)
	{
		QRectF rect = painterPath.boundingRect();
		if (bFirstIn)
		{
			bFirstIn = false;
			rTop = rect.top();
			rLeft = rect.left();
			rBottom = rect.bottom();
			rRight = rect.right();
		}
		else
		{
			if (rTop > rect.top())
			{
				rTop = rect.top();
			}

			if (rLeft > rect.left())
			{
				rLeft = rect.left();
			}

			if (rBottom < rect.bottom())
			{
				rBottom = rect.bottom();
			}

			if (rRight < rect.right())
			{
				rRight = rect.right();
			}
		}

	}
// 
// 	if (NULL != m_pMutilNormalPointItem)
// 	{
// 		QRectF rectMutilNormalPoint = m_pMutilNormalPointItem->boundingRect();
// 
// 		if (rLeft > rectMutilNormalPoint.left())
// 		{
// 			rLeft = rectMutilNormalPoint.left();
// 		}
// 		if (rTop > rectMutilNormalPoint.top())
// 		{
// 			rTop = rectMutilNormalPoint.top();
// 		}
// 		if (rRight < rectMutilNormalPoint.right())
// 		{
// 			rRight = rectMutilNormalPoint.right();
// 		}
// 		if (rBottom < rectMutilNormalPoint.bottom())
// 		{
// 			rBottom = rectMutilNormalPoint.bottom();
// 		}
// 	}

	return QRectF(rLeft - 50, rTop - 50, rRight - rLeft + 100, rBottom - rTop + 100);
}

void DLCustomScene::CreateRobotItem()
{
	if (m_pRobotItem == NULL)
	{
		m_pRobotItem = new DLRobotItem;
		m_pRobotItem->setZValue(45);
		this->addItem(m_pRobotItem);
		m_pRobotItem->setPos(QPointF(0, 0));
	}

}


void DLCustomScene::SetRobotItemPropertry(double pos_x, double pos_y, double angle)
{
// 	if (NULL != m_pRobotItem && m_iCustomSceneType == DL_BROWSE_TYPE)
// 	{
// 		m_pRobotItem->flushRobot(pos_x, pos_y, angle);
// 		return;
// 	}

	m_mutexLocation.lock();
	//qDebug() << "=============设置机器人节点属性：" << pos_x << pos_y << angle;
	m_dRobotCurrentX = pos_x;
	m_dRobotCurrentY = pos_y;
	m_dRobotCurrentAngle = angle;
	m_mutexLocation.unlock();

}

void DLCustomScene::UpdateRobotItem()
{
	m_mutexLocation.lock();
	if (m_pRobotItem != NULL)
	{
		//qDebug() << "============机器人原坐标：" << m_dRobotCurrentX << m_dRobotCurrentY << m_dRobotCurrentAngle;
		m_pRobotItem->flushRobot(m_dRobotCurrentX / m_rResolution, m_dRobotCurrentY / m_rResolution, m_dRobotCurrentAngle * 180 / PI);
	}
	m_mutexLocation.unlock();
}




DLRobotItem* DLCustomScene::RobotItem()
{
	return m_pRobotItem;
}


void DLCustomScene::create_pathItem()
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType)
	{
		if (m_pPathItem == NULL)
		{
			m_pPathItem = new DLPathItem;
			if (m_mapDeviceAreas.size() == 0) {
				m_pPathItem->initUpdateRect(QRectF(-10000, -10000, 20000, 20000));
			}
			else
			{
				//根据区域设置pathItem的尺寸
				auto itor = m_mapDeviceAreas.begin();
				HShape::Rectangle rect = ((DLDeviceAreaItem*)itor->second)->get_pro();
				m_pPathItem->initUpdateRect(QRectF(rect.center_.x_ - rect.width_ / 2 - 20, rect.center_.y_ - rect.height_ / 2 - 20, rect.width_ + 40, rect.height_ + 40));
			}
			m_pPathItem->initPath(m_mapAdvancedCurves);
			this->addItem(m_pPathItem);
		}
	}
	else //if (DL_COLLECT_EDIT_TYPE == m_iCustomSceneType)
	{
		if (m_pPathItem == NULL) {
			m_pPathItem = new DLPathItem;
			this->addItem(m_pPathItem);
		}
	}

}


// void DLCustomScene::hide_pathItem()
// {
// 	if (m_pPathItem != NULL) {
// 		m_pPathItem->hide();
// 	}
// }


// void DLCustomScene::show_pathItem()
// {
// 	if (m_pPathItem != NULL) {
// 		m_pPathItem->show();
// 	}
// }


// void DLCustomScene::set_pathItem_propertry(int id)
// {
// 
// }


// DLPathItem *DLCustomScene::pathItem()
// {
// 	return m_pPathItem;
// }


void DLCustomScene::setRelocateRetVal(int retVal)
{

}


int DLCustomScene::GetRelocateRetVal()
{
	return 0;
}


void DLCustomScene::create_laserItem()
{
	if (NULL == m_pLaserItem) {
		m_pLaserItem = new DLLaserItem;
		this->addItem(m_pLaserItem);
	}
}


// void DLCustomScene::hide_laserItem()
// {
// 	if (NULL != m_pLaserItem) {
// 		m_pLaserItem->hide();
// 	}
// }


// void DLCustomScene::show_laserItem()
// {
// 	if (NULL != m_pLaserItem) {
// 		m_pLaserItem->show();
// 	}
// }


void DLCustomScene::set_laserItem_propertry(std::vector<QPointF> pointList)
{
	m_mutexLaser.lock();
	m_vLaserDatas.clear();
	m_vLaserDatas = pointList;
//    m_vLaserDatas.push_back(QPointF(m_dRobotCurrentX, -m_dRobotCurrentY));
    m_vLaserDatas.push_back(QPointF(m_dRobotCurrentX, -m_dRobotCurrentY));
	m_mutexLaser.unlock();
}

void DLCustomScene::update_laserItem()
{
	m_mutexLaser.lock();
	if (NULL != m_pLaserItem) {
		if (m_vLaserDatas.size() > 1)
			m_pLaserItem->flushData(m_vLaserDatas, m_rResolution);
	}
	m_mutexLaser.unlock();
}


// DLLaserItem *DLCustomScene::laserItem()
// {
// 	return m_pLaserItem;
// }


QGraphicsItem* DLCustomScene::find_advancedCurve(int start_id, int end_id)
{
	{
		int id = (start_id << 16) + end_id;
		std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedCurves.find(id);
		if (it != m_mapAdvancedPoints.end()) {
			return it->second;
		}
		else {
			return NULL;
		}
	}

	{
		int id = (end_id << 16) + start_id;
		std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedCurves.find(id);
		if (it != m_mapAdvancedPoints.end()) {
			return it->second;
		}
		else {
			return NULL;
		}

	}
}


QGraphicsItem* DLCustomScene::find_advancedCurve(int start_id, int end_id, bool &is_positive)
{
	bool is_find = false;
	int id_1 = (start_id << 16) + end_id;
	std::map<int, QGraphicsItem*>::iterator itor_1 = m_mapAdvancedCurves.find(id_1);
	if (itor_1 != m_mapAdvancedCurves.end()) {
		is_positive = true;
		ROS_INFO("DLBackStageMapScene bezier_path dir:%d", is_positive);
		return itor_1->second;
	}
	else {
		is_find = false;
	}

	int id_2 = (end_id << 16) + start_id;
	std::map<int, QGraphicsItem*>::iterator itor_2 = m_mapAdvancedCurves.find(id_2);
	if (itor_2 != m_mapAdvancedCurves.end()) {
		is_positive = false;
		ROS_INFO("DLBackStageMapScene bezier_path dir:%d", is_positive);
		return itor_2->second;
	}
	else {
		is_find = false;
	}

	if (is_find == false) {
		return NULL;
	}
}

bool DLCustomScene::IsBorderPoint(QGraphicsItem *pItem)
{
	//return true;
	int index = 0;
	foreach(QPainterPath path, m_lstPainterPaths)
	{
		if ((path.contains(pItem->pos()) || m_lstLines.at(index).p1() == pItem->pos() || m_lstLines.at(index).p2() == pItem->pos()) && pItem->pos() != m_lstPaths.at(m_lstPaths.size() - 1)
			&& (path.contains(m_lstPaths.at(m_lstPaths.size() - 1)) || (m_lstLines.at(index).p1() == m_lstPaths.at(m_lstPaths.size() - 1) || m_lstLines.at(index).p2() == m_lstPaths.at(m_lstPaths.size() - 1))))
		{
			return true;
		}
		++index;
	}
	return false;
}

void DLCustomScene::DrawCurPath()
{
	if (NULL != m_pDrawPathItem)
	{
		this->removeItem(m_pDrawPathItem);
		delete m_pDrawPathItem;
		m_pDrawPathItem = NULL;
	}

	QPainterPath path;
	//path.moveTo(m_lstPaths.at(0));
	bool bFirstIn = true;
	foreach(QPointF ptCurPos, m_lstPaths)
	{
		if (bFirstIn)
		{
			bFirstIn = false;
			path.moveTo(ptCurPos);
			continue;
		}
		path.lineTo(ptCurPos);
	}
	//path.lineTo(ptMovePos);
	QPen pen;
	pen.setColor(Qt::red);
	pen.setWidth(3);
	m_pDrawPathItem = this->addPath(path, pen);
	m_pDrawPathItem->setZValue(DRAW_PATH_Z_VALUE);
}

void DLCustomScene::DrawCurPath(QPointF ptMovePos)
{
	if (NULL != m_pDrawPathItem)
	{
		this->removeItem(m_pDrawPathItem);
		delete m_pDrawPathItem;
		m_pDrawPathItem = NULL;
	}

	QPainterPath path;
	//path.moveTo(m_lstPaths.at(0));
	bool bFirstIn = true;
	foreach(QPointF ptCurPos, m_lstPaths)
	{
		if (bFirstIn)
		{
			bFirstIn = false;
			path.moveTo(ptCurPos);
			continue;
		}
		path.lineTo(ptCurPos);
	}
	path.lineTo(ptMovePos);
	QPen pen;
	pen.setColor(Qt::red);
	pen.setWidth(3);
	m_pDrawPathItem = this->addPath(path, pen);
	m_pDrawPathItem->setZValue(DRAW_PATH_Z_VALUE);
}

QPointF DLCustomScene::CalcNeedPos(const QLineF line, QPointF &ptCurPos)
{
	QPointF ptStartPos;
	QPointF ptEndPos;
	double k1 = 0.0;		//原直线的斜率
	double k2 = 0.0;		//当前直线的斜率
	if (m_lstPaths.at(m_lstPaths.size() - 1) == line.p1())
	{
		ptStartPos = line.p1();
		ptEndPos = line.p2();
	}
	else if (m_lstPaths.at(m_lstPaths.size() - 1) == line.p2())
	{
		ptStartPos = line.p2();
		ptEndPos = line.p1();
	}
	else if (m_lstPaths.size() == 1)
	{
		ptStartPos = m_pRobotItem->pos();

		QLineF lineRobot(line.p1(), m_pRobotItem->pos());  //机器人与p1
		QLineF lineCur(line.p1(), ptCurPos);				//点击点与p1

		int iRobotLen = lineRobot.length();					//机器人离p1的距离
		int iCurLen = lineCur.length();						//点击点离p1的距离
		if (iRobotLen > iCurLen)
		{
			ptEndPos = line.p1();
		}
		else
		{
			ptEndPos = line.p2();
		}
	}


	QLineF lineCur(ptStartPos, ptCurPos);
	QLineF lineOld(ptStartPos, ptEndPos);
	lineOld.setLength(lineCur.length());
	return lineOld.p2();
}

void DLCustomScene::UpdateLandmarkItemState(QPointF ptMovePos)
{
	//删除已有的节点状态
	DLLandmarkItem *pCurItem = dynamic_cast<DLLandmarkItem *>(m_pCurSelectItem);
	if (NULL != pCurItem)
	{
		pCurItem->SetDrawState(false);
	}
	else if (NULL != m_pCurSelectItem)
	{
		this->removeItem(m_pCurSelectItem);
		delete m_pCurSelectItem;
		m_pCurSelectItem = NULL;
	}


	QGraphicsItem *pItem = this->itemAt(ptMovePos, QTransform());
	DLLandmarkItem * pLandmarkItem = dynamic_cast<DLLandmarkItem *>(pItem);
	if (pLandmarkItem)
	{//如果是端点
		if (IsBorderPoint(pLandmarkItem))
		{
			pLandmarkItem->SetDrawState(true);
			m_pCurSelectItem = pLandmarkItem;
		}
	}
	else
	{//如果是区域点
		int index = 0;
		foreach(QPainterPath path, m_lstPainterPaths)
		{
			//qDebug() << "move pos:" << ptMovePos << "line p1:" << m_lstLines.at(index).p1() << "line p2:" << m_lstLines.at(index).p2() << "lst end pos:" << m_lstPaths.at(m_lstPaths.size() - 1);
			if (path.contains(ptMovePos) && (m_lstLines.at(index).p1() == m_lstPaths.at(m_lstPaths.size() - 1) || m_lstLines.at(index).p2() == m_lstPaths.at(m_lstPaths.size() - 1)))
			{
				QPen pen;
				pen.setColor(Qt::green);
				pen.setWidth(3);

				m_pCurSelectItem = this->addPath(path, pen, QBrush(Qt::green));
				m_pCurSelectItem->setOpacity(0.5);
				m_pCurSelectItem->setZValue(DRAW_SELECT_AREA);
				break;
			}
			else if (path.contains(ptMovePos) && m_lstPaths.size() == 1 && path.contains(m_pRobotItem->pos()))
			{
				QPen pen;
				pen.setColor(Qt::green);
				pen.setWidth(3);

				m_pCurSelectItem = this->addPath(path, pen, QBrush(Qt::green));
				m_pCurSelectItem->setOpacity(0.5);
				m_pCurSelectItem->setZValue(DRAW_SELECT_AREA);
				break;
			}

			++index;
		}
	}

}

bool DLCustomScene::MessageBox()
{
	QMessageBox msgBox;
	msgBox.setWindowTitle("自主导航");
	msgBox.setText("是否按此路径进行自主导航行走");
	msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
	msgBox.setButtonText(QMessageBox::Ok, QString("确 定"));
	msgBox.setButtonText(QMessageBox::Cancel, QString("取 消"));
	msgBox.setDefaultButton(QMessageBox::Ok);
	int ret = msgBox.exec();
	if (ret == QMessageBox::Cancel)
	{
		return false;
	}
	return true;
}

void DLCustomScene::SendData2Core()
{
	QList<MapSettingOutCoordinates> data;
	bool bIsFirstIn = true;
	foreach(QPointF ptCurPos, m_lstPaths)
	{
		if (/*this->itemAt(ptCurPos, QTransform()) == m_pRobotItem &&*/ bIsFirstIn)
		{//如果是机器人，不记录
			bIsFirstIn = false;
			continue;
		}
		MapSettingOutCoordinates stMapSettingOutCoordinates;
		DLLandmarkItem * pLandmarkItem = dynamic_cast<DLLandmarkItem *>(this->itemAt(ptCurPos, QTransform()));
		if (pLandmarkItem)
		{//如果是巡检点
			stMapSettingOutCoordinates.pointType = 0;
			stMapSettingOutCoordinates.pointID = pLandmarkItem->id();
		}
		else
		{//不是巡检点
			stMapSettingOutCoordinates.pointType = 1;
		}
		stMapSettingOutCoordinates.pointX = ptCurPos.rx() * m_rResolution;							//发送点的单位为m
		//stMapSettingOutCoordinates.pointX = ptCurPos.rx() / 100;							//发送点的单位为m
		stMapSettingOutCoordinates.pointY = -ptCurPos.ry() * m_rResolution;
		//stMapSettingOutCoordinates.pointY = ptCurPos.ry() / 100;
		data.push_back(stMapSettingOutCoordinates);
	}
	WHEEL_BACK_TO_CORE_SOCKET.robot_setting_out_run_point_req(data);						//发送节点
}

void DLCustomScene::InitAllArea()
{
	//初始化所有区域

	m_lstPainterPaths.clear();
	m_lstLines.clear();

	double dStartX = 0.0;
	double dStartY = 0.0;
	double dEndX = 0.0;
	double dEndY = 0.0;
	bool bIsFirstIn = true;


	auto iter = m_mapAdvancedCurves.begin();
	while (iter != m_mapAdvancedCurves.end())
	{
		int start_index = ((DLBezierItem*)iter->second)->start_id();
		int end_index = ((DLBezierItem*)iter->second)->end_id();
		//QPoint index(start_index, end_index);
		QGraphicsItem *item = find_bezier(start_index, end_index);
		if (item != NULL)
		{
			QLineF line = cal_bezier_path(item, start_index, end_index);
			m_lstLines.push_back(line);

			//QLineF lineTemp = line;			//记录最原始的线

			//处理区域
			QPainterPath painterPath;
			//QLineF line(QPointF(dStartX, dStartY), QPointF(dEndX, dEndY));
			int iLength = line.length();
			//第一次旋转
			line.setAngle(line.angle() + 90);
			line.setLength(DRAW_AREA_WIDTH / 2);

			QPointF ptStartPos = line.p2();
			//开始绘制
			painterPath.moveTo(line.p2());

			//第二次旋转
			line = QLineF(line.p2(), line.p1());
			line.setAngle(line.angle() + 90);
			line.setLength(iLength);
			painterPath.lineTo(line.p2());

			//第三次旋转
			line = QLineF(line.p2(), line.p1());
			line.setAngle(line.angle() + 90);
			line.setLength(DRAW_AREA_WIDTH);
			painterPath.lineTo(line.p2());

			//第四次旋转
			line = QLineF(line.p2(), line.p1());
			line.setAngle(line.angle() + 90);
			line.setLength(iLength);
			painterPath.lineTo(line.p2());

			painterPath.lineTo(ptStartPos);
			//m_mapPath2Line.insert(painterPath, lineTemp);
			m_lstPainterPaths.push_back(painterPath);

		}
		iter++;
	}
}

QGraphicsItem* DLCustomScene::find_bezier(int start_id, int end_id)
{
	int id = (start_id << 16) + end_id;//潜规则
	int _id = (end_id << 16) + start_id;//潜规则

										//巡检路线可能是反向的，搜寻两次确保一定找到
	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedCurves.find(id);
	std::map<int, QGraphicsItem*>::iterator _it = m_mapAdvancedCurves.find(_id);

	if (it != m_mapAdvancedCurves.end()) {
		return it->second;
	}

	if (_it != m_mapAdvancedCurves.end()) {
		return _it->second;
	}
	if (it == m_mapAdvancedCurves.end() && it == m_mapAdvancedCurves.end()) {
		return NULL;
	}
}

QLineF DLCustomScene::cal_bezier_path(QGraphicsItem *bezier_item, int start_index, int end_index)
{
	QPainterPath path;
	QLineF line;
	QPointF startPos, c1Pos, c2Pos, endPos;
	if (bezier_item != NULL) {

		int bezier_start_index = ((DLBezierItem*)bezier_item)->start_id();
		int bezier_end_index = ((DLBezierItem*)bezier_item)->end_id();

		if (start_index == bezier_start_index && end_index == bezier_end_index) {
			startPos = ((DLBezierItem*)bezier_item)->start_pos();
			c1Pos = ((DLBezierItem*)bezier_item)->c1_pos();
			c2Pos = ((DLBezierItem*)bezier_item)->c2_pos();
			endPos = ((DLBezierItem*)bezier_item)->end_pos();
		}
		else {
			endPos = ((DLBezierItem*)bezier_item)->start_pos();
			c2Pos = ((DLBezierItem*)bezier_item)->c1_pos();
			c1Pos = ((DLBezierItem*)bezier_item)->c2_pos();
			startPos = ((DLBezierItem*)bezier_item)->end_pos();
		}
	}

	line.setPoints(startPos, endPos);
	return line;
}

void DLCustomScene::ConnectSock()
{
	//绑定返回机器人云台的转动方向
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotRealtimeStatus.connect(boost::bind(&DLCustomScene::SigFlushPlateform, this, _1));
	//WHEEL_BACK_TO_CORE_SOCKET.wheelRobotRealtimeStatus.connect(boost::bind(&DLBackStageMapScene::flush_platformItem_propertry, this, _1));

	//绑定返回当前任务巡检设备巡检结果的回调函数
	//WHEEL_BACK_TO_CORE_SOCKET.WheelRobotInspectResultCallback.connect(boost::bind(&DLDeviceViewer::set_device_propertry, m_pDeviceViewer, _1));

	//绑定任务开始初始化路径
	WHEEL_BACK_TO_CORE_SOCKET.signal_wheelRobotTaskBegin.connect(boost::bind(&DLCustomScene::TaskBegin, this, _1));

	//绑定任务当前路径点序列号
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotUpdateTaskSerialNum.connect(boost::bind(&DLCustomScene::SetCurrentPathIndex, this, _1));

	//绑定任务结束
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotEndTaskMsg.connect(boost::bind(&DLCustomScene::TaskEnd, this, _1, _2, _3));

	//绑定返回当前机器人的所处路线的巡检路线的每一小段的起始位置和所处位置百分比
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotCurrentPointStatusCallback.connect(boost::bind(&DLCustomScene::SigFlushPath, this, _1));

	//绑定机器人在真实世界坐标系的位置
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotCurrentLoc2CollectMapTable.connect(boost::bind(&DLCustomScene::FlushCarItemPropertry, this, _1, _2, _3));

	//连接刷新路径曲线的属性
	connect(this, SIGNAL(SigFlushPath(WheelRobotTaskCurrentPointStatus)), this, SLOT(FlushRobotSlot(WheelRobotTaskCurrentPointStatus)), Qt::QueuedConnection);

	//连接刷新云台属性
	connect(this, SIGNAL(SigFlushPlateform(WheelRobotRealtimeStatus)), this, SLOT(FlushPlatformSlot(WheelRobotRealtimeStatus)), Qt::QueuedConnection);

	// 更新当前机器人模式状态;
	WHEEL_BACK_TO_CORE_SOCKET.wheelRobotSwitchRunningStatusSignal.connect(boost::bind(&DLCustomScene::signalRobotModeStateCallBack, this, _1));

	connect(this, &DLCustomScene::signalRobotModeStateCallBack, this, [=](WheelRobotSwitchRunningStatus status) {
		if (status != WHEEL_ROBOT_SWITCH_AUTORUNNING) {
			m_bIsStart = false;
		}
	});

}

void DLCustomScene::SetLandmarkItemMoved(bool bIsMoved)
{
	QList<QGraphicsItem *> lstItems = this->items();
	foreach(QGraphicsItem *pItem, lstItems)
	{
		DLLandmarkItem *pLandmarkItem = dynamic_cast<DLLandmarkItem *>(pItem);
		if (NULL != pLandmarkItem)
		{
			pLandmarkItem->setFlag(QGraphicsItem::ItemIsMovable, bIsMoved);
		}
	}
}

bool DLCustomScene::IsExistOtherAdvanceArea(QGraphicsItem *pAdvaceAreaItem, QGraphicsItem *pItem)
{
	if (nullptr == pAdvaceAreaItem || nullptr == pItem) return true;
	std::map<QGraphicsItem *, QVector<QGraphicsItem *>>::iterator it= m_mapAdvanceAreaItem2BesizerItems.begin();
	while (it != m_mapAdvanceAreaItem2BesizerItems.end())
	{
		if (it->first != pAdvaceAreaItem)
		{
			if (it->second.contains(pItem))
			{//存在
				return true;
			}
		}
		++it;
	}
	return false; //不存在
}

QGraphicsItem* DLCustomScene::FindItem(QPointF pos, int type)
{
	QGraphicsItem *node = NULL;
	QList<QGraphicsItem*> item_list = items(pos);
	foreach(QGraphicsItem *item, item_list) {
		if (item->type() == type)
		{
			node = item;
			break;
		}
	}
	return node;
}


QGraphicsItem* DLCustomScene::find_advancedpoint(int id)
{
	QGraphicsItem* item = NULL;
	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedPoints.find(id);
	if (it != m_mapAdvancedPoints.end()) {
		item = it->second;
	}
	return item;
}


// QList<QGraphicsItem*> DLCustomScene::GetDeviceList() const
// {
// 	QList<QGraphicsItem*> item_list;
// 	return item_list;
// }


int DLCustomScene::GetLandmarkId()
{
	return (m_iLandmarkId++);
}


int DLCustomScene::GetBezierId()
{
	return (++m_iBezierId);
}


int DLCustomScene::GetAdvancedAreaId()
{
	return (++m_iAdvancedAreaId);
}


//bool DLCustomScene::normalPoint_optimization()
//{
//	return is_normalPoint_optimization_;
//}


void DLCustomScene::InitLandmarkId()
{
	if (m_mapAdvancedPoints.size() > 0)
	{
		std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedPoints.end();
		--it;
		m_iLandmarkId = it->first;
		m_iLandmarkId++;
	}
	else
	{
		m_iLandmarkId = 0;
	}
}


void DLCustomScene::InitBezierId()
{
	if (m_mapAdvancedCurves.size() > 0)
	{
		std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedCurves.end();
		--it;
		m_iBezierId = it->first;
		++m_iBezierId;
	}
	else
	{
		m_iBezierId = 0;
	}

// 	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedCurves.begin();
// 
// 	int id = -1;
// 	while (it != m_mapAdvancedCurves.end()) {
// 		id = it->first;
// 		it++;
// 	}
// 	m_iBezierId = id;
}


void DLCustomScene::InitAdvancedAreaId()
{
	if (m_mapAdvancedAreas.size() > 0)
	{
		std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedAreas.end();
		--it;
		m_iAdvancedAreaId = it->first;
		++m_iAdvancedAreaId;
	}
	else
	{
		m_iAdvancedAreaId = 0;
	}

// 	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedAreas.begin();
// 
// 	int id = -1;
// 	while (it != m_mapAdvancedAreas.end()) {
// 		id = it->first;
// 		it++;
// 	}
// 	m_iAdvancedAreaId = id;
}


void DLCustomScene::remove_advancedpoint(int id)
{
	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedPoints.find(id);
	if (it != m_mapAdvancedPoints.end()) {
		removeItem(it->second);
		update();
		m_mapAdvancedPoints.erase(it);
	}
}


void DLCustomScene::remove_advancedcurve(int id)
{
	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedCurves.find(id);
	if (it != m_mapAdvancedCurves.end()) {
		removeItem(it->second);
		update();
		m_mapAdvancedCurves.erase(it);
	}
}


// void DLCustomScene::remove_point(int id)
// {
// }


void DLCustomScene::removeall(bool isOnlyLoadLm)
{
	m_dCoordinateAngle = 0;
	m_ptTransformPoint.setX(0);
	m_ptTransformPoint.setY(0);
	m_ptBgCenterPoint = QPointF(0.0, 0.0);
	m_rBgWidth = 0.0;
	m_rBgHeight = 0.0;
	m_strHighImagePath = "";

	m_iLandmarkId = 0;
	m_iBezierId = -1;
	m_lstAdvancedObjectDefines.clear();
	m_mapNormalPos.clear();
	m_mapNormalLines.clear();
	m_mapAdvancedPoints.clear();
	m_mapAdvancedLines.clear();
	m_mapAdvancedCurves.clear();
	m_mapAdvancedAreas.clear();
	m_vBackstagemapPaths.clear();

	if (NULL != m_pLoadPixmapItem)
	{
		this->removeItem(m_pLoadPixmapItem);
		m_pLoadPixmapItem = NULL;
	}

	if (isOnlyLoadLm == false) {
		m_mapDeviceAreas.clear();
		foreach(QGraphicsItem* item, this->items()) {
			if ((DLPlatformItem*)item == m_pPlatformItem || (DLLaserItem*)item == m_pLaserItem || (DLPathItem*)item == m_pPathItem || (DLRobotItem*)item == m_pRobotItem || (DLCoordinateItem*)item == m_pWorldCoordinateItem || (QGraphicsLineItem*)item == m_pVLineItem || (QGraphicsLineItem*)item == m_pHLineItem || (DLGridItem*)item == m_pGridItem) {
				continue;
			}
			else {
				this->removeItem(item);
			}
		}
	}
	else {
		foreach(QGraphicsItem* item, this->items()) {
			if ((DLPlatformItem*)item == m_pPlatformItem || item->type() == HShape::DEVICEAREAITEM || (DLLaserItem*)item == m_pLaserItem || (DLPathItem*)item == m_pPathItem || (DLRobotItem*)item == m_pRobotItem || (DLCoordinateItem*)item == m_pWorldCoordinateItem || (QGraphicsLineItem*)item == m_pVLineItem || (QGraphicsLineItem*)item == m_pHLineItem || (DLGridItem*)item == m_pGridItem) {
				continue;
			}
			else {
				this->removeItem(item);
			}
		}
	}
	update();
	QPixmapCache::clear();
}


void DLCustomScene::contextMenuEvent(QGraphicsSceneContextMenuEvent *contextMenuEvent)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType)
	{
		QTransform tran;
		QGraphicsItem *item = NULL;
		item = itemAt(contextMenuEvent->scenePos(), tran);

		if (item != NULL && HShape::LANDMARKITEM == item->type())
		{
			m_pCurrentItem = item;

			if (!((DLLandmarkItem*)item)->isSlelectState())
			{
				((DLLandmarkItem*)item)->setSelecteState(true);
				m_lstSelectIndexs.push_back(((DLLandmarkItem*)item)->id());
			}
			QString landmart_id = QString::number(((DLLandmarkItem*)item)->id());
			QMenu popMenu;
			QAction *back_home_action = new QAction("一键返航", this);
			QAction *robot_move_action = new QAction("行走", this);
			QAction *robotCreatePathAction = new QAction(("生成巡检任务"), this);
			QAction *view_device_action = new QAction(("查看巡检设备"), this);
			QAction *pRelacationAction = new QAction("重定位", this);
			popMenu.addAction(back_home_action);
			popMenu.addAction(robot_move_action);
			popMenu.addAction(robotCreatePathAction);
			popMenu.addAction(view_device_action);
			popMenu.addAction(pRelacationAction);

			connect(back_home_action, SIGNAL(triggered()), this, SLOT(slot_on_back_home()));
			connect(robot_move_action, SIGNAL(triggered()), this, SLOT(slot_on_robot_move()));
			connect(robotCreatePathAction, SIGNAL(triggered()), this, SLOT(CreatePatrolTaskSlot()));
			connect(view_device_action, SIGNAL(triggered()), this, SLOT(ReadViewDeviceStateSlot()));
			connect(pRelacationAction, SIGNAL(triggered()), this, SLOT(RelacationActionSlot()));
			popMenu.exec(QCursor::pos()); // 菜单出现的位置为当前鼠标的位置
		}
	}
	else if (DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{//
		QTransform tran;
		QGraphicsItem *item = NULL;
		item = itemAt(contextMenuEvent->scenePos(), tran);

		if (item != NULL && HShape::LANDMARKITEM == item->type())
		{
			m_pCurrentItem = item;
			m_strCurrentLandmarkId = QString::number(((DLLandmarkItem*)item)->id());
			QMenu popMenu;
			QAction *landmark_id_action = new QAction(m_strCurrentLandmarkId, this);
			QAction *robot_move_action = new QAction("行走", this);
			QAction *view_detail_info = new QAction("查看点位详情", this);

			popMenu.addAction(landmark_id_action);
			popMenu.addAction(robot_move_action);
			popMenu.addAction(view_detail_info);

			connect(robot_move_action, SIGNAL(triggered()), this, SLOT(slot_on_robot_move()));
			connect(view_detail_info, SIGNAL(triggered()), this, SLOT(slot_on_view_detail_info()));
			popMenu.exec(QCursor::pos()); // 菜单出现的位置为当前鼠标的位置
		}
	}
	else if (DL_COLLECT_EDIT_TYPE == m_iCustomSceneType)
	{
		QTransform tran;
		QGraphicsItem *item = NULL;
		item = itemAt(contextMenuEvent->scenePos(), tran);

		//if (item == NULL) {
		//	QMenu popMenu;
		//	QAction *point_hide_action = new QAction("隐藏部分轮廓点", this);
		//	QAction *point_show_action = new QAction("显示全部轮廓点", this);
		//	popMenu.addAction(point_hide_action);
		//	popMenu.addAction(point_show_action);

		//	connect(point_hide_action, SIGNAL(triggered()), this, SLOT(slot_on_hide_part_point()));
		//	connect(point_show_action, SIGNAL(triggered()), this, SLOT(slot_on_show_all_point()));

		//	popMenu.exec(QCursor::pos());       // 菜单出现的位置为当前鼠标的位置
		//}


		if (item != NULL && HShape::LANDMARKITEM == item->type())
		{

			m_pCurrentItem = item;
			QString id = QString::number(((DLLandmarkItem*)item)->id());

			QMenu popMenu;
			QAction *back_home_action = new QAction("一键返航", this);
			QAction *robot_move_action = new QAction("行走", this);
			QAction *line_hide_action = new QAction("隐藏辅助线", this);
			QAction *line_show_action = new QAction("显示辅助线", this);


			QAction *target_id_action = new QAction(id, this);
			popMenu.addAction(target_id_action);
			popMenu.addAction(robot_move_action);
			popMenu.addAction(line_hide_action);
			popMenu.addAction(line_show_action);
			if (((DLLandmarkItem*)item)->type() == HShape::LANDMARKITEM) {
				popMenu.addAction(back_home_action);
			}


			connect(back_home_action, SIGNAL(triggered()), this, SLOT(slot_on_back_home()));
			connect(robot_move_action, SIGNAL(triggered()), this, SLOT(slot_on_robot_move()));
			connect(line_hide_action, SIGNAL(triggered()), this, SLOT(slot_on_hide_auxLine()));
			connect(line_show_action, SIGNAL(triggered()), this, SLOT(slot_on_show_auxLine()));
			popMenu.exec(QCursor::pos());       // 菜单出现的位置为当前鼠标的位置
		}

		if (item != NULL && HShape::DEVICEAREAITEM == item->type())
		{
			m_pCurrentItem = item;
			QMenu popMenu;
			QAction *devicearea_dismoveable_action = new QAction("固定", this);
			QAction *devicearea_moveable_action = new QAction("取消固定", this);
			QAction *devicearea_load_pixmap_action = new QAction("加载图片", this);
			popMenu.addAction(devicearea_dismoveable_action);
			popMenu.addAction(devicearea_moveable_action);
			popMenu.addAction(devicearea_load_pixmap_action);

			connect(devicearea_moveable_action, SIGNAL(triggered()), this, SLOT(slot_on_deviceAre_moveable()));
			connect(devicearea_dismoveable_action, SIGNAL(triggered()), this, SLOT(slot_on_deviceArea_dismoveable()));
			connect(devicearea_load_pixmap_action, SIGNAL(triggered()), this, SLOT(slot_on_deviceAre_loadMap()));
			popMenu.exec(QCursor::pos());       // 菜单出现的位置为当前鼠标的位置
		}
	}
}

void DLCustomScene::mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType)
	{
		if (event->button() == Qt::LeftButton  && m_bIsDrawState)
		{
			bool bIsOk = MessageBox();					//消息弹框
			if (bIsOk)
			{//发送数据给core，退出绘制模式
				SendData2Core();
				m_bIsDrawState = false;
				emit ExitDrawStateSignal();		//退出信号
				DrawCurPath();
			}
		}
	}
	else if (DL_COLLECT_EDIT_TYPE == m_iCustomSceneType || DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{
		QTransform tran;
		QGraphicsItem *item = NULL;
		item = itemAt(event->scenePos(), tran);
		if (Qt::ControlModifier == event->modifiers())
		{//control 批量处理
			//int iBezierItemCount = 0;
			if (item)
				item->setSelected(true);
			QList<DLBezierItem *> lstParentItems;
			QList<QGraphicsItem*> lstItems = this->selectedItems();
			foreach(QGraphicsItem *pItem, lstItems)
			{
				DLBezierItem *pDLBezierItem = dynamic_cast<DLBezierItem *>(pItem);
				if (pDLBezierItem)
				{
					lstParentItems.push_back(pDLBezierItem);
					//++iBezierItemCount;
				}
			}
			if (lstParentItems.size() > 1)
			{//批量处理
				m_pBezierPropertyWidget->move(QCursor::pos());
				m_pBezierPropertyWidget->SetBatchSetParents(lstParentItems);
				m_pBezierPropertyWidget->SetBatchSet(true);
				m_pBezierPropertyWidget->InitBatchProperty();
				m_pBezierPropertyWidget->show();

				return QGraphicsScene::mouseDoubleClickEvent(event);
			}
		}

		if (item != NULL && HShape::LANDMARKITEM == item->type())
		{
			m_pLandmarkPropertyWidget->setParent((DLLandmarkItem*)item);
			m_pLandmarkPropertyWidget->show();
			m_pLandmarkPropertyWidget->move(calculate_window_point(m_pLandmarkPropertyWidget->size()));
			m_pLandmarkPropertyWidget->setLandMarkProperty();

		}
		if (item != NULL && HShape::DEVICEAREAITEM == item->type())
		{
			m_pDeviceAreaPropertyWidget->setParent((DLDeviceAreaItem*)item);
			m_pDeviceAreaPropertyWidget->move(calculate_window_point(m_pDeviceAreaPropertyWidget->size()));
			m_pDeviceAreaPropertyWidget->setDeviceAreaProperty();
			m_pDeviceAreaPropertyWidget->show();
		}
		if (item != NULL && HShape::BEZIERITEM == item->type())
		{
			m_pBezierPropertyWidget->setParent((DLBezierItem*)item);
			m_pBezierPropertyWidget->move(calculate_window_point(m_pBezierPropertyWidget->size()));
			m_pBezierPropertyWidget->setBezierProperty();
			m_pBezierPropertyWidget->SetBatchSet(false);
			m_pBezierPropertyWidget->show();
		}
		//去除高级区域属性框
// 		if (item != NULL && HShape::ADVANCEDAREA == item->type())
// 		{
// 			m_pAdvancedAreaPropertyWidget->setParent((DLAdvancedAreaItem*)item);
// 			m_pAdvancedAreaPropertyWidget->move(calculate_window_point(m_pLandmarkPropertyWidget->size()));
// 			m_pAdvancedAreaPropertyWidget->setAdvancedAreaProperty();
// 			m_pAdvancedAreaPropertyWidget->show();
// 		}

	}

}


void DLCustomScene::slot_on_back_home()
{
	WHEEL_BACK_TO_CORE_SOCKET.robot_control_back_to_charge();
}


void DLCustomScene::DrawStateChangedSlot(bool bIsDrawState)
{
	m_bIsDrawState = bIsDrawState;
	m_lstPaths.clear();								//清空路径
	m_lstPaths.push_back(m_pRobotItem->pos());
}

void DLCustomScene::RelacationActionSlot()
{//重定位槽函数
// 	DLLandmarkItem *pLandmarkItem = dynamic_cast<DLLandmarkItem *>(m_pCurrentItem);
// 	if (NULL != pLandmarkItem)
// 	{
// 		QString id = QString::number(pLandmarkItem->id());
// 		WHEEL_BACK_TO_CORE_SOCKET.robot_control_gotarget_req(id.toLocal8Bit().constData(), 0);
// 	}

	int iRet = QMessageBox::warning(this->views().at(0), "警告", "是否重定位？", QMessageBox::Ok | QMessageBox::Cancel);

	if (QMessageBox::Ok == iRet)
	{
		DLLandmarkItem *pLandmarkItem = dynamic_cast<DLLandmarkItem *>(m_pCurrentItem);
		if (nullptr != m_pOperator && nullptr != pLandmarkItem)
		{
			double dAngle = pLandmarkItem->angle();
			m_pOperator->RobotRecation(pLandmarkItem->pos(), dAngle);
		}
	}

	
}

void DLCustomScene::slot_on_map_reset(bool)
{
	m_bIsStart = false;
	std::vector<int> patrol_points;
	std::vector<QPoint> work_path;
	m_iTaskPatrolPoint = 0;
	SetWorkPath(work_path, patrol_points, false);
	ROS_INFO("DLBackStageMapScene map reset");
}

void DLCustomScene::FlushRobotSlot(WheelRobotTaskCurrentPointStatus currentPointStatus)
{//客户端刷新车的位置

	//计算小车在路径的位置和角度（目前是通过百分比映射关系）
	bool isPositive;
	QPointF startPos, c1Pos, c2Pos, endPos, point;

	if (currentPointStatus.start_point == currentPointStatus.end_point) {
		return;
	}

	QGraphicsItem *bezier_item = find_advancedCurve(currentPointStatus.start_point, currentPointStatus.end_point, isPositive);
	if (bezier_item != NULL) {

		if (isPositive) {
			startPos = ((DLBezierItem*)bezier_item)->start_pos();
			c1Pos = ((DLBezierItem*)bezier_item)->c1_pos();
			c2Pos = ((DLBezierItem*)bezier_item)->c2_pos();
			endPos = ((DLBezierItem*)bezier_item)->end_pos();
		}
		else {
			endPos = ((DLBezierItem*)bezier_item)->start_pos();
			c2Pos = ((DLBezierItem*)bezier_item)->c1_pos();
			c1Pos = ((DLBezierItem*)bezier_item)->c2_pos();
			startPos = ((DLBezierItem*)bezier_item)->end_pos();
		}
	}

	//qDebug() << "===============start Point:" << startPos << ";c1 point:" << c1Pos << ";c2 point:" << c2Pos << "; end pos:" << endPos;

	QPainterPath current_bezier_path;
	current_bezier_path.moveTo(startPos);
	current_bezier_path.cubicTo(QPointF(c1Pos), QPointF(c2Pos), QPointF(endPos));
	point = current_bezier_path.pointAtPercent(currentPointStatus.percent);
	double angle = cal_car_angle(startPos, c1Pos, c2Pos, endPos, currentPointStatus.percent);

	//取数据
	int current_patrol_index = 0;
	m_mutexPatrolPoint.lock();
	current_patrol_index = m_iTaskPatrolPoint;
	m_mutexPatrolPoint.unlock();

	//qDebug() << "===================调用设置机器人节点坐标：" << point.x() << point.y() << angle;

	//刷新车的位置
	//SetRobotItemPropertry(point.x(), point.y(), angle);

	if (m_bIsStart && currentPointStatus.start_point != currentPointStatus.end_point)
	{
		if (m_pPathItem != NULL && currentPointStatus.percent > 0.00000) 
		{
			m_pPathItem->UpdatePath(currentPointStatus.start_point, currentPointStatus.end_point, currentPointStatus.percent, current_patrol_index);
		}
	}
	ROS_INFO("DLBackStageMapScene start index:%d, end index: %d, percentage:%f", currentPointStatus.start_point, currentPointStatus.end_point, currentPointStatus.percent);

}

void DLCustomScene::FlushPlatformSlot(WheelRobotRealtimeStatus status)
{
	double robot_angle = 0;
	float fPlatformAngle = status.ptzStatus.pan / 100;
	QPointF point = m_pRobotItem->scenePos();

	m_mutexLocation.lock();
	robot_angle = m_dRobotCurrentAngle * 180 / PI;
	m_mutexLocation.unlock();

	//云台位置换算成世界坐标系
	//platform_angle = 0 - (platform_angle + robot_angle);
	fPlatformAngle = -(robot_angle - fPlatformAngle );
	SetPlatformItemPropertry(point.x(), point.y(), fPlatformAngle);
}

void DLCustomScene::slot_on_add_patrol_point()
{
	HShape::Ellipse pro;
	if (abs(m_dRobotCurrentX) > 999999) {
		pro.angle_ = 0;
		pro.center_.x_ = 0;
		pro.center_.y_ = 0;
		pro.width_ = 20;
		pro.height_ = 20;
		pro.angle_ = 0;
	}
	else {
		pro.angle_ = m_dRobotCurrentAngle;
		pro.center_.x_ = m_dRobotCurrentX / m_rResolution;
		pro.center_.y_ = m_dRobotCurrentY / m_rResolution;
		pro.width_ = 20;
		pro.height_ = 20;
		pro.angle_ = 0;
	}

	qreal rRoundDistance = ROUND_DISTANCE / m_rResolution;		//图上距离
	QPainterPath path;
	QRectF rect(pro.center_.x_ - rRoundDistance / 2, pro.center_.y_ - rRoundDistance / 2, rRoundDistance, rRoundDistance);
	path.addEllipse(rect);
	QList<QGraphicsItem *> lstItems = this->items(path);
	for (int index = 0; index < lstItems.size(); ++index)
	{
		DLLandmarkItem *pDLLandmarkItem = dynamic_cast<DLLandmarkItem *>(lstItems[index]);
		if (NULL != pDLLandmarkItem)
		{
			m_pReaderInfoWidget->set_info(MapHandleInfo(1, "添加巡检点失败！", "巡检点添加信息如下"));
			m_pReaderInfoWidget->show();
			return;
		}
	}

	int id = GetLandmarkId();
	QString name = QString::number(id);
	DLLandmarkItem* item = new DLLandmarkItem(pro, id, name.toStdString());
	add_advancedPoint(item);
}

void DLCustomScene::slot_on_view_detail_info()
{
	//通过巡检id号，检索设备列表
	QList<wheelDeviceDetailMsg> data;
	QList<DeviceDetail> device_data;
	WHEEL_ROBOT_DB.getDeviceListForPointId(m_strCurrentLandmarkId, data);
	for (int i = 0; i < data.size(); ++i)
	{
		DeviceDetail device_info;
		device_info.deviceUUid = data[i].deviceUUid;
		device_info.VoltageLevel = data[i].VoltageLevel;
		device_info.equipmentInterval = data[i].equipmentInterval;
		device_info.deviceArea = data[i].deviceArea;
		device_info.deviceType = data[i].deviceType;
		device_info.subDevice = data[i].subDevice;
		device_info.devicePointType = data[i].devicePointType;
		device_data.push_back(device_info);
	}
	m_pPointInfoViewer->loadData(device_data);
	m_pPointInfoViewer->show();
}

void DLCustomScene::CreatePatrolTaskSlot()
{
	QStringList device_list;

	for (int i = 0; i < m_lstSelectIndexs.size(); ++i)
	{
		//通过巡检id号，检索设备列表
		WHEEL_ROBOT_DB.getDeviceListForPointId(QString::number(m_lstSelectIndexs[i]), device_list);
	}

	if (device_list.size() == 0)
	{
		QMessageBox::information(NULL, "消息提示框", "下发任务失败！设备列表为空！\n检查是否误操作！检查点位是否有数据！", 
			QMessageBox::Yes);
		return;
	}

	WheelTaskEditStruct task;
	task.task_edit_date = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
	task.task_edit_name = "instant task";
	task.task_edit_type_id = WHEEL_USER_DEFINED_TASK;
	task.task_edit_uuid = QUuid::createUuid().toString().remove('{').remove('}').remove('-');
	WHEEL_BACK_TO_CORE_SOCKET.robot_task_edit_insert_from_map_req(task, device_list);

	ROS_INFO("DLBackStageMapScene assign task: %s, device size:%d", task.task_edit_uuid.toStdString(), device_list.size());
	m_lstSelectIndexs.clear();
}

void DLCustomScene::ReadViewDeviceStateSlot()
{
	QPoint point = QCursor::pos();

	if (NULL == m_pCheckDeviceDlg)
	{
		m_pCheckDeviceDlg = new CheckDeviceDlg();
		m_pCheckDeviceDlg->setWindowTitle("巡检设备");
	}
	;  //选中设备id
	QStringList lstEqiupments;
	QList<wheelDeviceDetailMsg> data;
	foreach(int iCurID, m_lstSelectIndexs)
	{
		QString strCurID = QString::number(iCurID);
		data.clear();
		WHEEL_ROBOT_DB.getDeviceListForPointId(strCurID, data);
		for (int i = 0; i < data.size(); ++i)
		{
			// 			DeviceDetail device_info;
			// 			device_info.deviceUUid = data[i].deviceUUid;
			// 			device_info.VoltageLevel = data[i].VoltageLevel;
			// 			device_info.equipmentInterval = data[i].equipmentInterval;
			// 			device_info.deviceArea = data[i].deviceArea;
			// 			device_info.deviceType = data[i].deviceType;
			// 			device_info.subDevice = data[i].subDevice;
			// 			device_info.devicePointType = data[i].devicePointType;
			// 			device_data.push_back(device_info);

			lstEqiupments << data[i].VoltageLevel + "-" + data[i].equipmentInterval + "-" + data[i].devicePointType;
		}
	}
	m_pCheckDeviceDlg->LoadData(lstEqiupments);

	m_pCheckDeviceDlg->move(point);
	m_pCheckDeviceDlg->show();
	//device_viewer_->move(point);
	//device_viewer_->show();

}

// void DLCustomScene::SendAreaPointsSlot(QPointF ptMinPoint, QPointF ptMaxPoint)
// {
// 
// 	if (NULL == m_pMutilNormalPointItem)
// 	{
// 		m_pMutilNormalPointItem = new DLMultiPointItem;
// 		this->add_normalPoint(m_pMutilNormalPointItem);
// 	}
// 	m_pMutilNormalPointItem->SetBoundingRect(ptMinPoint, ptMaxPoint);
// 	if (m_pReadBgMapThread)
// 	{
// 		m_pMutilNormalPointItem->SetAllAreaPoints(m_pReadBgMapThread->GetRectAreaPoints());
// 		m_pReadBgMapThread->Clear();
// 	}
// 	m_pMutilNormalPointItem->SetDisplayBackground(true);
// 	UpdateNormalBackground();
// 	//qDebug() << "============================读取地图的时间:" << m_time.elapsed();
// }

// void DLCustomScene::SelectItemChangeSlot(QGraphicsItem *pSelectItem)
// {
// // 	QList<QGraphicsItem *> lstItems = this->selectedItems();
// // 	foreach(QGraphicsItem *pItem, lstItems)
// // 	{
// // 		if (pItem && pSelectItem != pItem)
// // 		{
// // 			pItem->setSelected(false);
// // 		}
// // 	}
// 
// 	this->clearSelection();
// 	if (NULL != pSelectItem)
// 	{
// 		pSelectItem->setSelected(true);
// 	}
// 
// }

void DLCustomScene::slot_on_robot_move()
{
	DLLandmarkItem *pLandmarkItem = dynamic_cast<DLLandmarkItem *>(m_pCurrentItem);
	if (NULL != pLandmarkItem)
	{
		QString id = QString::number(pLandmarkItem->id());
		WHEEL_BACK_TO_CORE_SOCKET.robot_control_gotarget_req(id.toLocal8Bit().constData(), 0);
	}
}


void DLCustomScene::slot_on_deviceAre_moveable()
{
	DLDeviceAreaItem *pDeviceAreaItem = dynamic_cast<DLDeviceAreaItem *>(m_pCurrentItem);
	if (NULL != pDeviceAreaItem)
	{
		pDeviceAreaItem->set_deviceArea_moveable(true);
	}
}


void DLCustomScene::slot_on_deviceArea_dismoveable()
{
	DLDeviceAreaItem *pDeviceAreaItem = dynamic_cast<DLDeviceAreaItem *>(m_pCurrentItem);
	if (NULL != m_pCurrentItem)
	{
		pDeviceAreaItem->set_deviceArea_moveable(false);
	}
}


void DLCustomScene::slot_on_hide_auxLine()
{
	for (std::map<int, QGraphicsItem*>::iterator it = m_mapNormalLines.begin(); it != m_mapNormalLines.end(); ++it)
	{
		DLSegmentItem *pSegmentItem = dynamic_cast<DLSegmentItem*>(it->second);
		if (NULL != pSegmentItem)
		{
			pSegmentItem->setVisible(false);
		}
	}
}


void DLCustomScene::slot_on_show_auxLine()
{
	for (std::map<int, QGraphicsItem*>::iterator it = m_mapNormalLines.begin(); it != m_mapNormalLines.end(); ++it)
	{
		DLSegmentItem *pSegmentItem = dynamic_cast<DLSegmentItem *>(it->second);
		if (NULL != pSegmentItem)
		{
			pSegmentItem->setVisible(true);
		}
	}
}


//void DLCustomScene::slot_on_show_all_point()
//{
//	is_normalPoint_optimization_ = false;
//}
//
//
//void DLCustomScene::slot_on_hide_part_point()
//{
//	is_normalPoint_optimization_ = true;
//}



void DLCustomScene::slot_on_deviceAre_loadMap()
{
	QString fileName = QFileDialog::getOpenFileName(NULL, tr("Open File"), tr("d:/"), tr("Images (*.png)"));
	if (fileName.isEmpty()) {
		return;
	}
	DLDeviceAreaItem *pDLDeviceAreaItem = dynamic_cast<DLDeviceAreaItem *>(m_pCurrentItem);
	if (NULL != pDLDeviceAreaItem)
	{
		pDLDeviceAreaItem->loadPixMap(fileName);
	}
}


void DLCustomScene::cal_nearset_line(QGraphicsItem *item, int &index_h, int &part_h, int &index_v, int &part_v)
{
	if (item == NULL)
		return;

	if (HShape::DEVICEAREAITEM == item->type()) {
		double center_x = ((DLDeviceAreaItem*)item)->get_pro().center_.x_;
		double center_y = ((DLDeviceAreaItem*)item)->get_pro().center_.y_;
		double width = ((DLDeviceAreaItem*)item)->get_pro().width_;
		double height = ((DLDeviceAreaItem*)item)->get_pro().height_;
		QLineF HLine_1(QPointF(center_x - 20000, center_y - height / 2), QPointF(center_x + 20000, center_y - height / 2));
		QLineF HLine_2(QPointF(center_x - 20000, center_y + height / 2), QPointF(center_x + 20000, center_y + height / 2));
		QLineF VLine_1(QPointF(center_x - width / 2, center_y - 20000), QPointF(center_x - width / 2, center_y + 20000));
		QLineF VLine_2(QPointF(center_x + width / 2, center_y - 20000), QPointF(center_x + width / 2, center_y + 20000));

		int shortest_top_H = 9999999, shortest_top_h_index = -1, shortest_top_h_part = -1;
		for (std::map<int, QLineF>::iterator it = m_mapTopHLines.begin(); it != m_mapTopHLines.end(); ++it)
		{
			int temp = abs(it->second.p1().y() - HLine_1.p1().y());
			if (temp < shortest_top_H) {
				shortest_top_H = temp;
				if (shortest_top_H < CATCH_FACTOR) {
					shortest_top_h_index = it->first;
					shortest_top_h_part = 0;
				}
			}
		}

		int shortest_bottom_H = 9999999, shortest_bottom_h_index = -1, shortest_bottom_h_part = -1;
		for (std::map<int, QLineF>::iterator it = m_mapBottomHLines.begin(); it != m_mapBottomHLines.end(); ++it)
		{
			int temp = abs(it->second.p2().y() - HLine_2.p2().y());
			if (temp < shortest_bottom_H) {
				shortest_bottom_H = temp;
				if (shortest_bottom_H < CATCH_FACTOR) {
					shortest_bottom_h_index = it->first;
					shortest_bottom_h_part = 1;
				}
			}
		}

		int shortest_left_V = 9999999, shortest_left_v_index = -1, shortest_left_v_part = -1;
		for (std::map<int, QLineF>::iterator it = m_mapRightVLines.begin(); it != m_mapRightVLines.end(); ++it)
		{
			int temp = abs(it->second.p1().x() - VLine_2.p1().x());
			if (temp < shortest_left_V) {
				shortest_left_V = temp;
				if (shortest_left_V < CATCH_FACTOR) {
					shortest_left_v_index = it->first;
					shortest_left_v_part = 2;
				}
			}
		}

		int shortest_right_V = 9999999, shortest_right_v_index = -1, shortest_right_v_part = -1;
		for (std::map<int, QLineF>::iterator it = m_mapLeftVLines.begin(); it != m_mapLeftVLines.end(); ++it)
		{
			int temp = abs(it->second.p2().x() - VLine_1.p2().x());
			if (temp < shortest_right_V) {
				shortest_right_V = temp;
				if (shortest_right_V < CATCH_FACTOR) {
					shortest_right_v_index = it->first;
					shortest_right_v_part = 3;
				}
			}
		}

		if (shortest_top_H > shortest_bottom_H) {
			index_h = shortest_bottom_h_index;
			part_h = shortest_bottom_h_part;
		}
		else {
			index_h = shortest_top_h_index;
			part_h = shortest_top_h_part;
		}

		if (shortest_left_V > shortest_right_V) {
			index_v = shortest_right_v_index;
			part_v = shortest_right_v_part;
		}
		else {
			index_v = shortest_left_v_index;
			part_v = shortest_left_v_part;
		}
	}
	else if (HShape::LANDMARKITEM == item->type()) {
		double center_x = ((DLLandmarkItem*)item)->get_pro().center_.x_;
		double center_y = ((DLLandmarkItem*)item)->get_pro().center_.y_;
		double width = ((DLLandmarkItem*)item)->get_pro().width_;
		double height = ((DLLandmarkItem*)item)->get_pro().height_;
		QLineF HLine_1(QPointF(center_x - 20000, center_y), QPointF(center_x + 20000, center_y));
		QLineF VLine_1(QPointF(center_x, center_y - 20000), QPointF(center_x, center_y + 20000));

		int shortest_H = 9999999, shortest_h_index = -1;
		for (std::map<int, QLineF>::iterator it = m_mapLandmarkHLines.begin(); it != m_mapLandmarkHLines.end(); ++it)
		{
			int temp = abs(it->second.p1().y() - HLine_1.p1().y());
			if (temp < shortest_H) {
				shortest_H = temp;
				if (shortest_H < CATCH_FACTOR) {
					shortest_h_index = it->first;
				}
			}
		}
		index_h = shortest_h_index;

		int shortest_V = 9999999, shortest_v_index = -1;
		for (std::map<int, QLineF>::iterator it = m_mapLandmarkVLines.begin(); it != m_mapLandmarkVLines.end(); ++it)
		{
			int temp = abs(it->second.p2().x() - VLine_1.p2().x());
			if (temp < shortest_V) {
				shortest_V = temp;
				if (shortest_V < CATCH_FACTOR) {
					shortest_v_index = it->first;
				}
			}
		}
		index_v = shortest_v_index;

	}



}


void DLCustomScene::onHideItemControlPoint()
{
	for (std::map<int, QGraphicsItem*>::iterator it = m_mapNormalLines.begin(); it != m_mapNormalLines.end(); ++it)
	{
		DLSegmentItem *pDLSegmentItem = dynamic_cast<DLSegmentItem *>(it->second);
		if (NULL != pDLSegmentItem)
		{
			pDLSegmentItem->set_control_point_visible(false);
		}
	}
	for (std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedAreas.begin(); it != m_mapAdvancedAreas.end(); ++it)
	{
		DLAdvancedAreaItem *pDLAdvancedAreaItem = dynamic_cast<DLAdvancedAreaItem *>(it->second);
		if (NULL != pDLAdvancedAreaItem)
		{
			pDLAdvancedAreaItem->set_control_point_visible(false);
		}
	}
	for (std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedCurves.begin(); it != m_mapAdvancedCurves.end(); ++it)
	{
		DLBezierItem *pDLBezierItem = dynamic_cast<DLBezierItem *>(it->second);
		if (NULL != pDLBezierItem)
		{
			pDLBezierItem->set_control_point_visible(false);
		}
	}
	for (std::map<int, QGraphicsItem*>::iterator it = m_mapDeviceAreas.begin(); it != m_mapDeviceAreas.end(); ++it)
	{
		DLDeviceAreaItem *pDLDeviceAreaItem = dynamic_cast<DLDeviceAreaItem *>(it->second);
		if (NULL != pDLDeviceAreaItem)
		{
			pDLDeviceAreaItem->set_control_point_visible(false);
		}
	}
	for (std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedPoints.begin(); it != m_mapAdvancedPoints.end(); ++it)
	{
		//((DLLandmarkItem*)it->second)->set_control_point_visible(false);
	}
}


void DLCustomScene::onCatchItempoistion(QGraphicsItem *item, QPointF point)
{
	if (HShape::DEVICEAREAITEM == item->type()) {
		int index_h = -1, part_h = -1, index_v = -1, part_v = -1;
		cal_nearset_line(this->itemAt(point, QTransform()), index_h, part_h, index_v, part_v);

		if (part_h == 0) {
			std::map<int, QLineF>::iterator it = m_mapTopHLines.find(index_h);
			if (it != m_mapTopHLines.end()) {
				m_pHLineItem->setLine(it->second);
				m_pHLineItem->show();
			}
		}
		else if (part_h == 1) {
			std::map<int, QLineF>::iterator it = m_mapBottomHLines.find(index_h);
			if (it != m_mapBottomHLines.end()) {
				m_pHLineItem->setLine(it->second);
				m_pHLineItem->show();
			}
		}
		else {
			m_pHLineItem->hide();
		}

		if (part_v == 2) {
			std::map<int, QLineF>::iterator it = m_mapRightVLines.find(index_v);
			if (it != m_mapRightVLines.end()) {
				m_pVLineItem->setLine(it->second);
				m_pVLineItem->show();
			}
		}
		else if (part_v == 3) {
			std::map<int, QLineF>::iterator it = m_mapLeftVLines.find(index_v);
			if (it != m_mapLeftVLines.end()) {
				m_pVLineItem->setLine(it->second);
				m_pVLineItem->show();
			}
		}
		else {
			m_pVLineItem->hide();
		}
	}

	if (HShape::LANDMARKITEM == item->type()) {
		int index_h = -1, part_h = -1, index_v = -1, part_v = -1;
		cal_nearset_line(this->itemAt(point, QTransform()), index_h, part_h, index_v, part_v);

		std::map<int, QLineF>::iterator it_h = m_mapLandmarkHLines.find(index_h);
		if (it_h != m_mapLandmarkHLines.end()) {
			m_pHLineItem->setLine(it_h->second);
			m_pHLineItem->show();
		}
		else {
			m_pHLineItem->hide();
		}

		std::map<int, QLineF>::iterator it_v = m_mapLandmarkVLines.find(index_v);
		if (it_v != m_mapLandmarkVLines.end()) {
			m_pVLineItem->setLine(it_v->second);
			m_pVLineItem->show();
		}
		else {
			m_pVLineItem->hide();
		}
	}
}


bool DLCustomScene::GetItemIsChanged()
{
	bool bPointIsChanged = false;
	QList<QGraphicsItem *> lstItems = this->items();
	foreach(QGraphicsItem *pItem, lstItems)
	{
		HQShapeItem *pShapeItem = dynamic_cast<HQShapeItem *>(pItem);
		if (pShapeItem)
		{
			bPointIsChanged = pShapeItem->PointIsChanged();
			if (bPointIsChanged)
			{
				return bPointIsChanged;
			}
		}
	}
	return bPointIsChanged;
}


// int DLCustomScene::button()
// {
// 	return button_type_;
// }


// void DLCustomScene::GetNormalPosList(std::map<int, QPointF> &point_map)
// {
// 	point_map = m_mapNormalPos;
// }


// void DLCustomScene::SetMutilSelectRect(const QRectF &rect)
// {
// 	m_rectMutilArea = rect;
// }


void DLCustomScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType)
	{
		//m_pDeviceViewer->hide();
		if (!m_bIsDrawState)
		{//不是制定线
			QList<QGraphicsItem*> item_list = items(event->scenePos());
			if (event->button() == Qt::LeftButton)
			{

				m_ptStartPos = event->scenePos();
				m_bPressDown = true;
				if (item_list.size() < 1)
				{
					m_pSelectItem = new QGraphicsRectItem(QRectF(event->scenePos(), QSizeF(0, 0)).normalized());
					m_pSelectItem->setPen(Qt::NoPen);
					m_pSelectItem->setOpacity(0.5);
					m_pSelectItem->setBrush(QColor(133, 177, 222));
					m_pSelectItem->setZValue(50);
					this->addItem(m_pSelectItem);
				}
				else {
					if (QApplication::keyboardModifiers() == Qt::ControlModifier)
					{
						for (int i = 0; i < item_list.size(); ++i)
						{
							DLLandmarkItem *pDLLandmarkItem = dynamic_cast<DLLandmarkItem *>(item_list[i]);
							if (pDLLandmarkItem)
							{
								int id = pDLLandmarkItem->id();
								pDLLandmarkItem->setSelecteState(true);
								m_lstSelectIndexs.push_back(id);
								ROS_ERROR("add point id:%d", id);
							}
						}
					}
					else
					{
						std::map<int, QGraphicsItem*>::iterator itItem = m_mapAdvancedPoints.begin();
						while (itItem != m_mapAdvancedPoints.end())
						{
							DLLandmarkItem *pDLLandmarkItem = dynamic_cast<DLLandmarkItem *>(itItem->second);
							if (pDLLandmarkItem)
							{
								pDLLandmarkItem->setSelecteState(false);
								pDLLandmarkItem->update();
							}
							++itItem;
						}
						m_lstSelectIndexs.clear();
					}
				}
			}
			else if (event->button() == Qt::RightButton && item_list.size() == 0) {
				m_lstSelectIndexs.clear();
				for (auto itor = m_mapAdvancedPoints.begin(); itor != m_mapAdvancedPoints.end(); ++itor)
				{
					((DLLandmarkItem*)itor->second)->setSelecteState(false);
				}
				ROS_ERROR("clear points");
			}
		}
	}
	else if (DL_COLLECT_EDIT_TYPE == m_iCustomSceneType || DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{

		m_bIsPress = true;
		m_ptStartPoint = event->scenePos();
		m_ptEndPoint = event->scenePos();
	

		if (event->button() == Qt::RightButton)
		{
			if (m_pOperator->type() == DLOperator::TYPE_COORDINATE_TRANSFORM) {
// 				if (m_mapNormalPos.size() != 0) {
// 					m_lineStart.setP2(event->scenePos());
// 					m_lineStart.setP1(m_pMutilNormalPointItem->scenePos());
// 					m_lineEnd.setP1(m_pMutilNormalPointItem->scenePos());
// 				}
// 				m_iButtonType = 2;
				QGraphicsScene::mousePressEvent(event);
			}
			else if (DLOperator::TYPE_OVERFITTING_SELECT_POINT == m_pOperator->type())
			{//选择点
				QString strPointIDs = "";
				bool bIsFirst = true;
				m_vOverFittingPointsItems.clear();
				QList<QGraphicsItem *> lstSelectItem = this->selectedItems();
				foreach(QGraphicsItem *pItem, lstSelectItem)
				{
					DLLandmarkItem *pLandmarkItem = dynamic_cast<DLLandmarkItem *>(pItem);
					if (NULL != pLandmarkItem)
					{
						if (bIsFirst)
						{
							bIsFirst = false;
							strPointIDs += QString::number(pLandmarkItem->id());
						}
						else
						{
							strPointIDs += ",";
							strPointIDs += QString::number(pLandmarkItem->id());
						}
						m_vOverFittingPointsItems.push_back(pLandmarkItem);
					}
				}

				if (NULL != m_pOverFittingDlg)
				{
					m_pOverFittingDlg->SetSelectPointIDs(strPointIDs);
					m_pOverFittingDlg->show();
				}
			}
		}
		else if (event->button() == Qt::LeftButton)
		{
			m_pOperator->mousePressEvent(event);							//处理事件，创建item
			if (m_pOperator->type() == DLOperator::TYPE_SELECT) 
			{

				if (this->itemAt(event->scenePos(), QTransform()) == NULL)
				{
					onHideItemControlPoint();
				}

				m_mapLandmarkHLines.clear();
				m_mapLandmarkVLines.clear();

				m_mapTopHLines.clear();
				m_mapBottomHLines.clear();
				m_mapLeftVLines.clear();
				m_mapRightVLines.clear();

				QGraphicsItem *item = this->itemAt(event->scenePos(), QTransform());
				if (item != NULL) {

					if (HShape::DEVICEAREAITEM == item->type()) {
						for (std::map<int, QGraphicsItem*>::iterator it = m_mapDeviceAreas.begin(); it != m_mapDeviceAreas.end(); ++it)
						{
							if (it->second != item) {
								int id = ((DLDeviceAreaItem*)it->second)->id();
								double center_x = ((DLDeviceAreaItem*)it->second)->get_pro().center_.x_;
								double center_y = ((DLDeviceAreaItem*)it->second)->get_pro().center_.y_;
								double width = ((DLDeviceAreaItem*)it->second)->get_pro().width_;
								double height = ((DLDeviceAreaItem*)it->second)->get_pro().height_;
								m_mapTopHLines[id] = (QLineF(QPointF(center_x - 20000, center_y - height / 2), QPointF(center_x + 20000, center_y - height / 2)));
								m_mapBottomHLines[id] = (QLineF(QPointF(center_x - 20000, center_y + height / 2), QPointF(center_x + 20000, center_y + height / 2)));
								m_mapLeftVLines[id] = (QLineF(QPointF(center_x - width / 2, center_y - 20000), QPointF(center_x - width / 2, center_y + 20000)));
								m_mapRightVLines[id] = (QLineF(QPointF(center_x + width / 2, center_y - 20000), QPointF(center_x + width / 2, center_y + 20000)));
							}
						}
					}
					else if (HShape::LANDMARKITEM == item->type()) 
					{
						for (std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedPoints.begin(); it != m_mapAdvancedPoints.end(); ++it)
						{
							if (it->second != item) {
								int id = ((DLLandmarkItem*)it->second)->id();
								double center_x = ((DLLandmarkItem*)it->second)->get_pro().center_.x_;
								double center_y = ((DLLandmarkItem*)it->second)->get_pro().center_.y_;
								double width = ((DLLandmarkItem*)it->second)->get_pro().width_;
								double height = ((DLLandmarkItem*)it->second)->get_pro().height_;
								m_mapLandmarkHLines[id] = (QLineF(QPointF(center_x - 20000, center_y), QPointF(center_x + 20000, center_y)));
								m_mapLandmarkVLines[id] = (QLineF(QPointF(center_x, center_y - 20000), QPointF(center_x, center_y + 20000)));
							}
						}
					}
					else
					{
						if (NULL != m_pSelectItem)
						{
							this->removeItem(m_pSelectItem);
							m_pSelectItem = NULL;
						}
						m_pSelectItem = new QGraphicsRectItem(QRectF(event->scenePos(), QSizeF(0, 0)).normalized());
						m_pSelectItem->setPen(Qt::NoPen);
						m_pSelectItem->setOpacity(0.5);
						m_pSelectItem->setBrush(QColor(133, 177, 222));
						m_pSelectItem->setZValue(50);
						this->addItem(m_pSelectItem);
					}
				}

			}
			else if (DLOperator::TYPE_OVERFITTING_SELECT_LINE == m_pOperator->type())
			{
				DLLandmarkItem *pSelectItem = dynamic_cast<DLLandmarkItem *>(this->itemAt(event->scenePos(), QTransform()));
				if (NULL != pSelectItem)
				{
					pSelectItem->setFlag(QGraphicsItem::ItemIsMovable, false);
					QLineF line(m_ptStartPoint, m_ptEndPoint);
					m_pOverFittingLineItem = this->addLine(line);
					m_pOverFittingLineItem->setPen(QPen(Qt::red, 2));
				}

			}
			else if (DLOperator::TYPE_OVERFITTING_SELECT_POINT == m_pOperator->type())
			{
				QGraphicsItem *item = this->itemAt(event->scenePos(), QTransform());
				if (item == NULL || HShape::LANDMARKITEM != item->type())
				{//节点为空，或者节点不为特殊节点
					if (NULL != m_pSelectItem)
					{
						this->removeItem(m_pSelectItem);
						m_pSelectItem = NULL;
					}
					m_pSelectItem = new QGraphicsRectItem(QRectF(event->scenePos(), QSizeF(0, 0)).normalized());
					m_pSelectItem->setPen(Qt::NoPen);
					m_pSelectItem->setOpacity(0.5);
					m_pSelectItem->setBrush(QColor(133, 177, 222));
					m_pSelectItem->setZValue(50);
					this->addItem(m_pSelectItem);
				}
			}
			else if (m_pOperator->type() == DLOperator::TYPE_COORDINATE_TRANSFORM) 
			{
				m_iButtonType = 1;
			}
		}
		QGraphicsScene::mousePressEvent(event);
	}
	return QGraphicsScene::mousePressEvent(event);
}


void DLCustomScene::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType)
	{
		if (event->button() == Qt::LeftButton)
		{
			if (m_pSelectItem != NULL) 
			{
				QRectF rect = m_pSelectItem->rect();
				QList<QGraphicsItem*> item_list = this->items(rect);

				m_lstSelectIndexs.clear();
				for (int i = 0; i < item_list.size(); ++i)
				{
					if (item_list[i]->type() == HShape::LANDMARKITEM)
					{
						//int id = atoi(((DLLandmarkItem*)item_list[i])->instanceName().c_str());
						int id = ((DLLandmarkItem*)item_list[i])->id();
						((DLLandmarkItem*)item_list[i])->setSelecteState(true);
						m_lstSelectIndexs.push_back(id);
					}
				}
				this->removeItem(m_pSelectItem);
				m_pSelectItem = NULL;
				m_bPressDown = false;
			}
		}

		if (event->button() == Qt::LeftButton  && m_bIsDrawState)
		{//鼠标释放
			QPointF ptMovePos = event->scenePos();

			QGraphicsItem *pItem = this->itemAt(ptMovePos, QTransform());
			DLLandmarkItem * pLandmarkItem = dynamic_cast<DLLandmarkItem *>(pItem);
			if (pLandmarkItem)
			{
				if (IsBorderPoint(pLandmarkItem))
				{
					m_lstPaths.push_back(pLandmarkItem->pos());
				}
			}
			else
			{//处理不是端点
				int index = 0;
				foreach(QPainterPath path, m_lstPainterPaths)
				{
					if (path.contains(ptMovePos) && (m_lstLines.at(index).p1() == m_lstPaths.at(m_lstPaths.size() - 1) || m_lstLines.at(index).p2() == m_lstPaths.at(m_lstPaths.size() - 1)))
					{
						m_lstPaths.push_back(CalcNeedPos(m_lstLines.at(index), ptMovePos));
						break;
					}
					else if (path.contains(ptMovePos) && m_lstPaths.size() == 1 && path.contains(m_pRobotItem->pos()))
					{
						m_lstPaths.push_back(CalcNeedPos(m_lstLines.at(index), ptMovePos));
						break;
					}
					++index;
				}
			}

			DrawCurPath();
		}
	}
	else if (DL_COLLECT_EDIT_TYPE == m_iCustomSceneType || DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{
		if (m_bIsPress)
		{
			m_pOperator->mouseReleaseEvent(event);

			if (event->button() == Qt::RightButton)
			{

				if (m_bIsPress == true && m_pOperator->type() == DLOperator::TYPE_COORDINATE_TRANSFORM)
				{

					m_lineEnd.setP2(event->scenePos());
					m_dCoordinateAngle += m_lineEnd.angleTo(m_lineStart);
					m_iButtonType = Qt::NoButton;
				}
				else
				{
					if (this->itemAt(event->scenePos(), QTransform()) == NULL)
					{
						onHideItemControlPoint();
						SetOperateType(DLOperator::TYPE_SELECT);
					}
				}
			}
			else if (event->button() == Qt::LeftButton)
			{
				if (m_bIsCutoutState)
				{//截图状态
					m_ptEndPoint = event->scenePos();
					//m_ptEndPoint = event->pos();
					m_bIsPress = false;
					m_bIsCutoutState = false;

					//截图
					QGraphicsView *pView = this->views().at(0);
					QPixmap pixmap = pView->viewport()->grab(QRect(pView->mapFromScene(m_ptStartPoint), pView->mapFromScene(m_ptEndPoint)));
					if (NULL == m_pViewGrabDlg)
					{
						m_pViewGrabDlg = new DLViewGrabDlg();
					}
					m_pViewGrabDlg->SetPixmap(pixmap);
					m_pViewGrabDlg->exec();
					//if (m_pViewGrabDlg->exec() == QDialog::Accepted)
					//{
						//if (m_pMutilNormalPointItem)
						//{
	// 						QVector<QPointF> vPoints;
	// 						QRectF rect(m_ptStartPoint, m_ptEndPoint);
	// 						std::map<int, QPointF>::iterator itNormalPos = m_mapNormalPos.begin();
	// 						while (itNormalPos != m_mapNormalPos.end())
	// 						{
	// 							if (!rect.contains(m_pMutilNormalPointItem->mapToScene(itNormalPos->second)))
	// 							{
	// 								m_mapNormalPos.erase(itNormalPos);
	// 								++itNormalPos;
	// 								continue;
	// 							}
	// 							vPoints.push_back(itNormalPos->second);
	// 							++itNormalPos;
	// 						}
	// 						m_pMutilNormalPointItem->setMutilPointItem(vPoints);
	// 						m_pMutilNormalPointItem->update();
						//}
					//}
				}

				if (m_pOperator->type() == DLOperator::TYPE_SELECT)
				{
					QGraphicsItem *item = this->itemAt(event->scenePos(), QTransform());
					if (item != NULL)
					{

						if (HShape::DEVICEAREAITEM == item->type()) {

							HShape::Rectangle pro = ((DLDeviceAreaItem*)item)->get_pro();
							if (m_pHLineItem->isVisible()) {
								if (m_pHLineItem->line().p1().y() - pro.center_.y_ > 0) {
									double offset = pro.center_.y_ + pro.height_ / 2 - m_pHLineItem->line().p1().y();
									pro.center_.y_ -= offset;
									((DLDeviceAreaItem*)item)->slot_pro(pro);
								}
								else {
									double offset = pro.center_.y_ - pro.height_ / 2 - m_pHLineItem->line().p1().y();
									pro.center_.y_ -= offset;
									((DLDeviceAreaItem*)item)->slot_pro(pro);
								}
							}
							if (m_pVLineItem->isVisible()) {
								if (m_pVLineItem->line().p1().x() - pro.center_.x_ > 0) {
									double offset = pro.center_.x_ + pro.width_ / 2 - m_pVLineItem->line().p1().x();
									pro.center_.x_ -= offset;
									((DLDeviceAreaItem*)item)->slot_pro(pro);
								}
								else {
									double offset = pro.center_.x_ - pro.width_ / 2 - m_pVLineItem->line().p1().x();
									pro.center_.x_ -= offset;
									((DLDeviceAreaItem*)item)->slot_pro(pro);
								}
							}

						}
						else if (HShape::LANDMARKITEM == item->type()) {

							HShape::Ellipse pro = ((DLLandmarkItem*)item)->get_pro();
							if (m_pHLineItem->isVisible()) {
								double offset = pro.center_.y_ - m_pHLineItem->line().p1().y();
								pro.center_.y_ -= offset;
								((DLLandmarkItem*)item)->slot_pro(pro);

							}
							if (m_pVLineItem->isVisible()) {
								double offset = pro.center_.x_ - m_pVLineItem->line().p1().x();
								pro.center_.x_ -= offset;
								((DLLandmarkItem*)item)->slot_pro(pro);

							}

						}

						m_pHLineItem->hide();
						m_pVLineItem->hide();
					}

					if (NULL != m_pSelectItem)
					{
						QRectF rect = m_pSelectItem->rect();
						QList<QGraphicsItem*> item_list = this->items(rect);

						for (int i = 0; i < item_list.size(); ++i)
						{
							if (item_list[i]->type() == HShape::LANDMARKITEM)
							{
								item_list[i]->setSelected(true);
							}
						}
						this->removeItem(m_pSelectItem);
						m_pSelectItem = NULL;
					}
				}
				else if (m_pOperator->type() == DLOperator::TYPE_RELOCATION)
				{//如果是重定位模式，点击后回执为选择模式
					SetOperateType(DLOperator::TYPE_SELECT);
					emit ChangeTypeSignal(tr("选择"), DLOperator::TYPE_SELECT);
				}
				else if (DLOperator::TYPE_OVERFITTING_SELECT_LINE == m_pOperator->type())
				{//拟合线
					if (NULL != m_pOverFittingLineItem && m_bIsPress)
					{
						QString strLineName = "";
						if (m_ptStartPoint != m_ptEndPoint)
						{
							m_vOverFittingLineItem.clear();
							DLLandmarkItem *pSelectItem1 = dynamic_cast<DLLandmarkItem *>(this->itemAt(m_ptStartPoint, QTransform()));
							DLLandmarkItem *pSelectItem2 = dynamic_cast<DLLandmarkItem *>(this->itemAt(m_ptEndPoint, QTransform()));
							if (NULL != pSelectItem1 && NULL != pSelectItem2)
							{
								pSelectItem1->setFlag(QGraphicsItem::ItemIsMovable, true);
								pSelectItem2->setFlag(QGraphicsItem::ItemIsMovable, true);

								strLineName += QString::number(pSelectItem1->id());
								strLineName += ",";
								strLineName += QString::number(pSelectItem2->id());
								m_vOverFittingLineItem.push_back(pSelectItem1);
								m_vOverFittingLineItem.push_back(pSelectItem2);
							}
							else
							{
								this->removeItem(m_pOverFittingLineItem);
								m_pOverFittingLineItem = NULL;
							}
						}
						m_ptStartPoint = QPointF(0, 0);
						m_ptEndPoint = QPointF(0, 0);
						SetOperateType(DLOperator::TYPE_SELECT);
						emit ChangeTypeSignal(tr("选择"), DLOperator::TYPE_SELECT);
						if (NULL != m_pOverFittingDlg)
						{
							m_pOverFittingDlg->SetSelectLineID(strLineName);  //设置选中节点的id
							m_pOverFittingDlg->show();
						}
					}

				}

				else if (DLOperator::TYPE_OVERFITTING_SELECT_POINT == m_pOperator->type())
				{
					if (NULL != m_pSelectItem)
					{
						QRectF rect = m_pSelectItem->rect();
						QList<QGraphicsItem*> item_list = this->items(rect);

						for (int i = 0; i < item_list.size(); ++i)
						{
							if (item_list[i]->type() == HShape::LANDMARKITEM)
							{
								item_list[i]->setSelected(true);
							}
						}
						this->removeItem(m_pSelectItem);
						m_pSelectItem = NULL;
					}
				}
				else if (m_pOperator->type() == DLOperator::TYPE_COORDINATE_TRANSFORM) {

					// 				if (m_mapNormalPos.size() != 0) {
					// 					double offset_x = m_pMutilNormalPointItem->scenePos().x() - m_lineStart.p1().x();
					// 					double offset_y = m_pMutilNormalPointItem->scenePos().y() - m_lineStart.p1().y();
					// 					m_ptTransformPoint.setX(offset_x);
					// 					m_ptTransformPoint.setY(offset_y);
					// 				}
					// 
					// 				m_lineEnd.setP2(event->scenePos());
					m_iButtonType = Qt::NoButton;
				}
			}

			m_bIsPress = false;
		}


	}
	return QGraphicsScene::mouseReleaseEvent(event);

}


void DLCustomScene::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType)
	{
		if (m_bPressDown && m_pSelectItem != NULL) {
			QPointF end_pos = event->scenePos();
			m_pSelectItem->setRect(QRectF(m_ptStartPos, end_pos));
		}

		if (m_bIsDrawState)
		{//绘制状态和鼠标左键按下
			QPointF ptMovePos = event->scenePos();
			UpdateLandmarkItemState(ptMovePos);
			DrawCurPath(ptMovePos);
		}
	}
	else if (DL_COLLECT_EDIT_TYPE == m_iCustomSceneType || DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{
		if (m_bIsPress)
		{

			m_ptEndPoint = event->scenePos();
			m_pOperator->mouseMoveEvent(event);
			if (m_pOperator->type() == DLOperator::TYPE_SELECT) {

				QGraphicsItem *item = this->itemAt(event->scenePos(), QTransform());
				if (item != NULL) 
				{
					onCatchItempoistion(item, event->scenePos());
				}

				if (NULL != m_pSelectItem)
				{
					m_pSelectItem->setRect(QRectF(m_ptStartPoint, m_ptEndPoint));
				}
			}
			else if (DLOperator::TYPE_OVERFITTING_SELECT_LINE == m_pOperator->type() && m_bIsPress)
			{//拟合线
				if (NULL != m_pOverFittingLineItem)
				{
					QLineF line(m_ptStartPoint, m_ptEndPoint);
					m_pOverFittingLineItem->setLine(line);
				}
			}
			else if (DLOperator::TYPE_OVERFITTING_SELECT_POINT == m_pOperator->type())
			{
				if (NULL != m_pSelectItem)
				{
					m_pSelectItem->setRect(QRectF(m_ptStartPoint, m_ptEndPoint));
				}
			}
			else if (m_pOperator->type() == DLOperator::TYPE_COORDINATE_TRANSFORM) {
				// 			if (m_bIsPress == true && m_iButtonType == 2) {
				// 				m_lineEnd.setP2(event->scenePos());
				// 
				// 				if (m_mapNormalPos.size() != 0) {
				// 					double degree_angle = m_lineEnd.angleTo(m_lineStart);
				// 					m_pMutilNormalPointItem->setRotation(degree_angle + m_dCoordinateAngle);
				// 				}
				// 			}
							//QGraphicsScene::mouseMoveEvent(event);
			}
		}
	}
	QGraphicsScene::mouseMoveEvent(event);
}


void DLCustomScene::keyPressEvent(QKeyEvent *event)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType)
	{
		if (m_bIsDrawState)
		{
			if (event->key() == Qt::Key_Enter || event->key() == Qt::Key_Return)
			{//ESC键，退出绘制状态
				bool bIsOk = MessageBox();					//消息弹框
				if (bIsOk)
				{//发送数据给core，退出绘制模式
					SendData2Core();					//
					DrawCurPath();
					m_bIsDrawState = false;
					emit ExitDrawStateSignal();		//退出信号
				}
			}
			if (event->key() == Qt::Key_Escape)
			{
				if (NULL != m_pDrawPathItem)
				{
					this->removeItem(m_pDrawPathItem);
					delete m_pDrawPathItem;
					m_pDrawPathItem = NULL;
					m_lstPaths.clear();
				}
				m_lstPaths.push_back(m_pRobotItem->pos());

				//DrawCurPath();
				m_bIsDrawState = false;
				emit ExitDrawStateSignal();		//退出信号
			}
		}
	}
	else if (DL_COLLECT_EDIT_TYPE == m_iCustomSceneType || DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{
		if (event->key() == Qt::Key_Escape) {
			SetOperateType(DLOperator::TYPE_MOVE);
		}
		else if (event->key() == Qt::Key_Delete)
		{
			foreach(QGraphicsItem *item, selectedItems()) {
				int type = item->type();

				if (type == HShape::BEZIERCONTROLPTITEM)
				{
					item = item->parentItem();
					if (NULL == item) continue;
					type = item->type();
				}


				if (type == HShape::LANDMARKITEM) {
					int id = ((DLLandmarkItem*)item)->id();
					m_mapAdvancedPoints.erase(id);
					this->removeItem(item);
					for (auto itor = m_mapAdvancedCurves.begin(); itor != m_mapAdvancedCurves.end();)
					{
						int start_id = ((DLBezierItem*)itor->second)->start_id();
						int end_id = ((DLBezierItem*)itor->second)->end_id();

						if (start_id == id || end_id == id) {
							this->removeItem(itor->second);
							m_mapAdvancedCurves.erase(itor++);
						}
						else {
							itor++;
						}
					}
				}
				else if (type == HShape::BEZIERITEM) {
					int id = ((DLBezierItem*)item)->id();
					m_mapAdvancedCurves.erase(id);
					this->removeItem(item);
				}
				else if (type == HShape::DEVICEAREAITEM) {
					int id = ((DLDeviceAreaItem*)item)->id();
					m_mapDeviceAreas.erase(id);
					this->removeItem(item);
				}
// 				else if (type == HShape::MUTILPOINT) {
// 					m_mapNormalPos.clear();
// 					this->removeItem(m_pMutilNormalPointItem);
// 				}
				else if (type == HShape::SEGMENTITEM) {
					int id = ((DLSegmentItem*)item)->id();
					m_mapNormalLines.erase(id);
					this->removeItem(item);
				}
				else if (type == HShape::ADVANCEDAREA) {
					int id = ((DLAdvancedAreaItem*)item)->id();
					
// 					QRectF rect(item->boundingRect());
// 					QList<QGraphicsItem *> pItems = this->items(rect);
// 					for (int index = 0; index < pItems.size(); ++index)
					std::map<QGraphicsItem *, QVector<QGraphicsItem *>>::iterator it = m_mapAdvanceAreaItem2BesizerItems.find(item);
					if (it != m_mapAdvanceAreaItem2BesizerItems.end())
					{
						QVector<QGraphicsItem *> &vGraphicsItems = it->second;
						QVector<QGraphicsItem *>::iterator itItem = vGraphicsItems.begin();
						while (itItem != vGraphicsItems.end())
						{
							DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(*itItem);
							if (NULL != pBezierItem)
							{
								if (!IsExistOtherAdvanceArea(item, pBezierItem))
								{//判断item是否存在别的高级区域里
									pBezierItem->SetIsAdvanceArea(false);
									pBezierItem->update();
								}
							}
							++itItem;
						}

					}

					m_mapAdvanceAreaItem2BesizerItems.erase(item);   //移除高级区域
					m_mapAdvancedAreas.erase(id);
					this->removeItem(item);
				}
			}

			// 			for (auto it = m_mapSelectNormalPos.begin(); it != m_mapSelectNormalPos.end(); it++) {
			// 				auto temp_it = m_mapNormalPos.find(it->first);
			// 				if (temp_it != m_mapNormalPos.end())
			// 				{
			// 					m_mapNormalPos.erase(temp_it++);
			// 				}
			// 			}
			// 
			// 			QVector<QPointF>  points;
			// 			for (auto it = m_mapNormalPos.begin(); it != m_mapNormalPos.end(); it++)
			// 			{
			// 				points.push_back(it->second);
			// 			}
			// 			if (m_pMutilNormalPointItem != NULL) {
			// 				m_pMutilNormalPointItem->setMutilPointItem(points);
			// 				m_pMutilNormalPointItem->update();
			// 			}

// 			if (m_pMutilNormalPointItem)
// 			{
// 				QVector<QPointF> vPoints = m_pMutilNormalPointItem->getMutilePoint();
// 				//QRectF rect(m_ptStartPoint, m_ptEndPoint);
// 				QVector<QPointF>::iterator itNormalPos = vPoints.begin();
// 				while (itNormalPos != vPoints.end())
// 				{
// 					if (m_rectMutilArea.contains(m_pMutilNormalPointItem->mapToScene(*itNormalPos)))
// 					{
// 						itNormalPos = vPoints.erase(itNormalPos);
// 						continue;
// 					}
// 					//vPoints.push_back(itNormalPos);
// 					++itNormalPos;
// 				}
// 				m_pMutilNormalPointItem->setMutilPointItem(vPoints);
// 				m_pMutilNormalPointItem->update();
// 			}
		}
	}
	QGraphicsScene::keyPressEvent(event);
}


void DLCustomScene::add_deviceArea(QGraphicsItem *item)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType || DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{
		item->setFlag(QGraphicsItem::ItemIsMovable, false);
		//item->setFlag(QGraphicsItem::ItemIsSelectable, false);
		((DLDeviceAreaItem*)item)->set_control_point_visible(false);
	}
	add_no_deviceArea(item);
	update();
}


void DLCustomScene::add_no_deviceArea(QGraphicsItem *item)
{
	if (item == NULL) {
		return;
	}

	item->setZValue(2);
	int id = ((DLDeviceAreaItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = m_mapDeviceAreas.find(id);
	if (it != m_mapDeviceAreas.end()) {
		it = m_mapDeviceAreas.end();
		it--;
		id = ((DLDeviceAreaItem*)it->second)->id();
		m_mapDeviceAreas[id + 1] = item;
		((DLDeviceAreaItem*)item)->set_id(id + 1);
		this->addItem(item);
	}
	else {
		m_mapDeviceAreas[id] = item;
		this->addItem(item);
	}

}


// void DLCustomScene::add_deviceItem(QGraphicsObject *item)
// {
// 
// }


void DLCustomScene::add_normalPoint(QGraphicsItem *item)
{
	add_no_normalPoint(item);
	update();
}


void DLCustomScene::add_no_normalPoint(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}
	this->addItem(item);
}


void DLCustomScene::add_normalLine(QGraphicsItem *item)
{
	add_no_normalLine(item);
	update();
}


void DLCustomScene::add_no_normalLine(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}

	int id = ((DLSegmentItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = m_mapNormalLines.find(id);
	if (it != m_mapNormalLines.end()) {
		it = m_mapNormalLines.end();
		it--;
		id = it->first;
		m_mapNormalLines[id + 1] = item;
		this->addItem(item);
	}
	else
	{
		m_mapNormalLines[id] = item;
		this->addItem(item);
	}

}


void DLCustomScene::add_advancedPoint(QGraphicsItem *item)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType || DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{
		item->setFlag(QGraphicsItem::ItemIsMovable, false);
		item->setFlag(QGraphicsItem::ItemIsSelectable, false);
	}
	add_no_advancedPoint(item);
	update();
}


void DLCustomScene::add_no_advancedPoint(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}

	int id = ((DLLandmarkItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedPoints.find(id);
	if (it != m_mapAdvancedPoints.end()) {
		it = m_mapAdvancedPoints.end();
		it--;
		id = ((DLLandmarkItem*)it->second)->id();
		m_mapAdvancedPoints[id + 1] = item;
		this->addItem(item);
	}
	else
	{
		m_mapAdvancedPoints[id] = item;
		this->addItem(item);
	}

}


void DLCustomScene::add_advancedLine(QGraphicsItem *item)
{
	add_no_advancedLine(item);
	update();
}


void DLCustomScene::add_no_advancedLine(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}

	int id = ((DLSegmentItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedLines.find(id);
	if (it != m_mapAdvancedLines.end()) {
		it = m_mapAdvancedLines.end();
		it--;
		id = ((DLSegmentItem*)it->second)->id();
		m_mapAdvancedLines[id + 1] = item;
		this->addItem(item);
	}
	else {
		m_mapAdvancedLines[id] = item;
		this->addItem(item);
	}

}


void DLCustomScene::add_advancedCurve(QGraphicsItem *item)
{
	if (DL_BROWSE_TYPE == m_iCustomSceneType || DL_COLLECT_MAP_TYPE == m_iCustomSceneType)
	{
		item->setFlag(QGraphicsItem::ItemIsMovable, false);
		item->setFlag(QGraphicsItem::ItemIsSelectable, false);
		((DLBezierItem*)item)->set_child_selectable(false);
	}
	add_no_advancedCurve(item);
	update();
}


void DLCustomScene::add_no_advancedCurve(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}

	item->setZValue(5);
	int id = ((DLBezierItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedCurves.find(id);
	if (it != m_mapAdvancedCurves.end()) {
		it = m_mapAdvancedCurves.end();
		it--;
		id = ((DLBezierItem*)it->second)->id();
		m_mapAdvancedCurves[id + 1] = item;
		this->addItem(item);
	}
	else
	{
		m_mapAdvancedCurves[id] = item;
		this->addItem(item);
	}

}


void DLCustomScene::add_advancedArea(QGraphicsItem *item)
{
	add_no_advancedArea(item);
	update();
}


void DLCustomScene::add_no_advancedArea(QGraphicsItem *item)
{
	if (NULL == item) {
		return;
	}

	int id = ((DLAdvancedAreaItem*)item)->id();
	std::map<int, QGraphicsItem*>::iterator it = m_mapAdvancedAreas.find(id);
	if (it != m_mapAdvancedAreas.end()) {
		it = m_mapAdvancedAreas.end();
		it--;
		id = ((DLAdvancedAreaItem*)it->second)->id();
		m_mapAdvancedAreas[id + 1] = item;
		this->addItem(item);
	}
	else
	{
		m_mapAdvancedAreas[id] = item;
		this->addItem(item);
	}

}


// void DLCustomScene::SetMutilPointItem(std::map<int, QPointF> points_map)
// {
// 	m_mapSelectNormalPos.clear();
// 	m_mapSelectNormalPos = points_map;
// }


QPointF DLCustomScene::CalcTransform(double pos_x, double pos_y)
{
	return QPointF(pos_x, pos_y);
// 	if (m_pMutilNormalPointItem == NULL) {
// 		return QPointF(pos_x, pos_y);
// 	}
	//QPointF point;
// 	double angle = (m_dCoordinateAngle * PI) / 180;
// 		
// 	//绕item中心点旋转后的坐标
// 	QPointF start_point = m_pMutilNormalPointItem->scenePos() - m_ptTransformPoint;
// 
// 	double rotate_x = (pos_x - start_point.x())*cos(angle) - (pos_y - start_point.y())*sin(angle) + start_point.x();
// 	double rotate_y = (pos_x - start_point.x())*sin(angle) + (pos_y - start_point.y())*cos(angle) + start_point.y();
// 
// 	//平移后的坐标
// 	double translate_x = rotate_x + m_ptTransformPoint.x();
// 	double translate_y = rotate_y + m_ptTransformPoint.y();
// 	point.setX(translate_x);
// 	point.setY(translate_y);

// 	QLineF line(m_pMutilNormalPointItem->scenePos(), QPointF(pos_x, pos_y));
// 	line.setAngle(line.angle() + m_dCoordinateAngle);
// 	return line.p2();
}


QPoint DLCustomScene::calculate_window_point(QSize size)
{
	int screen_width = QApplication::desktop()->width();
	int screen_height = QApplication::desktop()->height();
	QPoint point = QCursor::pos();

	if (screen_width - point.x() >= size.width() && screen_height - point.y() >= size.height())
		point = QPoint(point);
	else if (screen_width - point.x() < size.width() && screen_height - point.y() >= size.height())
		point = QPoint(point.x() - size.width(), point.y());
	else if (screen_width - point.x() < size.width() && screen_height - point.y() < size.height())
		point = QPoint(point.x() - size.width(), point.y() - size.height());
	else if (screen_width - point.x() >= size.width() && screen_height - point.y() < size.height())
		point = QPoint(point.x(), point.y() - size.height());

	return point;
}


void DLCustomScene::SetIsShowBgPixmap(bool bIsShowPixmap)
{
// 	if (bIsShowPixmap)
// 	{
// 		m_pBgPixmapItem->show();
// 		m_pMutilNormalPointItem->hide();
// 	}
// 	else
// 	{
// 		m_pBgPixmapItem->hide();
// 		m_pMutilNormalPointItem->show();
// 	}
}

void DLCustomScene::ShowOverfittingDlg()
{
	if (NULL == m_pOverFittingDlg)
	{
		if (this->views().size() > 0)
		{
			m_pOverFittingDlg = new DLOverfittingDlg(this->views().at(0));
			m_pOverFittingDlg->SetScene(this);
		}
	}
	m_vOverFittingLineItem.clear();
	m_vOverFittingPointsItems.clear();
	m_pOverFittingDlg->ClearContent();
	m_pOverFittingDlg->show();
}

void DLCustomScene::CancelOverfitting()
{
	if (NULL != m_pOverFittingLineItem)
	{
		this->removeItem(m_pOverFittingLineItem);
		m_pOverFittingLineItem = NULL;
	}
	m_vOverFittingLineItem.clear();
	m_vOverFittingPointsItems.clear();
	emit ChangeTypeSignal(tr("取消拟合"), DLOperator::TYPE_SELECT);
	SetOperateType(DLOperator::TYPE_SELECT);
	update();
}

void DLCustomScene::StartOverfitting()
{
	if (NULL != m_pOverFittingLineItem)
	{
		this->removeItem(m_pOverFittingLineItem);
		m_pOverFittingLineItem = NULL;
	}

	SetOperateType(DLOperator::TYPE_SELECT);
	emit ChangeTypeSignal(tr("开始拟合"), DLOperator::TYPE_SELECT);

	//开始拟合
	if (m_vOverFittingLineItem.size() == 2)
	{
		QLineF lineStandard(m_vOverFittingLineItem.at(1)->pos(), m_vOverFittingLineItem.at(0)->pos());
		for (int index = 0; index < m_vOverFittingPointsItems.size(); ++index)
		{
			//((DLLandmarkItem*)(APIt->second))->get_pro().center_.x_ * m_rResolution;
			DLLandmarkItem *pGraphicsItem = dynamic_cast<DLLandmarkItem *>(m_vOverFittingPointsItems.at(index));
			if (NULL != pGraphicsItem)
			{
				
				QLineF lineTemp(pGraphicsItem->pos(), m_vOverFittingLineItem.at(0)->pos());
				qreal rAngle = lineTemp.angleTo(lineStandard);
				
				qreal rRotate = lineStandard.angle() + 90;
				lineTemp.setAngle(rRotate);
				QPointF ptIntersert;
				lineTemp.intersect(lineStandard, &ptIntersert);
				//pGraphicsItem->setPos(ptIntersert);
				HShape::Ellipse pro = pGraphicsItem->get_pro();
				pro.center_.x_ = ptIntersert.x();
				pro.center_.y_ = ptIntersert.y();

				pGraphicsItem->set_ellipse_pro(pro);
				//this->addLine(lineTemp);

			}

		}
	}
	update();
}

void DLCustomScene::ClearItemFocus()
{
	QList<QGraphicsItem *> lstSelectItems = this->selectedItems();
	foreach(QGraphicsItem *pItem, lstSelectItems)
	{
		if (NULL != pItem)
		{
			pItem->setSelected(false);
		}
	}

	if (NULL != m_pOverFittingLineItem)
	{
		this->removeItem(m_pOverFittingLineItem);
		m_pOverFittingDlg = NULL;
	}
}

void DLCustomScene::UpdateNormalBackground()
{
//	if (NULL == m_pMutilNormalPointItem) return;
	if (NULL == m_pLoadPixmapItem) return;
	QList<QGraphicsView *> lstViews = views();
	if (lstViews.size() > 0)
	{
		QGraphicsView *pView = lstViews.at(0);
		if (NULL != pView)
		{
			QRect rect = pView->rect();
			QPointF ptStartPoint = pView->mapToScene(rect.topLeft());
			QPointF ptEndPoint = pView->mapToScene(rect.bottomRight());
			//m_pMutilNormalPointItem->UpdateArea(ptStartPoint, ptEndPoint);
			//m_pMutilNormalPointItem->update();
		
			m_pLoadPixmapItem->UpdateArea(ptStartPoint, ptEndPoint);
			m_pLoadPixmapItem->update();
		}
	}
}

void DLCustomScene::ShowMessage(const QString &strMessage)
{
	m_pReaderInfoWidget->show();
	m_pReaderInfoWidget->set_info(MapHandleInfo(0, strMessage));
}

void DLCustomScene::HideMessage()
{
	m_pReaderInfoWidget->hide();
}

qreal DLCustomScene::GetResolution()
{
	return m_rResolution;
}

void DLCustomScene::AddAdvanceAreaItemMap(QGraphicsItem *pAdvanceAreaItem, QGraphicsItem *pItem)
{
	std::map<QGraphicsItem *, QVector<QGraphicsItem *>>::iterator it = m_mapAdvanceAreaItem2BesizerItems.find(pAdvanceAreaItem);
	if (it != m_mapAdvanceAreaItem2BesizerItems.end())
	{
		it->second.push_back(pItem);
	}
	else
	{
		QVector<QGraphicsItem *> vItems;
		vItems.push_back(pItem);
		m_mapAdvanceAreaItem2BesizerItems[pAdvanceAreaItem] = vItems;
	}
}

void DLCustomScene::ReadFinishedSlot(int type, bool isOnlyLoadLm)
{
	switch (type)
	{
	case MapReader::READ_THREAD_TYPE:
	{//加载地图
		_load_json_map(isOnlyLoadLm);
	}break;
	case MapReader::WRITE_THREAD_TYPE:
	{//保存smap文件
		_save_json_map();
		//emit ReadFinishedSignal(m_pReaderThread->isRunning());       //读写结束，检查地图是否需要上传
	}break;
	default:
		break;
	}
}


void  DLCustomScene::_save_json_map()
{
	if (m_pReaderInfoWidget->getState())
	{
		m_pReaderInfoWidget->hide();
	}
}


void  DLCustomScene::_load_json_map(bool isOnlyLoadLm)
{
	
	// 	QTime time;
	// 	time.start();
	// 	QPixmap pixmap;
	// 	pixmap.load("C:/Users/alex_wei/Desktop/map_pic/map_4.png");
	// 	addPixmap(pixmap);
	// 	qDebug() << "加载图片所用时间：" << time.elapsed();


	//创建背景图片
// 	QPixmap pixmap;
// 	pixmap.load(m_strBgImagePath);
// 	HShape::Rectangle stRectPro;
// 	stRectPro.center_.x_ = m_ptBgCenterPoint.x();
// 	stRectPro.center_.y_ = m_ptBgCenterPoint.y();
// 	stRectPro.width_ = pixmap.width();
// 	stRectPro.height_ = pixmap.height();
// 	m_pBgPixmapItem = new DLPixmapItem(stRectPro, m_strBgImagePath);
//  	this->addItem(m_pBgPixmapItem);
// 	m_pBgPixmapItem->setZValue(-1);
//  	m_pBgPixmapItem->setPos(-m_ptBgCenterPoint.x() + stRectPro.width_ / 2, -m_ptBgCenterPoint.y() + stRectPro.height_ / 2);
// 	QPixmapCache::clear();

	this->blockSignals(true);
	
	MapData map_data = m_pReaderThread->GetData();

	std::list<NormalPosition>::iterator NPIt;
	std::list<NormalLine>::iterator NLIt;
	std::list<AdvancedDefine>::iterator ADIt;
	std::list<AdvancedPosition>::iterator APIt;
	std::list<AdvancedLine>::iterator ALIt;
	std::list<AdvancedCurve>::iterator ACIt;
	std::list<AdvancedArea>::iterator AAIt;
	std::list< AreaRect>::iterator ARIt;

	//----------header----------
	m_header.mapName = map_data.header.mapName;
	m_header.mapType = map_data.header.mapType;
	m_header.minPos.pos_x = map_data.header.minPos.pos_x / m_rResolution;
	m_header.minPos.pos_y = -map_data.header.minPos.pos_y / m_rResolution;
	m_header.maxPos.pos_x = map_data.header.maxPos.pos_x / m_rResolution;
	m_header.maxPos.pos_y = -map_data.header.maxPos.pos_y / m_rResolution;
	m_header.resolution = map_data.header.resolution;

	double width = abs(m_header.minPos.pos_x - m_header.maxPos.pos_x);
	double height = abs(m_header.minPos.pos_y - m_header.maxPos.pos_y);
	QRectF map_boundRect_(QPointF(m_header.minPos.pos_x, m_header.minPos.pos_y), QSizeF(width, height));

	//--------DeviceArea-------
	int advancedAreaID = 0;
	for (ARIt = map_data.MapAreaRectList.begin(); ARIt != map_data.MapAreaRectList.end(); ++ARIt)
	{
		std::string className = ARIt->className;
		std::string instanceName = ARIt->instanceName;

		HShape::Rectangle pro;
		pro.center_.x_ = ARIt->centerPos.pos_x / m_rResolution;
		pro.center_.y_ = -ARIt->centerPos.pos_y / m_rResolution;
		pro.width_ = ARIt->width / m_rResolution;
		pro.height_ = ARIt->height / m_rResolution;

		DLDeviceAreaItem *item = NULL;
		if (className.compare("DEVICE") == 0) {
			item = new DLDeviceAreaItem(pro, advancedAreaID, DLDeviceAreaItem::DeviceArea);
			item->set_class_name(className);
			item->set_instance_name(instanceName);
			if (DL_BROWSE_TYPE == m_iCustomSceneType)
			{
				WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
				QString iconPath = QString("%1/%2/%3%4").arg(stMapInfo.rootPath).arg("station").arg(instanceName.c_str()).arg(".png");
				//std::string iconPath = QString("%1/%2/").arg(stMapInfo.rootPath).arg("station") + instanceName + ".png";
				item->loadPixMap(iconPath);
			}
		}
		else if (className.compare("STATION") == 0) {
			item = new DLDeviceAreaItem(pro, advancedAreaID, DLDeviceAreaItem::Station);
			if (DL_BROWSE_TYPE == m_iCustomSceneType)
			{
				item->set_instance_name(instanceName);
				WheelRobotBackgroundConfigStruct stMapInfo = WHEEL_ROBOT_BACKGROUND_CONFIG.getCfg();
				//std::string iconPath = "C:\\Users\\alex_wei\\Desktop\\station\\" + instanceName + ".png";
				QString iconPath = QString("%1/%2/%3%4").arg(stMapInfo.rootPath).arg("station").arg(instanceName.c_str()).arg(".png");
				item->loadPixMap(iconPath);
			}
		}
		this->add_deviceArea(item);
		advancedAreaID++;
	}

	//---------advancedObjectDefine----------
	m_lstAdvancedObjectDefines.clear();
	for (ADIt = map_data.MapAdvancedObjectDefineList.begin(); ADIt != map_data.MapAdvancedObjectDefineList.end(); ADIt++)
	{
		AdvancedDefine advancedDefine;
		advancedDefine.className = ADIt->className;
		advancedDefine.type = ADIt->type;
		m_lstAdvancedObjectDefines.push_back(advancedDefine);
	}


	//------------------cut out area---------
// 	m_ptStartPoint = QPointF(map_data.stCutoutArea.dStartX, map_data.stCutoutArea.dStartY);
// 	m_ptEndPoint = QPointF(map_data.stCutoutArea.dEndX, map_data.stCutoutArea.dEndY);
// 	m_strBgPixmap = map_data.stCutoutArea.strBgPixmap.c_str();
// 
// 	if (DL_BROWSE_TYPE == m_iCustomSceneType)
// 	{
// 		QRectF rect(m_ptStartPoint, m_ptEndPoint);
// 		HShape::Rectangle stRectPro;
// 		stRectPro.center_.x_ = rect.center().rx();
// 		stRectPro.center_.y_ = rect.center().ry();
// 		stRectPro.width_ = rect.width();
// 		stRectPro.height_ = rect.height();
// 		m_pBgPixmapItem = new DLPixmapItem(stRectPro, m_strBgPixmap);
// 		this->addItem(m_pBgPixmapItem);
// 		m_pBgPixmapItem->setPos(rect.center().rx(), rect.center().ry());
// 	}

	//---------normalPoslist---------
//	m_dCoordinateAngle = map_data.stNormalPosArea.dNormalPosAngle;
// 	m_pMutilNormalPointItem = new DLMultiPointItem();
// 	if (m_pMutilNormalPointItem != NULL)
// 	{
// 		//this->add_normalPoint(m_pMutilNormalPointItem);
// 		m_pMutilNormalPointItem->setZValue(2);
// 		m_pMutilNormalPointItem->setPos(map_data.stNormalPosArea.dPosX, map_data.stNormalPosArea.dPosY);	//设置背景的位置
// 		m_pMutilNormalPointItem->setRotation(m_dCoordinateAngle);											//设置背景的角度
// 
// 		if (DL_BROWSE_TYPE == m_iCustomSceneType)
// 		{
// 			m_pMutilNormalPointItem->hide();
// 		}
// 		//connect(this, SIGNAL(sig_sceneRect_changed(const QRectF &)), m_pMutilNormalPointItem, SLOT(slot_pro(const QRectF &)));
// 	}
// 
// 	int normalPointID = 0;
// 	QVector<QPointF> point_list;
// 	m_mapNormalPos.clear();
// 	for (NPIt = map_data.MapNormalPosList.begin(); NPIt != map_data.MapNormalPosList.end(); ++NPIt)
// 	{
// 		QPointF point(100 * NPIt->pos_x, 100 * NPIt->pos_y);
// 		QPointF ptMutilNornalItemPoint = m_pMutilNormalPointItem->mapFromScene(point);
// 		point_list.push_back(ptMutilNornalItemPoint);
// 		m_mapNormalPos[normalPointID] = point;
// 		normalPointID++;
// 	}
// 	m_pMutilNormalPointItem->setMutilPointItem(point_list);


	//---------normalLineList//---------
	int normalLineID = 0;
	m_mapNormalLines.clear();
	for (NLIt = map_data.MapNormalLineList.begin(); NLIt != map_data.MapNormalLineList.end(); ++NLIt)
	{
		HShape::Segment pro;
		pro.start_.x_ = NLIt->startPos.pos_x / m_rResolution;
		pro.start_.y_ = -NLIt->startPos.pos_y / m_rResolution;
		pro.end_.x_ = NLIt->endPos.pos_x / m_rResolution;
		pro.end_.y_ = -NLIt->endPos.pos_y / m_rResolution;

		DLSegmentItem *item = new DLSegmentItem(pro, normalLineID, DLSegmentItem::NormalLine);
		this->add_normalLine(item);
		normalLineID++;
	}


	//------advanvedPoint-------
	for (APIt = map_data.MapAdvancedPointList.begin(); APIt != map_data.MapAdvancedPointList.end(); ++APIt)
	{
		std::string instanceName = APIt->instanceName;
		std::string className = APIt->className;
		std::string temp = instanceName;
		std::string tempStr;
		for (unsigned int i = 0; i < temp.size(); i++)
		{
			if (temp[i] > 0x2F && temp[i] < 0x3A) {
				tempStr.push_back(temp[i]);
			}
		}
		int id = atoi(tempStr.c_str());
		double angle = rad2angle(APIt->pos.pos_dir);
		double x = APIt->pos.pos_x / m_rResolution;
		double y = -APIt->pos.pos_y / m_rResolution;
		DLLandmarkItem *item = new DLLandmarkItem(QPointF(x, y), id, angle, instanceName, className);
		item->setPropertyList(APIt->PropertyList);
		item->setIsFixedList(APIt->ISFixedList);
		this->add_advancedPoint(item);
	}


	//--------advancedLine--------
	int advancedLineID = 0;
	for (ALIt = map_data.MapAdvancedLineList.begin(); ALIt != map_data.MapAdvancedLineList.end(); ++ALIt)
	{
		HShape::Segment pro;
		pro.start_.x_ = ALIt->normalLine.startPos.pos_x / m_rResolution;
		pro.start_.y_ = -ALIt->normalLine.startPos.pos_y / m_rResolution;
		pro.end_.x_ = ALIt->normalLine.endPos.pos_x / m_rResolution;
		pro.end_.y_ = -ALIt->normalLine.endPos.pos_y / m_rResolution;

		if (ALIt->className == "ForbiddenLine")
		{
			DLSegmentItem *item = new DLSegmentItem(pro, advancedLineID, DLSegmentItem::ForbiddenLine);
			this->add_advancedLine(item);
		}
		else if (ALIt->className == "VirtualLine")
		{
			DLSegmentItem *item = new DLSegmentItem(pro, advancedLineID, DLSegmentItem::VirtualLine);
			this->add_advancedLine(item);
		}
		advancedLineID++;
	}



	//--------advancedCurve-------
	for (ACIt = map_data.MapAdvancedCurveList.begin(); ACIt != map_data.MapAdvancedCurveList.end(); ACIt++)
	{
		QPointF controlPos1_1(ACIt->controlPos1.pos_x / m_rResolution, -ACIt->controlPos1.pos_y / m_rResolution);
		QPointF controlPos2_1(ACIt->controlPos2.pos_x / m_rResolution, -ACIt->controlPos2.pos_y / m_rResolution);
		std::string className_1 = ACIt->className;
		std::string instanceName_1 = ACIt->instanceName;

		QPoint bezier_index = cal_bezier_index(*ACIt);
		int start_id = bezier_index.x();
		int end_id = bezier_index.y();

		int id = (start_id << 16) + end_id;

		DLLandmarkItem *start_item = (DLLandmarkItem*)this->find_advancedpoint(start_id);
		DLLandmarkItem *end_item = (DLLandmarkItem*)this->find_advancedpoint(end_id);

		DLBezierItem *item_1 = NULL;
		if (start_item != NULL && end_item != NULL) {
			item_1 = new DLBezierItem(start_item, end_item, controlPos1_1, controlPos2_1, id, className_1, instanceName_1);
			this->add_advancedCurve(item_1);

			if (ACIt->PropertyList.size() != 0) {
				std::string temp_value = ACIt->PropertyList[0].value;
				ACIt->PropertyList[0].value = std::to_string(std::stod(temp_value)  / m_rResolution);

				if (ACIt->PropertyList[6].value == "false") {
					item_1->setPassing(false);
				}
				else {
					item_1->setPassing(true);
				}
				item_1->setPropertyList(ACIt->PropertyList);
			}
		}

		ACIt++;

		if (ACIt != map_data.MapAdvancedCurveList.end())
		{
			QPoint second_bezier_index = cal_bezier_index(*ACIt);
			int second_start_id = second_bezier_index.x();
			int second_end_id = second_bezier_index.y();

			if (second_end_id == start_id && second_start_id == end_id) {
				item_1->set_dir_pro(true);

				std::string temp_value = ACIt->PropertyList[0].value;
				ACIt->PropertyList[0].value = std::to_string(std::stod(temp_value) / m_rResolution);
				item_1->setNagetivePropertyList(ACIt->PropertyList);
			}
			else {
				item_1->set_dir_pro(false);
				ACIt--;
			}
		}
		else {
			item_1->set_dir_pro(false);
			ACIt--;
		}
	}


	//--------advancedArea-------
	for (AAIt = map_data.MapAdvanceAreaList.begin(); AAIt != map_data.MapAdvanceAreaList.end(); ++AAIt)
	{
		std::string className = AAIt->className;
		std::string instanceName = AAIt->instanceName;
		if (instanceName.empty()) {
			instanceName = "1";
		}

		QPointF bottom_left(AAIt->posGroup[0].pos_x / m_rResolution, -AAIt->posGroup[0].pos_y / m_rResolution);
		QPointF top_right(AAIt->posGroup[1].pos_x / m_rResolution, -AAIt->posGroup[1].pos_y / m_rResolution);

		HShape::Rectangle rect_pro;
		rect_pro.center_.x_ = (bottom_left.x() + top_right.x()) / 2;
		rect_pro.center_.y_ = (bottom_left.y() + top_right.y()) / 2;
		rect_pro.width_ = abs(bottom_left.x() - top_right.x());
		rect_pro.height_ = abs(bottom_left.y() - top_right.y());

		DLAdvancedAreaItem *item = new DLAdvancedAreaItem(rect_pro, std::stoi(instanceName), className, instanceName, DLAdvancedAreaItem::Forbidden);
		item->setPropertyList(AAIt->PropertyList);
		this->add_advancedArea(item);


		//设置高级区域的线不可通行
		QRectF rect(bottom_left, top_right);
		QList<QGraphicsItem *> pItems = this->items(rect);
		for (int index = 0; index < pItems.size(); ++index)
		{
			DLBezierItem *pBezierItem = dynamic_cast<DLBezierItem *>(pItems[index]);
			if (NULL != pBezierItem)
			{
				AddAdvanceAreaItemMap(item, pBezierItem);    //添加到高级区域映射中去
				pBezierItem->SetIsAdvanceArea(true);
			}
		}
	}

	InitLandmarkId();
	InitBezierId();
	InitAdvancedAreaId();

	InitAllArea();

	//初始化路径item
	//reader_info_widget_->hide();
	create_pathItem();
	////m_pDeviceViewer->loadDevice();

	m_pReaderInfoWidget->hide();
	this->blockSignals(false);

}


QPoint DLCustomScene::cal_bezier_index(AdvancedCurve bezier)
{
	std::string className = bezier.className;
	std::string instanceName = bezier.instanceName;

	std::string startPosID = bezier.startPos.instanceName;
	std::string endPosID = bezier.endPos.instanceName;
	std::string tempStr_start;
	std::string tempStr_end;

	for (unsigned int i = 0; i < startPosID.size(); i++)
	{
		if (startPosID[i] > 0x2F && startPosID[i] < 0x3A) {
			tempStr_start.push_back(startPosID[i]);
		}
	}
	for (unsigned int i = 0; i < endPosID.size(); i++)
	{
		if (endPosID[i] > 0x2F && endPosID[i] <= 0x3A) {
			tempStr_end.push_back(endPosID[i]);
		}
	}

	int start_id = atoi(tempStr_start.c_str());
	int end_id = atoi(tempStr_end.c_str());

	return QPoint(start_id, end_id);

}


void DLCustomScene::save_json_map(QString file)
{
	MapData map_data;
	/////////////////////////////////////////////////////////header///////////////////////////////////////////////////
	map_data.header.mapType = m_header.mapType;
	map_data.header.mapName = m_header.mapName;
	QPointF __minPos = CalcTransform(m_header.minPos.pos_x, m_header.minPos.pos_y);
	QPointF __maxPos = CalcTransform(m_header.maxPos.pos_x, m_header.maxPos.pos_y);
	map_data.header.minPos.pos_x = __minPos.x() * m_rResolution;
	map_data.header.minPos.pos_y = -__minPos.y() * m_rResolution;
	map_data.header.maxPos.pos_x = __maxPos.x() * m_rResolution;
	map_data.header.maxPos.pos_y = -__maxPos.y() * m_rResolution;

	map_data.header.resolution = m_header.resolution;
	map_data.header.version = m_header.version;

	///////////////////////////////////////cut out area///////////////////////////////////
// 	map_data.stCutoutArea.dStartX = m_ptStartPoint.rx();
// 	map_data.stCutoutArea.dStartY = m_ptStartPoint.ry();
// 	map_data.stCutoutArea.dEndX = m_ptEndPoint.rx();
// 	map_data.stCutoutArea.dEndY = m_ptEndPoint.ry();
// 	map_data.stCutoutArea.strBgPixmap = m_strBgPixmap.toLocal8Bit().data();

	///////////////////////////////////////normal pos area///////////////////////////////////
// 	map_data.stNormalPosArea.dNormalPosAngle = m_pMutilNormalPointItem->rotation();
// 	map_data.stNormalPosArea.dPosX = m_pMutilNormalPointItem->pos().rx();
// 	map_data.stNormalPosArea.dPosY = m_pMutilNormalPointItem->pos().ry();
// 
// 
// 	/////////////////////////////////////////////////////////normalPosList///////////////////////////////////////////////////
// 	QVector<QPointF> vPoints = m_pMutilNormalPointItem->getMutilePoint();
// 	foreach(QPointF ptPoint, vPoints)
// 	{
// 		QPointF pRobotNormalPos = m_pMutilNormalPointItem->mapToScene(ptPoint);
// 		NormalPosition normalPosition;
// 		normalPosition.pos_x = pRobotNormalPos.x() / 100;
// 		normalPosition.pos_y = pRobotNormalPos.y() / 100;
// 		map_data.MapNormalPosList.push_back(normalPosition);
// 	}

	// 	std::map<int, QPointF>::iterator MapIt;
	// 	MapIt = m_mapNormalPos.begin();
	// 	QPointF __normalPos;
	// 	while (MapIt != m_mapNormalPos.end())
	// 	{
	// 		NormalPosition normalPosition;
	// 		__normalPos = CalcTransform(MapIt->second.x(), MapIt->second.y());
	// 		normalPosition.pos_x = __normalPos.x() / 100;
	// 		normalPosition.pos_y = __normalPos.y() / 100;
	// 		map_data.MapNormalPosList.push_back(normalPosition);
	// 		MapIt++;
	// 	}


		/////////////////////////////////////////////////////////normalLineList///////////////////////////////////////////////////
	std::map<int, QGraphicsItem*>::iterator NlIt = m_mapNormalLines.begin();
	while (NlIt != m_mapNormalLines.end())
	{
		NormalLine normalLine;
		normalLine.startPos.pos_x = ((DLSegmentItem*)(NlIt->second))->get_pro().start_.x_ * m_rResolution;
		normalLine.startPos.pos_y = -((DLSegmentItem*)(NlIt->second))->get_pro().start_.y_  * m_rResolution;
		normalLine.endPos.pos_x = ((DLSegmentItem*)(NlIt->second))->get_pro().end_.x_ * m_rResolution;
		normalLine.endPos.pos_y = -((DLSegmentItem*)(NlIt->second))->get_pro().end_.y_ * m_rResolution;
		map_data.MapNormalLineList.push_back(normalLine);
		NlIt++;
	}


	/////////////////////////////////////////////////////////advancedObjectDefineList///////////////////////////////////////////////////
	std::list<AdvancedDefine>::iterator AODIt = m_lstAdvancedObjectDefines.begin();
	while (AODIt != m_lstAdvancedObjectDefines.end())
	{
		AdvancedDefine advancedDefine;
		advancedDefine.className = AODIt->className;
		advancedDefine.type = AODIt->type;
		map_data.MapAdvancedObjectDefineList.push_back(advancedDefine);
		AODIt++;
	}


	/////////////////////////////////////////////////////////advancedPointLis///////////////////////////////////////////////////
	std::map<int, QGraphicsItem*>::iterator APIt = m_mapAdvancedPoints.begin();
	std::vector<Property>::iterator AP_ProIt;
	std::vector<Add_IsFixed>::iterator AP_IsFixedIt;
	while (APIt != m_mapAdvancedPoints.end())
	{
		AdvancedPosition advancedPosition;
		double angle = ((DLLandmarkItem*)(APIt->second))->get_pro().angle_;

		advancedPosition.className = ((DLLandmarkItem*)(APIt->second))->className();
		advancedPosition.instanceName = ((DLLandmarkItem*)(APIt->second))->instanceName();
		advancedPosition.pos.pos_x = ((DLLandmarkItem*)(APIt->second))->get_pro().center_.x_ * m_rResolution;
		advancedPosition.pos.pos_y = -((DLLandmarkItem*)(APIt->second))->get_pro().center_.y_ * m_rResolution;
		advancedPosition.pos.pos_dir = angle2rad(((DLLandmarkItem*)(APIt->second))->get_pro().angle_);

		std::vector<Property> propertyVec = ((DLLandmarkItem*)(APIt->second))->getPropertyList();
		for (AP_ProIt = propertyVec.begin(); AP_ProIt != propertyVec.end(); AP_ProIt++)
		{
			Property property;
			property.key = AP_ProIt->key;
			property.type = AP_ProIt->type;
			property.value = AP_ProIt->value;
			advancedPosition.PropertyList.push_back(property);
		}
		std::vector<Add_IsFixed> nFixedVec = ((DLLandmarkItem*)(APIt->second))->getIsFixedList();
		for (AP_IsFixedIt = nFixedVec.begin(); AP_IsFixedIt != nFixedVec.end(); AP_IsFixedIt++)
		{
			Add_IsFixed isFixedTmp;
			isFixedTmp.key = AP_IsFixedIt->key;
			isFixedTmp.type = AP_IsFixedIt->type;
			isFixedTmp.value = AP_IsFixedIt->value;
			advancedPosition.ISFixedList.push_back(isFixedTmp);
		}
		map_data.MapAdvancedPointList.push_back(advancedPosition);
		APIt++;
	}

	/////////////////////////////////////////////////////////advancedLineList///////////////////////////////////////////////////
	std::map<int, QGraphicsItem*>::iterator ALIt = m_mapAdvancedLines.begin();
	while (ALIt != m_mapAdvancedLines.end())
	{
		AdvancedLine advancedLine;
		advancedLine.className = ((DLSegmentItem*)(ALIt->second))->className();
		advancedLine.instanceName = ((DLSegmentItem*)(ALIt->second))->instanceName();
		advancedLine.normalLine.startPos.pos_x = ((DLSegmentItem*)(ALIt->second))->get_pro().start_.x_ * m_rResolution;
		advancedLine.normalLine.startPos.pos_y = -((DLSegmentItem*)(ALIt->second))->get_pro().start_.x_ * m_rResolution;
		advancedLine.normalLine.endPos.pos_x = ((DLSegmentItem*)(ALIt->second))->get_pro().end_.x_ * m_rResolution;
		advancedLine.normalLine.endPos.pos_y = -((DLSegmentItem*)(ALIt->second))->get_pro().end_.y_ * m_rResolution;
		map_data.MapAdvancedLineList.push_back(advancedLine);
		ALIt++;
	}

	/////////////////////////////////////////////////////////advancedAreaList///////////////////////////////////////////////////
	std::map<int, QGraphicsItem*>::iterator AAIt = m_mapAdvancedAreas.begin();
	while (AAIt != m_mapAdvancedAreas.end())
	{
		std::vector<Property>::iterator AA_ProIt = ((DLAdvancedAreaItem*)(AAIt->second))->getPropertyList().begin();
		AdvancedArea advancedArea;
		advancedArea.className = ((DLAdvancedAreaItem*)(AAIt->second))->className();
		advancedArea.instanceName = ((DLAdvancedAreaItem*)(AAIt->second))->instanceName();

		HShape::Rectangle pro = ((DLAdvancedAreaItem*)(AAIt->second))->get_pro();

		NormalPosition normalPosition_1, normalPosition_2;
		normalPosition_1.pos_x = (pro.center_.x_ - pro.width_ / 2) * m_rResolution;
		normalPosition_1.pos_y = -(pro.center_.y_ - pro.height_ / 2) * m_rResolution;
		normalPosition_2.pos_x = (pro.center_.x_ + pro.width_ / 2) * m_rResolution;
		normalPosition_2.pos_y = -(pro.center_.y_ + pro.height_ / 2) * m_rResolution;
		advancedArea.posGroup.push_back(normalPosition_1);
		advancedArea.posGroup.push_back(normalPosition_2);


		while (AA_ProIt != ((DLAdvancedAreaItem*)(AAIt->second))->getPropertyList().end())
		{
			Property property;
			property.key = AA_ProIt->key;
			property.type = AA_ProIt->type;
			property.value = AA_ProIt->value;
			advancedArea.PropertyList.push_back(property);
			AA_ProIt++;
		}
		map_data.MapAdvanceAreaList.push_back(advancedArea);
		AAIt++;
	}


	//////////////////////////////////////////////////////////DeviceAreaList ///////////////////////////////////////////////////
	std::map<int, QGraphicsItem*>::iterator DAIt = m_mapDeviceAreas.begin();
	while (DAIt != m_mapDeviceAreas.end())
	{
		AreaRect areaRect;
		areaRect.className = ((DLDeviceAreaItem*)(DAIt->second))->className();
		areaRect.instanceName = ((DLDeviceAreaItem*)(DAIt->second))->instanceName();
		areaRect.centerPos.pos_x = ((DLDeviceAreaItem*)(DAIt->second))->get_pro().center_.x_ * m_rResolution;
		areaRect.centerPos.pos_y = -((DLDeviceAreaItem*)(DAIt->second))->get_pro().center_.y_ * m_rResolution;
		areaRect.width = ((DLDeviceAreaItem*)(DAIt->second))->get_pro().width_ * m_rResolution;
		areaRect.height = ((DLDeviceAreaItem*)(DAIt->second))->get_pro().height_  * m_rResolution;
		map_data.MapAreaRectList.push_back(areaRect);
		DAIt++;
	}

	/////////////////////////////////////////////////////////advancedCurveList///////////////////////////////////////////////////
	std::map<int, QGraphicsItem*>::iterator ACIt = m_mapAdvancedCurves.begin();
	std::vector<Property>::iterator AC_ProIt;
	while (ACIt != m_mapAdvancedCurves.end())
	{
		DLLandmarkItem *start_item = (DLLandmarkItem*)((DLBezierItem*)(ACIt->second))->start_item();
		DLLandmarkItem *end_item = (DLLandmarkItem*)((DLBezierItem*)(ACIt->second))->end_item();
		bool isDoubleDir = ((DLBezierItem*)(ACIt->second))->dir_pro();

		AdvancedCurve advancedCurve;
		advancedCurve.className = ((DLBezierItem*)(ACIt->second))->className();
		advancedCurve.instanceName = ((DLBezierItem*)(ACIt->second))->instanceName();
		advancedCurve.startPos.className = start_item->className();
		advancedCurve.startPos.instanceName = start_item->instanceName();
		advancedCurve.startPos.pos_x = start_item->get_pro().center_.x_ * m_rResolution;
		advancedCurve.startPos.pos_y = -start_item->get_pro().center_.y_ * m_rResolution;
		advancedCurve.endPos.className = end_item->className();
		advancedCurve.endPos.instanceName = end_item->instanceName();
		advancedCurve.endPos.pos_x = end_item->get_pro().center_.x_ * m_rResolution;
		advancedCurve.endPos.pos_y = -end_item->get_pro().center_.y_ * m_rResolution;
		advancedCurve.controlPos1.pos_x = ((DLBezierItem*)(ACIt->second))->get_pro().c1_.x_ * m_rResolution;
		advancedCurve.controlPos1.pos_y = -((DLBezierItem*)(ACIt->second))->get_pro().c1_.y_ * m_rResolution;
		advancedCurve.controlPos2.pos_x = ((DLBezierItem*)(ACIt->second))->get_pro().c2_.x_ * m_rResolution;
		advancedCurve.controlPos2.pos_y = -((DLBezierItem*)(ACIt->second))->get_pro().c2_.y_ * m_rResolution;

		AdvancedCurve advancedCurve1;
		advancedCurve1.className = ((DLBezierItem*)(ACIt->second))->className();
		advancedCurve1.instanceName = ((DLBezierItem*)(ACIt->second))->instanceName();
		advancedCurve1.startPos.className = end_item->className();
		advancedCurve1.startPos.instanceName = end_item->instanceName();
		advancedCurve1.startPos.pos_x = end_item->get_pro().center_.x_ * m_rResolution;
		advancedCurve1.startPos.pos_y = -end_item->get_pro().center_.y_ * m_rResolution;
		advancedCurve1.endPos.className = start_item->className();
		advancedCurve1.endPos.instanceName = start_item->instanceName();
		advancedCurve1.endPos.pos_x = start_item->get_pro().center_.x_ * m_rResolution;
		advancedCurve1.endPos.pos_y = -start_item->get_pro().center_.y_ * m_rResolution;
		advancedCurve1.controlPos1.pos_x = ((DLBezierItem*)(ACIt->second))->get_pro().c2_.x_ * m_rResolution;
		advancedCurve1.controlPos1.pos_y = -((DLBezierItem*)(ACIt->second))->get_pro().c2_.y_ * m_rResolution;
		advancedCurve1.controlPos2.pos_x = ((DLBezierItem*)(ACIt->second))->get_pro().c1_.x_ * m_rResolution;
		advancedCurve1.controlPos2.pos_y = -((DLBezierItem*)(ACIt->second))->get_pro().c1_.y_ * m_rResolution;

		for (AC_ProIt = ((DLBezierItem*)(ACIt->second))->getPropertyList().begin(); AC_ProIt != ((DLBezierItem*)(ACIt->second))->getPropertyList().end(); ++AC_ProIt)
		{
			Property property;
			property.key = AC_ProIt->key;
			property.type = AC_ProIt->type;
			if (property.key == "weight") {
				property.value = std::to_string(std::stod(AC_ProIt->value) * m_rResolution);
			}
			else {
				property.value = AC_ProIt->value;
			}
			advancedCurve.PropertyList.push_back(property);
		}

		for (AC_ProIt = ((DLBezierItem*)(ACIt->second))->getNagetivePropertyList().begin(); AC_ProIt != ((DLBezierItem*)(ACIt->second))->getNagetivePropertyList().end(); ++AC_ProIt)
		{
			Property property;
			property.key = AC_ProIt->key;
			property.type = AC_ProIt->type;
			if (property.key == "weight") {
				property.value = std::to_string(std::stod(AC_ProIt->value) * m_rResolution);
			}
			else {
				property.value = AC_ProIt->value;
			}
			advancedCurve1.PropertyList.push_back(property);
		}

		if (isDoubleDir) {
			map_data.MapAdvancedCurveList.push_back(advancedCurve);
			map_data.MapAdvancedCurveList.push_back(advancedCurve1);
		}
		else {
			map_data.MapAdvancedCurveList.push_back(advancedCurve);
		}
		ACIt++;
	}

	int err_code;
	QString err_desc = "";
	bool ret = check_smap(err_code, err_desc);
	if (ret)
	{
		m_pReaderInfoWidget->set_info(MapHandleInfo(err_code, err_desc, "地图保存信息如下"));
		m_pReaderInfoWidget->show();
		std::string filename_new = file.toLocal8Bit();
		m_pReaderThread->SetMapData(filename_new, map_data);
		m_pReaderThread->start();
	}
	else 
	{
		m_pReaderInfoWidget->set_info(MapHandleInfo(err_code, err_desc, "地图保存信息如下"));
		m_pReaderInfoWidget->show();
	}

}


void DLCustomScene::load_json_map(QString strDirPath, bool isOnlyLoadLm)
{
	removeall();			//清空画面
	if (NULL != m_pPathItem)
	{
		this->removeItem(m_pPathItem);
		m_pPathItem = NULL;
	}


	QPixmapCache::clear();

	QDir dir(strDirPath);
	//配置文件
	QStringList nameFilters;
	nameFilters << "*.cfg";

	QString strRootPath;
	QStringList files = dir.entryList(nameFilters, QDir::Files | QDir::Readable, QDir::Name);
	if (files.size() > 0)
	{//默认取第一个
		QString strCfgFile = QString("%1/%2").arg(strDirPath).arg(files.at(0));
	//	strRootPath = strCfgFile;
	//	QFile fileTemp(strCfgFile);

	//	if (!fileTemp.open(QIODevice::ReadOnly | QIODevice::Text)) return;
	//	if (!fileTemp.open(QIODevice::ReadOnly | QIODevice::Text)) return;

		QSettings settings(strCfgFile, QSettings::IniFormat);
		m_rResolution = settings.value("ImageResolution").toDouble();
		m_ptBgCenterPoint.setX(settings.value("CentreX").toDouble());
		m_ptBgCenterPoint.setY(settings.value("CentreY").toDouble());
		m_rBgWidth = settings.value("Width").toDouble();
		m_rBgHeight = settings.value("Height").toDouble();
		//m_strLowImagePath = settings.value("LowImage").toByteArray();
		m_strMediumImagePath = settings.value("MediumImage").toByteArray();
		//m_strHighImagePath = settings.value("HighImage").toByteArray();
	}

	QList<QGraphicsView *> lstViews = this->views();
	if (lstViews.size() > 0)
	{
		DLCollectMapView *pView = dynamic_cast<DLCollectMapView *>(lstViews.at(0));
		if (NULL != pView)
		{
			pView->SetResolution(m_rResolution);
		}
	}

	//smap文件
	nameFilters.clear();
	nameFilters << "*.mapinfo";
	files = dir.entryList(nameFilters, QDir::Files | QDir::Readable, QDir::Name);
	if (files.size() > 0)
	{//默认取第一个
		m_pReaderInfoWidget->show();
		m_pReaderInfoWidget->set_info(MapHandleInfo(0, "正在打开smap地图，请耐心等待。"));
		QString strSmapTemp = QString("%1/%2").arg(strDirPath).arg(files.at(0));
		std::string strSmapFileName = strSmapTemp.toLocal8Bit().data();
		m_pReaderThread->SetReadPro(strSmapFileName, MapReader::READ_THREAD_TYPE, isOnlyLoadLm);
		m_pReaderThread->start();
	}
	else
	{
		m_pReaderInfoWidget->show();
		m_pReaderInfoWidget->set_info(MapHandleInfo(0, "正在打开smap地图，请耐心等待。"));
		m_pReaderThread->ClearMapData();
		_load_json_map(isOnlyLoadLm);
	}



// 	if (NULL != m_pReadBgMapThread)
// 	{
// 		QString strFilePath = QString("%1/%2").arg(strDirPath).arg("测试数据.txt");
// 		m_pReadBgMapThread->SetFilePath(strFilePath);
// 		m_pReadBgMapThread->Start();
// 		m_time.start();
// 	}

	if (NULL == m_pLoadPixmapItem)
	{
		m_pLoadPixmapItem = new LoadPixmapItem;
		this->addItem(m_pLoadPixmapItem);
		QRectF rectBg(QPointF(-m_ptBgCenterPoint.x(), -m_ptBgCenterPoint.y()), QPointF(-m_ptBgCenterPoint.x() + m_rBgWidth, -m_ptBgCenterPoint.y() + m_rBgHeight));
		m_pLoadPixmapItem->SetStartEndPoint(rectBg.topLeft(), rectBg.bottomRight());
 		
// 		QPixmap highLevelPixmap;
//  		QString strFilePath = QString("%1/%2").arg(strDirPath).arg(m_strHighImagePath);
//   		highLevelPixmap.load(strFilePath);		//加载高清图
// 		m_pLoadPixmapItem->SetHighDefinitionPixmap(highLevelPixmap);
// 		highLevelPixmap.load("");

		QPixmap mediumLevelPixmap;
		QString strMediumPath = QString("%1/%2").arg(strDirPath).arg(m_strMediumImagePath);
		mediumLevelPixmap.load(strMediumPath);		//加载原图
  		m_pLoadPixmapItem->SetMediumPixmap(mediumLevelPixmap);
		mediumLevelPixmap.load("");

// 		QString strLowPath = QString("%1/%2").arg(strDirPath).arg(m_strLowImagePath);
// 		QPixmap lowLevelPixmap;
// 		lowLevelPixmap.load(strLowPath);
// 		m_pLoadPixmapItem->SetLowPixmap(lowLevelPixmap);
// 		lowLevelPixmap.load("");

		m_pLoadPixmapItem->UpdateArea(rectBg.topLeft(), rectBg.bottomRight());

	}
	QPixmapCache::clear();
}


void DLCustomScene::connect_backstage_map_path()
{

	InitBezierId();
	m_iBezierId = -1;
	for (auto itor = m_mapAdvancedCurves.begin(); itor != m_mapAdvancedCurves.end(); itor++)
	{
		this->removeItem(itor->second);
	}
	m_mapAdvancedCurves.clear();

	//--------advancedCurve-------
	for (int i = 0; i < m_vBackstagemapPaths.size(); i++)
	{
		int start_id = m_vBackstagemapPaths[i].x();
		int end_id = m_vBackstagemapPaths[i].y();
		int id = (start_id << 16) + end_id;

		DLLandmarkItem *start_item = (DLLandmarkItem*)this->find_advancedpoint(start_id);
		DLLandmarkItem *end_item = (DLLandmarkItem*)this->find_advancedpoint(end_id);

		if (start_item != NULL && end_item != NULL)
		{
			DLBezierItem *item = new DLBezierItem(start_item, end_item, id);
			this->add_advancedCurve(item);
		}
	}
}



void DLCustomScene::SetCutOutState(bool bIsCutoutState)
{
	m_bIsCutoutState = bIsCutoutState;
}

void DLCustomScene::SetBgPixmap(const QString &strBgPixmap)
{
	//m_strBgPixmap = strBgPixmap;
}

bool DLCustomScene::getBezier_curvature(const QPainterPath &bezier_path, int pointCount)
{
	QPointF point_start = bezier_path.pointAtPercent(0);
	for (int i = 1; i <= pointCount - 1; i++)
	{
		double step_pre = i / (double)pointCount;
		double step_next = (i + 1) / (double)pointCount;

		QPointF point_pre = bezier_path.pointAtPercent(step_pre);
		QPointF point_next = bezier_path.pointAtPercent(step_next);

		double line1 = QLineF(point_start, point_pre).length();
		double line2 = QLineF(point_start, point_next).length();
		double line3 = QLineF(point_pre, point_next).length();
		double value = (line1 * line1 + line3 * line3 - line2 * line2) / (2 * line1 * line3);

		value = value > 1 ? 1 : value;
		value = value < -1 ? -1 : value;

		double angel = acos(value) * 180.0 / M_PI;
		if (angel < 90) {
			return false;
		}
	}
	return true;
}


void DLCustomScene::SetWorkPath(std::vector<QPoint> work_paths, std::vector<int> patrol_points, bool state)
{
	if (m_pPathItem != NULL) {
		m_pPathItem->SetWorkPath(work_paths, patrol_points, state);
	}
}

void DLCustomScene::create_platformItem()
{
	if (m_pPlatformItem == NULL)
	{
		m_pPlatformItem = new DLPlatformItem;
		this->addItem(m_pPlatformItem);
		m_pPlatformItem->setPos(GetCenter());
		m_pPlatformItem->setZValue(46);
	}
}

double DLCustomScene::cal_car_angle(QPointF startPos, QPointF c1Pos, QPointF c2Pos, QPointF endPos, double percentage)
{
	double p0_x = startPos.x();
	double p0_y = startPos.y();
	double p1_x = c1Pos.x();
	double p1_y = c1Pos.y();
	double p2_x = c2Pos.x();
	double p2_y = c2Pos.y();
	double p3_x = endPos.x();
	double p3_y = endPos.y();

	double delta_x = (3 * p1_x - p0_x - 3 * p2_x + p3_x)*percentage*percentage + (2 * p0_x - 4 * p1_x + 2 * p2_x)*percentage + p1_x - p0_x;
	double delta_y = (3 * p1_y - p0_y - 3 * p2_y + p3_y)*percentage*percentage + (2 * p0_y - 4 * p1_y + 2 * p2_y)*percentage + p1_y - p0_y;

	return atan2f(delta_y, delta_x) * 180 / 3.14;
}

void DLCustomScene::SetPlatformItemPropertry(double pos_x, double pos_y, double angle)
{
	if (m_pPlatformItem != NULL)
	{
		m_pPlatformItem->setPos(pos_x, pos_y);
		m_pPlatformItem->setRotation(-angle);
	}
}

bool DLCustomScene::check_smap(int &error_code, QString &err_desc)
{
	bool ret = true;
	int warn_code = 0;
	error_code = 0;

	//检测充电点，充电辅助点的旋转
	QString charge_point_desc("");
	std::map<std::string, std::string> record_landmarks;
	std::map<int, QGraphicsItem*>::iterator APIt = m_mapAdvancedPoints.begin();
	std::vector<Property>::iterator AP_ProIt;
	while (APIt != m_mapAdvancedPoints.end())
	{
		std::string instance_name = ((DLLandmarkItem*)(APIt->second))->instanceName();
		std::string class_name = ((DLLandmarkItem*)(APIt->second))->className();
		std::string allowspin;

		std::vector<Property> propertyVec = ((DLLandmarkItem*)(APIt->second))->getPropertyList();
		for (AP_ProIt = propertyVec.begin(); AP_ProIt != propertyVec.end(); AP_ProIt++)
		{
			if (AP_ProIt->key == "allowspin") {
				allowspin = AP_ProIt->value;
			}
		}
		if (class_name == CHARGE_LADMARK && allowspin == "true") {
			ret = false;
			error_code = 2;
			charge_point_desc += "充电点属性设置错误，不可以旋转\n";
		}
		else if (class_name == CHARGEAUX_LANDMARK && allowspin == "false") {
			ret = false;
			error_code = 2;
			charge_point_desc += "充电辅助点属性设置错误，可以旋转\n";
		}
		record_landmarks[instance_name] = instance_name;
		APIt++;
	}


	//检测速度是否有NULL和贝塞尔曲线的曲率
	QString bezier_desc("");
	std::map<std::string, std::string> record_bezier_landmarks;
	std::map<int, QGraphicsItem*>::iterator ACIt = m_mapAdvancedCurves.begin();
	std::vector<Property>::iterator AC_ProIt;

	while (ACIt != m_mapAdvancedCurves.end())
	{
		DLLandmarkItem *start_item = (DLLandmarkItem*)((DLBezierItem*)(ACIt->second))->start_item();
		DLLandmarkItem *end_item = (DLLandmarkItem*)((DLBezierItem*)(ACIt->second))->end_item();

		std::string start_instance_name = start_item->instanceName();
		std::string end_instance_name = end_item->instanceName();
		record_bezier_landmarks[start_instance_name] = start_instance_name;
		record_bezier_landmarks[end_instance_name] = end_instance_name;

// 		QString temp_head = QString::fromStdString(start_instance_name) + "和" + QString::fromStdString(end_instance_name) + "之间的路径,";
// 		HShape::Bezier pro = ((DLBezierItem*)(ACIt->second))->get_pro();
// 		QPainterPath bezier_path;
// 		bezier_path.moveTo(pro.start_.x_ / 100, pro.start_.y_ / 100);
// 		bezier_path.cubicTo(QPointF(pro.c1_.x_ / 100, pro.c1_.y_ / 100), QPointF(pro.c2_.x_ / 100, pro.c2_.y_ / 100), QPointF(pro.end_.x_ / 100, pro.end_.y_ / 100));
// 
// 		double bezier_length = bezier_path.length();
// 		int point_count = bezier_length > 4 ? 301 : 501;
// 		bool is_curvature_ok = getBezier_curvature(bezier_path, point_count);
// 		if (!is_curvature_ok) {
// 			ret = false;
// 			error_code = 2;
// 		}
// 
// 		QString bezier_speed_desc("");
// 		bool is_empty = true;
// 		for (AC_ProIt = ((DLBezierItem*)(ACIt->second))->getPropertyList().begin(); AC_ProIt != ((DLBezierItem*)(ACIt->second))->getPropertyList().end(); ++AC_ProIt)
// 		{
// 			std::string pro_key = AC_ProIt->key;
// 
// 			if (AC_ProIt->value == "NULL") {
// 				if (AC_ProIt->key == "maxspeed" || AC_ProIt->key == "maxrot" || AC_ProIt->key == "blockdist") {
// 					ret = false;
// 					error_code = 2;
// 					bezier_speed_desc = bezier_speed_desc + pro_key.c_str() + ",";
// 				}
// 				else {
// 					warn_code = 1;
// 					bezier_speed_desc = bezier_speed_desc + pro_key.c_str() + ",";
// 				}
// 				is_empty = false;
// 			}
// 		}
// 
// 		if (!is_curvature_ok && !is_empty) {
// 			bezier_desc = bezier_desc + temp_head + "曲率异常," + bezier_speed_desc + "设置错误\n";
// 		}
// 		else if (!is_curvature_ok && is_empty) {
// 			bezier_desc = bezier_desc + temp_head + "曲率异常\n";
// 		}
// 		else if (is_curvature_ok && !is_empty) {
// 			bezier_desc = temp_head + bezier_speed_desc + "设置错误\n";
// 		}
		ACIt++;
	}


	// 检测孤立点
	QString advanced_point_desc("");
	record_landmarks.size();
	record_bezier_landmarks.size();
	std::string isolated_points;
	bool is_empty = true;
	for (auto it = record_landmarks.begin(); it != record_landmarks.end(); ++it)
	{
		std::string landermark_key = it->first;

		//若记录点的列表在贝塞尔点列表未找到，则为孤立点
		if (record_bezier_landmarks.find(landermark_key) == record_bezier_landmarks.end()) {
			isolated_points = isolated_points + landermark_key + ",";
			is_empty = false;
			warn_code = 1;
		}
	}
	if (!is_empty) {
		advanced_point_desc = advanced_point_desc + QString::fromLocal8Bit(isolated_points.c_str()) + "以上点为孤立点\n";
	}


	QString err_desc_head;
	//没出现错误，出现警告，error_code为警告
	if (error_code == 0 && warn_code == 0)
	{
		error_code = warn_code;
		err_desc_head = "保存地图成功。";
	}
	else if (error_code == 0 && warn_code != 0) {
		error_code = warn_code;
		err_desc_head = "保存地图成功，告警如下：\n";
	}
	else {
		err_desc_head = "保存地图失败，错误如下：\n";
	}
	err_desc = err_desc_head + charge_point_desc + bezier_desc + advanced_point_desc;
	return ret;
}





