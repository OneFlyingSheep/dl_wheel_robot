#include <QtWidgets>
#include "DLMultiPointItem.h"
#include <QDebug>

#define MAX_POINT_SIZE (12000)

#define CELL_WIDTH (100.0)
#define CELL_HEIGHT (100.0)
#define CELL_COUNT (100.0)

DLMultiPointItem::DLMultiPointItem(QGraphicsItem *parent)
	: DLShapeItem(parent)
	,m_ptStartPoint(0.0, 0.0)
	, m_ptEndPoint(0.0, 0.0)
	, m_ptCurStartPoint(0.0, 0.0)
	, m_ptCurEndPoint(0.0, 0.0)
	,m_iRowStartIndex(0)
	, m_iRowEndIndex(0)
	,m_iColumnStartIndex(0)
	, m_iColumnEndIndex(0)
{
	
	m_bIsShowShape = false;
	setZValue(0);
	//setTransformAble(false);
}


DLMultiPointItem::~DLMultiPointItem()
{

}

void DLMultiPointItem::setTransformAble(bool state)
{
// 	if (state) {
// 		m_bIsShowShape = state;
// 		setFlag(ItemIsSelectable, true);
// 		setFlag(ItemIsMovable, true);
// 		setAcceptedMouseButtons(Qt::LeftButton);
// 		setCursor(Qt::ClosedHandCursor);
// 	}
// 	else {
// 		setFlag(ItemIsSelectable, false);
// 		setFlag(ItemIsMovable, false);
// 		setAcceptedMouseButtons(Qt::NoButton);
// 		setCursor(Qt::ArrowCursor);
// 		m_bIsShowShape = state;
// 	}
// 	
}


// void DLMultiPointItem::SetIsShowBackgroundImage(bool bIsShowBg)
// {
// 	m_bIsShowPixmap = bIsShowBg;
// }

QRectF DLMultiPointItem::boundingRect() const
{
	//return QPolygonF(m_vPoints).boundingRect();
	return QRectF(m_ptStartPoint, m_ptEndPoint);
}


QPainterPath DLMultiPointItem::shape() const
{
	QPainterPath path;
	path.addRect(boundingRect());

	QPainterPathStroker stroker;
	stroker.setWidth(20);
	return stroker.createStroke(path);
	
}


// void DLMultiPointItem::cal_show_area(const QRectF &rect)
// {
// 
// 	m_vUnitSquares.clear();
// 
// 	//1、根据最大的地图区域，计算划分10*10米方块
// 	double unit_width = 1000;
// 	double unit_height = 1000;
// 	double topleft_x = rect.topLeft().x();
// 	double topleft_y = rect.topLeft().y();
// 	double bottomright_x = rect.bottomRight().x();
// 	double bottomright_y = rect.bottomRight().y();
// 
// 	topleft_x = topleft_x > 0 ? int(topleft_x / 1000) * 1000 : int(topleft_x / 1000 - 1) * 1000;
// 	topleft_y = topleft_y > 0 ? int(topleft_y / 1000) * 1000 : int(topleft_y / 1000 - 1) * 1000;
// 	bottomright_x = bottomright_x > 0 ? int(bottomright_x / 1000 + 1) * 1000 : int(bottomright_x / 1000) * 1000;
// 	bottomright_y = bottomright_y > 0 ? int(bottomright_y / 1000 + 1) * 1000 : int(bottomright_y / 1000) * 1000;
// 
// 	m_optimizationPro.map_max_rect_ = rect;
// 	m_optimizationPro.row_ = abs(topleft_y - bottomright_y) / 1000;
// 	m_optimizationPro.column_ = abs(topleft_x - bottomright_x) / 1000;
// 
// 
// 	for (int row = 0; row < m_optimizationPro.row_; ++row)
// 	{
// 		for (int colomn = 0; colomn < m_optimizationPro.column_; ++colomn) 
// 		{
// 			UnitRectPro pro;
// 			double temp_topleft_x = topleft_x + colomn * unit_width;
// 			double temp_topleft_y = topleft_y + row * unit_height;
// 			pro.unit_square_ = QRectF(temp_topleft_x, temp_topleft_y, unit_width, unit_height);
// 
// 			for (int i = 0; i < m_vPoints.size(); ++i)
// 			{
// 				if (!pro.unit_square_.contains(m_vPoints[i]))
// 					continue;
// 				else
// 					pro.points_.push_back(m_vPoints[i]);
// 			}
// 			m_vUnitSquares.push_back(pro);
// 		}
// 	}
// }


//  void DLMultiPointItem::slot_pro(const QRectF &rect)
//  {
//  	if (m_vPoints.size() < MAX_POINT_SIZE) {
//  		qDebug() << "if (point_list_.size() < MAX_POINT_SIZE)";
//  		return;
//  	}
//  
//  	m_vDrawPoints.clear();
//  	m_rectLastVeiwpot = rect;
//  
//  	QRectF intersected_rectangle = rect.intersected(m_optimizationPro.map_max_rect_);
//  	if (abs(intersected_rectangle.width() * intersected_rectangle.height()) < 0.0000001) {
//  		update();
//  		qDebug() << "intersected_rectangle ============" << intersected_rectangle;
//  		return;
//  	}
//  	qDebug() << "void DLMultiPointItem::slot_pro(const QRectF &rect)";
//  
//  	//视口矩形属性
//  	int topleft_index(0);
//  	int bottomright_index(0);
//  	bool first_contain = false;
//  	bool last_contain = false;
//  
//  	//正向查找第一个符合条件的矩形框
//  	for (int i = 0; i < m_vUnitSquares.size(); ++i)
//  	{
//  		//记录下第一次找到的单位矩形的坐标
//  		first_contain = m_vUnitSquares[i].unit_square_.contains(intersected_rectangle.topLeft());
//  		if (first_contain) {
//  			topleft_index = i;
//  			break;
//  		}
//  	}
//  
//  	//逆向查找第一个符合条件的矩形框
//  	for (int i = m_vUnitSquares.size() - 1; i >= 0; --i)
//  	{
//  		//记录下第一次找到的单位矩形的坐标
//  		last_contain = m_vUnitSquares[i].unit_square_.contains(intersected_rectangle.bottomRight());
//  		if (last_contain) {
//  			bottomright_index = i;
//  			break;
//  		}
//  	}
//  
//  	for (int i = topleft_index; i < bottomright_index; i++)
//  	{
//  		m_vDrawPoints.append(m_vUnitSquares[i].points_);
//  	}
//  	update();
//  }


void DLMultiPointItem::setMutilPointItem(const QVector<QPointF> & points)
{
	m_vPoints = points;
}


QVector<QPointF> DLMultiPointItem::getMutilePoint()
{
	return m_vPoints;
}


void DLMultiPointItem::SetBoundingRect(QPointF ptStarttPoint, QPointF ptEndPointF)
{
	m_ptStartPoint = ptStarttPoint;
	m_ptEndPoint = ptEndPointF;
}

void DLMultiPointItem::SetDisplayBackground(bool bIsShow)
{
	m_bIsShowShape = bIsShow;
}

void DLMultiPointItem::SetAllAreaPoints(QVector<QVector<AreaInfo>> vAreaPoints)
{
	m_vRectAreaPoints = vAreaPoints;
}

void DLMultiPointItem::UpdateArea(QPointF ptCurStartPoint, QPointF ptcurEndPoint)
{
	m_ptCurStartPoint = this->mapFromScene(ptCurStartPoint);
	m_ptCurEndPoint = this->mapFromScene(ptcurEndPoint);
	
	if (m_ptCurEndPoint.x() < m_ptStartPoint.x() || m_ptCurEndPoint.y() < m_ptStartPoint.y())
	{
		m_bIsShowShape = false;
	}
	else
	{
		m_bIsShowShape = true;
	}

	if (m_ptCurStartPoint.x() < m_ptStartPoint.x())
	{
		m_ptCurStartPoint.setX(m_ptStartPoint.x());
	}

	if (m_ptCurStartPoint.y() < m_ptStartPoint.y())
	{
		m_ptCurStartPoint.setY(m_ptStartPoint.y());
	}

	if (m_ptCurEndPoint.x() > m_ptEndPoint.x())
	{
		m_ptCurEndPoint.setX(m_ptEndPoint.x());
	}

	if (m_ptCurEndPoint.y() > m_ptEndPoint.y())
	{
		m_ptCurEndPoint.setY(m_ptEndPoint.y());
	}

	CalcDrawArea();
}

void DLMultiPointItem::SetBackgroundingPixmap(QPixmap pixmap)
{
	m_pixmap = m_pixmap;
}

void DLMultiPointItem::paint( QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget /*= 0*/ )
{
	const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
	if (m_bIsShowShape)
	{
		painter->save();
		painter->setPen(QPen(Qt::lightGray, 5, Qt::SolidLine));
		painter->drawRect(boundingRect());
		painter->restore();

		painter->save();

		int iCellCount = CELL_COUNT;
		if (lod <= 0.5)
		{
			iCellCount = CELL_COUNT;
		}
		else if (lod <= 1)
		{
			iCellCount = 200;
		}
		else if (lod >= 1)
		{
			iCellCount = 300;
		}
		else if (lod >= 1.5)
		{
			iCellCount = 400;
		}
		else if (lod >= 2)
		{
			iCellCount = 500;
		}

		for (int iRowIndex = m_iRowStartIndex; iRowIndex < m_vRectAreaPoints.size() && iRowIndex < m_iRowEndIndex; ++iRowIndex)
		{
			for (int iColumnIndex = m_iColumnStartIndex; iColumnIndex < m_vRectAreaPoints[iRowIndex].size() && iColumnIndex < m_iColumnEndIndex; ++iColumnIndex)
			{
				QVector<QPointF> vAreaPoints = m_vRectAreaPoints[iRowIndex][iColumnIndex].vPoints;
				int iStepIndex = vAreaPoints.size() / iCellCount;
				if (iStepIndex == 0)
				{
					iStepIndex++;
				}
				for (int iIndex = 0; iIndex < vAreaPoints.size(); iIndex += iStepIndex)
				{
					painter->drawRect(QRectF(vAreaPoints[iIndex].x() - 0.5, vAreaPoints[iIndex].y() - 0.5, 1, 1));
				}
			}
		}

	//	painter->drawPixmap()

		painter->restore();

	}
	//painter->setBrush(QBrush(Qt::black));

 //	if (!m_bIsShowPixmap)
// 	{
//		if (m_vPoints.size() < MAX_POINT_SIZE) {
// 		for (int i = 0; i < m_vPoints.size(); ++i)
// 		{
// 			painter->drawRect(QRectF(m_vPoints[i].x() - 0.5, m_vPoints[i].y() - 0.5, 1, 1));
// 		}
	//	}
// 		else {
// 
// 			int count = m_vDrawPoints.size();
// 			int step = count / 10000 + 1;
// 			if (count <= 10000) {
// 				for (int i = 0; i < m_vDrawPoints.size(); ++i)
// 				{
// 					painter->drawRect(QRectF(m_vDrawPoints[i].x() - 0.5, m_vDrawPoints[i].y() - 0.5, 1, 1));
// 				}
// 			}
// 			if (count > 10000) {
// 				for (int i = 0; i < m_vDrawPoints.size(); i += step)
// 				{
// 					painter->drawRect(QRectF(m_vDrawPoints[i].x() - step * 0.5, m_vDrawPoints[i].y() - step * 0.5, step, step));
// 				}
// 			}
// 		}
//	}





}

void DLMultiPointItem::CalcDrawArea()
{
	m_iRowStartIndex = (m_ptCurStartPoint.y() - m_ptStartPoint.y()) / CELL_HEIGHT;
	m_iRowEndIndex = (m_ptCurEndPoint.y() - m_ptStartPoint.y()) / CELL_HEIGHT;

	m_iColumnStartIndex = (m_ptCurStartPoint.x() - m_ptStartPoint.x()) / CELL_WIDTH;
	m_iColumnEndIndex = (m_ptCurEndPoint.x() - m_ptStartPoint.x()) / CELL_WIDTH;

	--m_iRowStartIndex;
	--m_iColumnStartIndex;

	if (m_iRowStartIndex < 0)
	{
		m_iRowStartIndex = 0;
	}
	if (m_iColumnStartIndex < 0)
	{
		m_iColumnStartIndex = 0;
	}

	m_iRowEndIndex++;
	m_iColumnEndIndex++;

}

