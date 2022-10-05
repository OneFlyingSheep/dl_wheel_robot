#ifndef DLMULTIPOINTITEM_ALEXWEI_20180423_H
#define DLMULTIPOINTITEM_ALEXWEI_20180423_H

#pragma execution_character_set("utf-8")

#include "DLShapeItem.h"
#include <QVector>
#include <QPolygonF>

struct MapOptimizationPro 
{
	int row_;
	int column_;
	QRectF map_max_rect_;
public:
	MapOptimizationPro() {
		row_ = 0;
		column_ = 0;
	}
};

struct UnitRectPro 
{
	QRectF unit_square_;
	QVector<QPointF> points_;
};


struct AreaInfo;

class DLMultiPointItem : public DLShapeItem
{

	Q_OBJECT

public:
	int type() const {
		return HShape::MUTILPOINT;
	}

	DLMultiPointItem(QGraphicsItem *parent = NULL);
	~DLMultiPointItem();
	void setTransformAble(bool state = false);
	void setMutilPointItem(const QVector<QPointF> & points);
	QVector<QPointF> getMutilePoint();
	//void SetIsShowBackgroundImage(bool bIsShowBg);

	void SetBoundingRect(QPointF ptStarttPoint, QPointF ptEndPointF);
	void SetDisplayBackground(bool bIsShow);
	void SetAllAreaPoints(QVector<QVector<AreaInfo>> vAreaPoints);

	void UpdateArea(QPointF ptCurStartPoint, QPointF ptcurEndPoint);


	void SetBackgroundingPixmap(QPixmap pixmap);


protected:
	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

private:
	//void cal_show_area(const QRectF &rect);

	void CalcDrawArea();

public slots:
	//void slot_pro(const QRectF &rect);

private:
	QVector<QPointF> m_vPoints;
	QVector<QPointF> m_vDrawPoints;
	bool m_bIsShowShape;
	//bool m_bIsShowPixmap;									//¼ÇÂ¼ÊÇ·ñ¼ÓÔØ±³¾°Í¼Æ¬

	QPointF m_ptStartPoint;
	QPointF m_ptEndPoint;
	bool m_bPressed;

	QVector<QVector<AreaInfo>> m_vRectAreaPoints;
	QPointF m_ptCurStartPoint;
	QPointF m_ptCurEndPoint;

	int m_iRowStartIndex;
	int m_iRowEndIndex;
	int m_iColumnStartIndex;
	int m_iColumnEndIndex;

	QPixmap m_pixmap;
};


#endif