#ifndef DLSEGMENTITEM_ALEXWEI_20180428_H
#define DLSEGMENTITEM_ALEXWEI_20180428_H

#include "HQShapeItem.h"

class HQShapeItem;

class DLSegmentItem : public HQShapeItem
{//��ͨ����
	Q_OBJECT

public:
	enum LineProperty{
		NormalLine = -1,
		ForbiddenLine = 0,
		VirtualLine = 1
	};

	int id() {
		return id_;
	}

	enum { Type = HShape::SEGMENTITEM };
	int type() const
	{
		return Type;
	}

public:

	DLSegmentItem(HShape::Segment pro, int id, std::string className, std::string instanceName, LineProperty lineClassName = NormalLine);
	DLSegmentItem(HShape::Segment pro, int id, LineProperty lineClassName = NormalLine);
	~DLSegmentItem();

public:

	void set_start_item(HQShapeItem *item);
	std::string className();
	std::string instanceName();
	HShape::Segment get_pro();
	
	
public:

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

public slots:
	void slot_pro(HShape::Segment pro);//�ⲿ��������

signals:
	void sig_pro(HShape::Segment pro);//���Ա仯֪ͨ�ⲿ

public:

	void update_shape();//����item����
	void set_control_point_visible(bool visible);

public:

	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent *event);


private:
	//�ӿڲ�����
	void set_segment_pro(HShape::Segment pro);//
	void set_control_pro(HShape::Segment pro);
	//HShape::Segment get_pro();
	HShape::Segment cal_pro();//���Ƶ���ƶ�������״�ı仯

private:
	int id_;
	std::string className_;
	std::string instanceName_;
	LineProperty lineClassName_;
	HShape::Segment pro_;
	
private:

	HQShapeItem *start_item_;
	HQShapeItem *end_item_;
	QPointF last_start_point_;
	QPointF last_end_point_;

};


#endif