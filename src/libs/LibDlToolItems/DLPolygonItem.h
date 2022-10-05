#ifndef DLPOLYGONITEM_ALEXWEI_20180428_H
#define DLPOLYGONITEM_ALEXWEI_20180428_H

#include "HQShapeItem.h"
#include <QPainterPath>

class DLControlTransformItem;
class DLSegmentItem;

class DLPolygonItem;
class QLabel;
class QLineEdit;
class QCheckBox;
class QGroupBox;


class PolygonPropertyWidget : public QWidget
{
	Q_OBJECT
public:
	PolygonPropertyWidget();
	~PolygonPropertyWidget();

	void keyPressEvent(QKeyEvent *event);
	void setAdvancedAreaProperty();
	void setParent(DLPolygonItem *item);

private slots:
	void slot_on_set_advancedArea_info();

private:
	QGroupBox * advancedArea_groupBox_;

	QGroupBox *advancedArea_base_groupBox_;
	QLabel *className_label_;
	QLineEdit *className_lineEdit_;
	QLabel *intanceId_label_;
	QLineEdit *instanceId_lineEdit_;

	QGroupBox *advancedArea_advanced_groupBox_;
	QCheckBox *ultrasonic_checkBox_;
	QCheckBox *fallingdown_checkBox_;
	QCheckBox *gyroCaliLine_checkBox_;
	QCheckBox *laserDevice_checkBox_;
	QCheckBox *collisionDevice_checkBox_;

	QPushButton *save_info_buttom_;

private:
	DLPolygonItem * parent_;


};




class DLPolygonItem : public HQShapeItem
{
	Q_OBJECT

public:
	enum LineProperty{
		NormalLine = -1,
		ForbiddenLine = 0,
		VirtualLine = 1
	};
	enum { Type = HShape::POLYGON};
	int type() const
	{
		return Type;
	}
public:

	DLPolygonItem(QVector<QPointF> points, int id, std::string className, std::string instanceName, LineProperty lineClassName = NormalLine);
	DLPolygonItem(QVector<QPointF> points, int id, LineProperty lineClassName = NormalLine);
	~DLPolygonItem();

public:
	//void set_start_item(HQShapeItem *item);
	std::string className();
	std::string instanceName();
	int id();
	void set_className(std::string className) {
		className_ = className;
	}
	void set_instanceName(std::string instanceName) {
		instanceName_ = instanceName;
	}
	void set_id(int id) {
		id_ = id;
	}

	QVector<QPointF> get_pro();
	
	void setPropertyList(std::vector<Property> PropertyVec) {
		PorpertyList_.clear();
		std::vector<Property>::iterator it;
		for (it = PropertyVec.begin(); it != PropertyVec.end(); ++it)
		{
			Property tempPro;
			tempPro.key = it->key;
			tempPro.type = it->type;
			tempPro.value = it->value;
			PorpertyList_.push_back(tempPro);
		}
	}

	std::vector<Property>& getPropertyList() {
		return PorpertyList_;
	}

	void setISFixedList(std::vector<Add_IsFixed> isFixedList)
	{
		m_isFixedList.clear();
		std::vector<Add_IsFixed>::iterator it;
		for (it = isFixedList.begin(); it != isFixedList.end(); ++it)
		{
			Add_IsFixed tempPro;
			tempPro.key = it->key;
			tempPro.type = it->type;
			tempPro.value = it->value;
			m_isFixedList.push_back(tempPro);
		}
	}

	std::vector<Add_IsFixed>& getIsFixedInfoList()
	{
		return m_isFixedList;
	}
	
public:

	QRectF boundingRect() const;
	QPainterPath shape() const;
	void paint(QPainter *painter,const QStyleOptionGraphicsItem *option,QWidget * widget = 0);

public slots:
	void slot_pro(QVector<QPointF> points);//外部设置属性

signals:
	void sig_pro(QVector<QPointF> points);//属性变化通知外部

public:
	void update_shape();//供子item调用
	void set_control_point_visible(bool visible);

public:
	void mousePressEvent(QGraphicsSceneMouseEvent * event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent * event);
	void mouseReleaseEvent( QGraphicsSceneMouseEvent * event);
	void mouseDoubleClickEvent(QGraphicsSceneMouseEvent * event);

private:
	//接口不开放
	void set_polygon_pro(QVector<QPointF> points);//
	void set_control_pro(QVector<QPointF> points);


private:
	int id_;
	std::string className_;
	std::string instanceName_;
	LineProperty lineClassName_;
	QVector<QPointF> cal_pro();//控制点的移动导致形状的变化


private:
	QVector<QPointF> points_;
	QVector<QPointF> last_points_;
	QVector<DLControlTransformItem*> control_item_vec_;
	std::vector<Property> PorpertyList_;
	std::vector<Add_IsFixed> m_isFixedList;
};


#endif