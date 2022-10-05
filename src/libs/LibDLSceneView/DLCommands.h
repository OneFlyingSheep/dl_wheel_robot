#ifndef DLCOMMANDS_ALEXWEI_20180420_H
#define DLCOMMANDS_ALEXWEI_20180420_H

#include <QUndoCommand>
#include <QPointF>
#include <QLine>



class QGraphicsScene;
class DLLandmarkItem;
class DLBezierItem;
class QGraphicsLineItem;
class QGraphicsSceneMouseEvent;
class QGraphicsItem;
class QGraphicsRectItem;
class DLDeviceItem;
class HQShapeArcItem;
class DLRelocateProcesser;
class DLPictureItem;
class DLPolygonItem;
class DLSegmentItem;
class DLDeviceAreaItem;
class DLRelocateItem;
class HQShapeItem;
class QWidget;
class QLabel;
class QPushButton;
class DLAdvancedAreaItem;


class DLUndoCommandInterface:public QObject, public QUndoCommand
{
	Q_OBJECT

public:
	DLUndoCommandInterface(QGraphicsScene *scene);
	DLUndoCommandInterface(QGraphicsScene *scene, int lineType);

	
public:
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	QGraphicsScene *scene(){
		return scene_;
	}
	
	// «∑Ò”––ß
	virtual bool is_useful(){
		return true;
	}

	
public:
	QGraphicsScene *scene_;
	int type_;
};


class DLCoordinateTransformCommand : public DLUndoCommandInterface
{
	Q_OBJECT

public:
	DLCoordinateTransformCommand(QGraphicsScene *scene);
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	bool isPress_;
	QLineF start_line_;
	QLineF end_line_;
	double point_angle;

};


class DLAddDeviceCommand :public DLUndoCommandInterface
{

public:

	DLAddDeviceCommand(QGraphicsScene *scene);
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

public:


};


class DLRelocationCommand : public DLUndoCommandInterface
{
	Q_OBJECT

public:
	DLRelocationCommand(QGraphicsScene *scene);

	void RobotRelcation(QPointF &ptRobotPos, double dAngle);

public:
    int getRelocateRetVal();

signals:
	void signal_confirm_relocation();

protected:
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private slots:
	void slot_release_thread();
	void slot_get_relocate_state(int state);
	void slot_confirm_relocate();

private:
	QPushButton * confirm_relocate_button_;
	QLabel * relocate_tip_label_;
	QWidget *relocate_tip_widget_;

private:
	DLRelocateProcesser *relocateProcesser_;
	QPointF start_point_;
	QPointF end_point_;
	double point_angle;
	DLRelocateItem *relocate_item_;
	bool isPress;

};


class DLAddArcCommand:public DLUndoCommandInterface
{

public:

	DLAddArcCommand(QGraphicsScene *scene);
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

public:


private:
	HQShapeArcItem *item_;
};


class DLAddLineCommand : public DLUndoCommandInterface
{
public:
	DLAddLineCommand(QGraphicsScene *scene, int type);
	void undo();
	void redo();

	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	QGraphicsLineItem *virtual_item_;
	DLSegmentItem * item_; 
	QPointF start_point_;
	QPointF end_point_;
};


class DLAddPolygonCommand : public DLUndoCommandInterface
{
public:
	DLAddPolygonCommand(QGraphicsScene *scene);
	void undo();
	void redo();

	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	QGraphicsRectItem *virtual_item_;
	DLAdvancedAreaItem *item_;
	QPointF topLeft_point_;
	QPointF bottomRight_point_;
};


class DLAddDevAreaCommand : public DLUndoCommandInterface
{
public:
	DLAddDevAreaCommand(QGraphicsScene *scene);
	void undo();
	void redo();

	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	QGraphicsRectItem * virtual_item_;
	HQShapeItem *item_;
	QPointF topLeft_point_;
	QPointF bottomRight_point_;
};


class DLAddSTationAreaCommand : public DLUndoCommandInterface
{
public:
	DLAddSTationAreaCommand(QGraphicsScene *scene);
	void undo();
	void redo();

	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	QGraphicsRectItem * virtual_item_;
	HQShapeItem *item_;
	QPointF topLeft_point_;
	QPointF bottomRight_point_;
};


class DLAddEdgeCommand:public DLUndoCommandInterface
{

public:

	DLAddEdgeCommand(QGraphicsScene *scene);

	void undo();
	void redo();

	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:

	QGraphicsLineItem *virtual_item_;
	DLBezierItem *bezier_item_;
	QList<DLBezierItem*> bezier_list_;
	QList<QGraphicsLineItem*> virtual_list_;

	QPointF start_point_;
	QPointF end_point_;

};


class DLAddLandmarkCommand:public DLUndoCommandInterface
{
	
public:

	DLAddLandmarkCommand(QGraphicsScene *scene);
	void undo();
	void redo();

	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	DLLandmarkItem *item_;

};


class DLAddPictureCommand:public DLUndoCommandInterface
{

public:
	DLAddPictureCommand(QGraphicsScene *scene);
	void undo();
	void redo();
	
	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	DLPictureItem *item_;


};


class DLSelectCommand:public DLUndoCommandInterface
{
public:
	DLSelectCommand(QGraphicsScene *scene);

private:
	void undo();
	void redo();

	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);
	
private:
	QPointF topleft_pos_;
	QGraphicsRectItem *rect_item_;


};


class DLDeleteCommand:public DLUndoCommandInterface
{

public:
	DLDeleteCommand(QGraphicsScene *scene);

	void undo();
	void redo();

	virtual bool is_useful();
	virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	QList<QGraphicsItem*> item_list_;
	QGraphicsRectItem *rect_item_;
};



#endif
