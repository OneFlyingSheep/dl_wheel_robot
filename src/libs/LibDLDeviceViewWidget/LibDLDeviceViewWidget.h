#ifndef DLDEVICEVIEWWIDGET_ALEXWEI_2018_06_05_H
#define DLDEVICEVIEWWIDGET_ALEXWEI_2018_06_05_H

#include <QPointF>
#include <QGraphicsView>
#include <QGraphicsScene>
#include "LibDlToolItems/DLShapeItem.h"
#include "common/DLWheelRobotGlobalDef.hpp"

class QGraphicsItem;


class DLDeviceGraphcisView : public QGraphicsView
{
	Q_OBJECT
public:
	DLDeviceGraphcisView(QWidget *parent = 0);
	void initView();

protected:
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);

signals:
	void sig_changed();
	

public:
	//平移速度
	void setTranslateSpeed(double speed);
	double translateSpeed();

	//缩放增量
	void  setZoomDelta(double delta);
	double zoomDelta();


public slots:
	void zoomIn();  // 放大
	void zoomOut();  // 缩小
	void zoom(float scaleFactor); // 缩放 - scaleFactor：缩放的比例因子
	void translate(QPointF delta);  // 平移


private:
	Qt::MouseButton translate_button_;  // 平移按钮
	qreal translate_speed_;  // 平移速度
	qreal zoom_delta_;		// 缩放的增量
	bool bMouse_translate_;  // 平移标识
	QPoint last_mousePos_;  // 鼠标最后按下的位置
	qreal scale_;			// 缩放值
	bool viewPort_mode_;	//视口调整模式

};

////////////////////////////////////////


class DLDeviceGraphicsScene : public QGraphicsScene
{
	Q_OBJECT
public:
	DLDeviceGraphicsScene(QObject *parent = 0);
	~DLDeviceGraphicsScene();

public:
	//QPointF get_center();
	//void removeall();
	void loadDevice();
	void setDevice(WheelInspectResultStruct current_task_info);

protected:
	void mousePressEvent(QGraphicsSceneMouseEvent *event);
	void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
	void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

private:
	QMap<QString, QGraphicsItem*> device_list_;

};


class DLDeviceViewer : public QWidget
{
	Q_OBJECT

public:
	DLDeviceViewer(QWidget *parent = 0);
	void loadDevice();

	void set_device_propertry(WheelInspectResultStruct current_task_info);

private:
	DLDeviceGraphcisView * view_;
	DLDeviceGraphicsScene * scene_;
};




#endif // !






