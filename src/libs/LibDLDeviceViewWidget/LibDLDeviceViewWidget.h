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
	//ƽ���ٶ�
	void setTranslateSpeed(double speed);
	double translateSpeed();

	//��������
	void  setZoomDelta(double delta);
	double zoomDelta();


public slots:
	void zoomIn();  // �Ŵ�
	void zoomOut();  // ��С
	void zoom(float scaleFactor); // ���� - scaleFactor�����ŵı�������
	void translate(QPointF delta);  // ƽ��


private:
	Qt::MouseButton translate_button_;  // ƽ�ư�ť
	qreal translate_speed_;  // ƽ���ٶ�
	qreal zoom_delta_;		// ���ŵ�����
	bool bMouse_translate_;  // ƽ�Ʊ�ʶ
	QPoint last_mousePos_;  // �������µ�λ��
	qreal scale_;			// ����ֵ
	bool viewPort_mode_;	//�ӿڵ���ģʽ

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






