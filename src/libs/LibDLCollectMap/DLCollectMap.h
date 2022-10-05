#ifndef DLCOLLECTMAP_ALEXWEI_2018_06_05_H
#define DLCOLLECTMAP_ALEXWEI_2018_06_05_H


#include <QGraphicsView>
#include <QGraphicsScene>
#include "LibMapReader/MapData.h"
#include "LibDlToolItems/DLShapeItem.h"
#include <QWidget>
#include <QMutex>
#include <QTime>
#include <boost/thread.hpp>
//#include "common/DLWheelRobotGlobalDef.hpp"



class DLMultiPointItem;
class DLCoordinateItem;
class DLGridItem;
class QGraphicsLineItem;
class MapReader;
class QLabel;
class MapReaderInfoWidget;
class QWidget;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;
class QLabel;
class QPushButton;
class QTimer;
class DLRobotItem;
class DLLaserItem;
class DLPathItem;
class DL3DRobotViewer;
class CustomTableWidget;
class DLPointInfoWidget;
class QTableWidget;
struct RobotLocateInfo
{
public:
	RobotLocateInfo(double x = 0, double y = 0, double angle = 0) 
		:x_(x), y_(y), angle_(angle)
	{
	}

	double x_;
	double y_;
	double angle_;
};


struct DeviceDetail
{
	QString deviceUUid;
	QString VoltageLevel;
	QString equipmentInterval;
	QString deviceArea;
	QString deviceType;
	QString subDevice;
	QString devicePointType;
};




class DLPointInfoWidget : public QWidget
{
	Q_OBJECT
public:
	DLPointInfoWidget(QWidget *parent = 0);
	void initWidget();
	void loadData(QList<DeviceDetail> data);

private:
	QTableWidget *tableWidget_;


};



class DLCollectMapView : public QGraphicsView
{
	Q_OBJECT
public:
	DLCollectMapView(QWidget *parent = 0);
	void initView();
	void SetResolution(qreal rResolution);

protected:
	void keyPressEvent(QKeyEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	virtual void showEvent(QShowEvent *event);

signals:
	void sig_changed();
    void sig_add_patrolPoint();
    void sig_open_smap(QString);

signals:
	void sig_viewRect_changed(const QRectF &rect);
private:
	QPushButton *zoom_in_pushbutton_;
	QPushButton *zoom_out_pushbutton_;
	QPushButton *center_pushbutton_;
    QPushButton *add_patrol_point_pushbutton_;
	QPushButton *viewPort_mode_pushbutton_;
	QPushButton *three_d_view_pushbutton_;


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
	void slot_on_center();
	void slot_on_flush_viewPort(QPointF point);
	void slot_on_viewPortMode(bool mode);
	void slot_on_view3d();

private:
	Qt::MouseButton translate_button_;  // ƽ�ư�ť
	qreal translate_speed_;  // ƽ���ٶ�
	qreal zoom_delta_;  // ���ŵ�����
	bool bMouse_translate_;  // ƽ�Ʊ�ʶ
	QPoint last_mousePos_;  // �������µ�λ��
	qreal scale_;  // ����ֵ
	bool viewPort_mode_;	//�ӿڵ���ģʽ
	qreal m_rResolution;
	QLabel *m_pResolutionNameLbl;
	QLabel *m_pResolutionLbl;
	bool m_bIsFirstIn;

};
#endif // !DLCOLLECTMAP_ALEXWEI_2018_06_05_H






