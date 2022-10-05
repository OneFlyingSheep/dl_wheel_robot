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
	void slot_on_center();
	void slot_on_flush_viewPort(QPointF point);
	void slot_on_viewPortMode(bool mode);
	void slot_on_view3d();

private:
	Qt::MouseButton translate_button_;  // 平移按钮
	qreal translate_speed_;  // 平移速度
	qreal zoom_delta_;  // 缩放的增量
	bool bMouse_translate_;  // 平移标识
	QPoint last_mousePos_;  // 鼠标最后按下的位置
	qreal scale_;  // 缩放值
	bool viewPort_mode_;	//视口调整模式
	qreal m_rResolution;
	QLabel *m_pResolutionNameLbl;
	QLabel *m_pResolutionLbl;
	bool m_bIsFirstIn;

};
#endif // !DLCOLLECTMAP_ALEXWEI_2018_06_05_H






