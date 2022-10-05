#ifndef DLBACKSTAGEMAPWIDGET_ALEXWEI_2018_06_05_H
#define DLBACKSTAGEMAPWIDGET_ALEXWEI_2018_06_05_H

#include <QPointF>
#include <QMutex>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QDialog>
#include <QListWidget>
#include "LibMapReader/MapData.h"
#include "LibDlToolItems/DLShapeItem.h"
#include "common/DLWheelRobotGlobalDef.hpp"

class DLCoordinateItem;
class MapReader;
class QLabel;
class QPushButton;
class MapReaderInfoWidget;
class QWidget;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;
class QLabel;
class DLRobotItem;
class DLPathItem;
class QTimer;
class QGraphicsRectItem;
class DLPlatformItem;
class DLDeviceViewer;

class DLBackStageMapView : public QGraphicsView
{
	Q_OBJECT
public:
	DLBackStageMapView(QWidget *parent = 0);
	void initView();
	void OpenDefaultMap();			//打开默认地图

protected:
	void keyPressEvent(QKeyEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);

signals:
	void sig_changed();
	void DrawStateChangedSignal(bool);					//绘制状态的信号


private:
	QPushButton * center_pushbutton_;
	QPushButton *follow_pushbutton_;
	QPushButton *open_pushbutton_;
	QPushButton *m_pDrawPathBtn;							//绘制路劲的按钮
	//QPushButton *m_pCutBgBtn;

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
	void slot_on_open_map();

	void ExitDrawStateSlot();			//退出绘制状态的槽函数
	void DrawPathBtnSlot(bool);
	void CutBgBtnSlot(bool bIsChecked);

private:
	Qt::MouseButton translate_button_;  // 平移按钮
	qreal translate_speed_;  // 平移速度
	qreal zoom_delta_;  // 缩放的增量
	bool bMouse_translate_;  // 平移标识
	QPoint last_mousePos_;  // 鼠标最后按下的位置
	qreal scale_;  // 缩放值
	bool viewPort_mode_;	//视口调整模式

};
#endif // !






