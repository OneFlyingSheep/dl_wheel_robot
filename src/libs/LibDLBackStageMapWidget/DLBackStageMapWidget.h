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
	void OpenDefaultMap();			//��Ĭ�ϵ�ͼ

protected:
	void keyPressEvent(QKeyEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);

signals:
	void sig_changed();
	void DrawStateChangedSignal(bool);					//����״̬���ź�


private:
	QPushButton * center_pushbutton_;
	QPushButton *follow_pushbutton_;
	QPushButton *open_pushbutton_;
	QPushButton *m_pDrawPathBtn;							//����·���İ�ť
	//QPushButton *m_pCutBgBtn;

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
	void slot_on_open_map();

	void ExitDrawStateSlot();			//�˳�����״̬�Ĳۺ���
	void DrawPathBtnSlot(bool);
	void CutBgBtnSlot(bool bIsChecked);

private:
	Qt::MouseButton translate_button_;  // ƽ�ư�ť
	qreal translate_speed_;  // ƽ���ٶ�
	qreal zoom_delta_;  // ���ŵ�����
	bool bMouse_translate_;  // ƽ�Ʊ�ʶ
	QPoint last_mousePos_;  // �������µ�λ��
	qreal scale_;  // ����ֵ
	bool viewPort_mode_;	//�ӿڵ���ģʽ

};
#endif // !






