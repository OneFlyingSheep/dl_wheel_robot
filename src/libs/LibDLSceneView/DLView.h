#ifndef DLVIEW_ALEXWEI_20180418_H
#define DLVIEW_ALEXWEI_20180418_H

#include <QGraphicsView>
#include <QFrame>

class QWidget;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;
class QLabel;
class QPushButton;
class QTimer;
class DirectionButton;
class DLCustomScene;

struct ScaleInfo {
	QPointF point_;
	bool flag_;	// 1����5�ı�����2����10�ı���
};

class DLView : public QGraphicsView
{
	Q_OBJECT
public:
	DLView(QWidget *parent = 0);
	void initView();
	std::vector<ScaleInfo> get_horizon();
	std::vector<ScaleInfo> get_vertical();
	void SetCutOutState(bool bIsCutOutState);

protected:
	void keyPressEvent(QKeyEvent *event);
	void mouseMoveEvent(QMouseEvent *event);
	void mousePressEvent(QMouseEvent *event);
	void mouseReleaseEvent(QMouseEvent *event);
	void wheelEvent(QWheelEvent *event);
	void drawBackground(QPainter *painter, const QRectF &rect);
	void paintEvent(QPaintEvent *event);

signals:
	void sig_changed();

signals:
	void sig_viewRect_changed(const QRectF &rect);

private:
	QPushButton *zoom_in_pushbutton_;
	QPushButton *zoom_out_pushbutton_;
	QPushButton *center_pushbutton_;
	QPushButton *three_d_view_pushbutton_;

	///////////////////////////////////////////
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

private:
	Qt::MouseButton translate_button_;  // ƽ�ư�ť
	qreal translate_speed_;  // ƽ���ٶ�
	qreal zoom_delta_;  // ���ŵ�����
	bool bMouse_translate_;  // ƽ�Ʊ�ʶ
	QPoint last_mousePos_;  // �������µ�λ��
	qreal scale_;  // ����ֵ

private:
	std::vector<ScaleInfo> horizon_vec_;		//��¼ˮƽ�ĵ�
	std::vector<ScaleInfo> vertical_vec_;		//��¼��ֱ�ĵ�
	bool m_bIsCutoutState;
	bool m_bIsPressed;
	QPoint m_ptStartPoint;
	QPoint m_ptEndPoint;
};

class DLFrameView : public QFrame
{
	Q_OBJECT
public:
	DLFrameView();
	~DLFrameView();
	void setScene(DLCustomScene* scene);
	void SetCutOutState(bool bIsCutOutState);

protected:
	void paintEvent(QPaintEvent *e);

private:
	DLView *view_;

};



#endif