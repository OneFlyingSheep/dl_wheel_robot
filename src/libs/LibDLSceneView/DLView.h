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
	bool flag_;	// 1代表5的倍数，2代表10的倍数
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

private:
	Qt::MouseButton translate_button_;  // 平移按钮
	qreal translate_speed_;  // 平移速度
	qreal zoom_delta_;  // 缩放的增量
	bool bMouse_translate_;  // 平移标识
	QPoint last_mousePos_;  // 鼠标最后按下的位置
	qreal scale_;  // 缩放值

private:
	std::vector<ScaleInfo> horizon_vec_;		//记录水平的点
	std::vector<ScaleInfo> vertical_vec_;		//记录垂直的点
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